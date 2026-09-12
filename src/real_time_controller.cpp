// real_time_controller.cpp — 1000Hz实时控制回路
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <mutex>
#include <vector>
#include <algorithm>
#include <cmath>
#include "pid_controller.hpp"

class RealTimeController : public rclcpp::Node {
public:
    RealTimeController() : Node("real_time_controller") {
        // Franka Panda 7-DOF PID参数
        std::vector<double> default_kp = {50.0, 45.0, 40.0, 35.0, 30.0, 25.0, 20.0};
        std::vector<double> default_ki = {0.5, 0.4, 0.3, 0.2, 0.1, 0.1, 0.05};
        std::vector<double> default_kd = {2.0, 1.8, 1.5, 1.0, 0.8, 0.5, 0.3};
        std::vector<double> default_kff = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> default_max = {3.0, 2.5, 2.0, 1.5, 1.0, 0.8, 0.5};
        // 微分低通滤波系数：0.1 ≈ 18Hz截止频率 @ 1000Hz，抑制测量噪声
        std::vector<double> default_deriv_alpha = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

        // 声明参数（可通过YAML覆盖）
        this->declare_parameter("kp", default_kp);
        this->declare_parameter("ki", default_ki);
        this->declare_parameter("kd", default_kd);
        this->declare_parameter("kff", default_kff);
        this->declare_parameter("max_output", default_max);
        this->declare_parameter("deriv_filter_alpha", default_deriv_alpha);

        auto kp = this->get_parameter("kp").as_double_array();
        auto ki = this->get_parameter("ki").as_double_array();
        auto kd = this->get_parameter("kd").as_double_array();
        auto kff = this->get_parameter("kff").as_double_array();
        auto max_out = this->get_parameter("max_output").as_double_array();
        auto dfa = this->get_parameter("deriv_filter_alpha").as_double_array();

        size_t n = kp.size();
        pid_params_.resize(n);
        pid_state_.resize(n);
        current_state_.resize(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            pid_params_[i].kp = kp[i];
            pid_params_[i].ki = ki[i];
            pid_params_[i].kd = kd[i];
            pid_params_[i].kff = kff[i];
            pid_params_[i].max_output = max_out[i];
            pid_params_[i].deriv_filter_alpha = dfa[i];
        }

        traj_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/planning/trajectory", 1,
            [this](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(traj_mtx_);
                latest_traj_ = *msg;
                traj_idx_ = 0;
                traj_received_ = true;
            });

        estop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/safety/estop", 1,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    estop_triggered_ = true;
                    RCLCPP_WARN(get_logger(), "E-stop received, halting trajectory output");
                } else {
                    estop_triggered_ = false;
                    RCLCPP_INFO(get_logger(), "E-stop cleared, resuming trajectory output");
                }
            });

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 1,
            [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(state_mtx_);
                if (msg->position.empty()) return;
                if (!msg->name.empty() && msg->name.size() == msg->position.size()) {
                    for (size_t i = 0; i < msg->name.size(); ++i) {
                        for (size_t j = 0; j < cmd_msg_.name.size(); ++j) {
                            if (msg->name[i] == cmd_msg_.name[j] && j < current_state_.size()) {
                                current_state_[j] = msg->position[i];
                                break;
                            }
                        }
                    }
                } else {
                    const size_t n = std::min(msg->position.size(), current_state_.size());
                    for (size_t i = 0; i < n; ++i) {
                        current_state_[i] = msg->position[i];
                    }
                }
                joint_states_received_ = true;
            });

        cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/control/joint_cmd", 10);

        // 预分配JointState消息，避免1000Hz下重复分配字符串
        cmd_msg_.name = {"panda_joint1", "panda_joint2", "panda_joint3",
                         "panda_joint4", "panda_joint5", "panda_joint6",
                         "panda_joint7"};
        cmd_msg_.effort.resize(n);
        cmd_msg_.position.resize(n);

        // 动态计算 dt 用的时间戳初始化
        prev_time_ = now();

        // 1000Hz控制定时器（1ms周期）
        control_timer_ = create_wall_timer(
            std::chrono::microseconds(1000),
            [this]() { this->control_step(); });

        RCLCPP_INFO(get_logger(),
            "Real-time controller started: %zu DOF @ 1000 Hz", n);
    }

private:
    void control_step() {
        if (estop_triggered_) return;

        // 动态计算实际 dt（秒），首次使用默认 1ms
        const rclcpp::Time t_now = now();
        double dt = 0.001;
        if (!first_step_) {
            dt = (t_now - prev_time_).seconds();
            if (dt <= 0.0 || dt > 0.01) {
                dt = 0.001;  // 异常时回退到默认 1ms
            }
        }
        prev_time_ = t_now;
        first_step_ = false;

        trajectory_msgs::msg::JointTrajectoryPoint target;
        bool have_traj = false;

        {
            std::lock_guard<std::mutex> lk(traj_mtx_);
            if (!latest_traj_.points.empty()) {
                if (traj_idx_ < latest_traj_.points.size()) {
                    target = latest_traj_.points[traj_idx_];
                    traj_idx_++;
                    have_traj = true;
                } else {
                    // 轨迹执行完，保持最后位置
                    target = latest_traj_.points.back();
                    have_traj = true;
                }
            }
        }

        if (!have_traj) return;

        const size_t n = std::min(target.positions.size(), pid_params_.size());

        cmd_msg_.header.stamp = now();
        if (cmd_msg_.effort.size() != n) {
            cmd_msg_.effort.resize(n);
            cmd_msg_.position.resize(n);
        }

        const bool has_velocity = !target.velocities.empty();

        for (size_t i = 0; i < n; ++i) {
            const bool has_vel = (has_velocity && i < target.velocities.size());
            const double vel = has_vel ? target.velocities[i] : 0.0;
            double current;
            {
                std::lock_guard<std::mutex> lk(state_mtx_);
                current = joint_states_received_ ? current_state_[i] : target.positions[i];
            }
            cmd_msg_.effort[i] = compute_pid_output(
                pid_params_[i], pid_state_[i],
                target.positions[i], current, dt,
                has_vel, vel);
            cmd_msg_.position[i] = target.positions[i];
        }

        cmd_pub_->publish(cmd_msg_);

        // 每1秒打印一次状态
        if (++print_counter_ >= 1000) {
            print_counter_ = 0;
            if (traj_received_) {
                RCLCPP_INFO(this->get_logger(),
                    "Control active: %zu joints, tracking trajectory", n);
                traj_received_ = false;
            }
        }
    }

    std::vector<PIDParams> pid_params_;
    std::vector<PIDState> pid_state_;
    std::vector<double> current_state_;
    trajectory_msgs::msg::JointTrajectory latest_traj_;
    std::mutex traj_mtx_;
    std::mutex state_mtx_;                  ///< 保护 current_state_ 的互斥锁
    size_t traj_idx_{0};
    bool traj_received_{false};
    bool joint_states_received_{false};     ///< 是否收到过 /joint_states 反馈
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    sensor_msgs::msg::JointState cmd_msg_;  ///< 预分配的关节指令消息
    int print_counter_{0};                  ///< 状态打印计数器（避免static局部变量）
    std::atomic<bool> estop_triggered_{false}; ///< 急停触发标志
    rclcpp::Time prev_time_;                ///< 上次控制步时间戳，用于动态计算 dt
    bool first_step_{true};                 ///< 是否为首次控制步
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealTimeController>());
    rclcpp::shutdown();
    return 0;
}
