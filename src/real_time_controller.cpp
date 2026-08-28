// real_time_controller.cpp — 1000Hz实时控制回路
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <mutex>
#include <vector>
#include <algorithm>
#include <cmath>

struct PIDGains {
    double kp, ki, kd;
    double integral{0.0};
    double prev_error{0.0};
    double max_output;
};

class RealTimeController : public rclcpp::Node {
public:
    RealTimeController() : Node("real_time_controller") {
        // Franka Panda 7-DOF PID参数
        std::vector<double> default_kp = {50.0, 45.0, 40.0, 35.0, 30.0, 25.0, 20.0};
        std::vector<double> default_ki = {0.5, 0.4, 0.3, 0.2, 0.1, 0.1, 0.05};
        std::vector<double> default_kd = {2.0, 1.8, 1.5, 1.0, 0.8, 0.5, 0.3};
        std::vector<double> default_max = {3.0, 2.5, 2.0, 1.5, 1.0, 0.8, 0.5};

        // 声明参数（可通过YAML覆盖）
        this->declare_parameter("kp", default_kp);
        this->declare_parameter("ki", default_ki);
        this->declare_parameter("kd", default_kd);
        this->declare_parameter("max_output", default_max);

        auto kp = this->get_parameter("kp").as_double_array();
        auto ki = this->get_parameter("ki").as_double_array();
        auto kd = this->get_parameter("kd").as_double_array();
        auto max_out = this->get_parameter("max_output").as_double_array();

        size_t n = kp.size();
        pid_.resize(n);
        current_state_.resize(n, 0.0);
        for (size_t i = 0; i < n; ++i) {
            pid_[i].kp = kp[i];
            pid_[i].ki = ki[i];
            pid_[i].kd = kd[i];
            pid_[i].max_output = max_out[i];
        }

        traj_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/planning/trajectory", 1,
            [this](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(traj_mtx_);
                latest_traj_ = *msg;
                traj_idx_ = 0;
                traj_received_ = true;
            });

        cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/control/joint_cmd", 10);

        // 1000Hz控制定时器（1ms周期）
        control_timer_ = create_wall_timer(
            std::chrono::microseconds(1000),
            [this]() { this->control_step(); });

        RCLCPP_INFO(get_logger(),
            "Real-time controller started: %zu DOF @ 1000 Hz", n);
    }

private:
    void control_step() {
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

        size_t n = std::min(target.positions.size(), pid_.size());

        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = now();
        cmd.name = {"panda_joint1", "panda_joint2", "panda_joint3",
                    "panda_joint4", "panda_joint5", "panda_joint6",
                    "panda_joint7"};

        for (size_t i = 0; i < n; ++i) {
            double error = target.positions[i] - current_state_[i];
            double deriv = (error - pid_[i].prev_error) * 1000.0;  // dt=1ms
            pid_[i].integral += error * 0.001;

            // 积分抗饱和
            if (pid_[i].ki > 0) {
                pid_[i].integral = std::clamp(pid_[i].integral,
                    -pid_[i].max_output / pid_[i].ki,
                     pid_[i].max_output / pid_[i].ki);
            }

            double output = pid_[i].kp * error
                          + pid_[i].ki * pid_[i].integral
                          + pid_[i].kd * deriv;
            output = std::clamp(output, -pid_[i].max_output, pid_[i].max_output);

            cmd.effort.push_back(output);
            cmd.position.push_back(target.positions[i]);
            pid_[i].prev_error = error;
            current_state_[i] = target.positions[i];  // 直接跟踪目标位置
        }

        cmd_pub_->publish(cmd);

        // 每1秒打印一次状态
        static int print_counter = 0;
        if (++print_counter >= 1000) {
            print_counter = 0;
            if (traj_received_) {
                RCLCPP_INFO(this->get_logger(),
                    "Control active: %zu joints, tracking trajectory", n);
                traj_received_ = false;
            }
        }
    }

    std::vector<PIDGains> pid_;
    std::vector<double> current_state_;
    trajectory_msgs::msg::JointTrajectory latest_traj_;
    std::mutex traj_mtx_;
    size_t traj_idx_{0};
    bool traj_received_{false};
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealTimeController>());
    rclcpp::shutdown();
    return 0;
}
