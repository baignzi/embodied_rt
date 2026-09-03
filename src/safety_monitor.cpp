// safety_monitor.cpp — 安全监控节点
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cmath>
#include <functional>
#include <vector>

class SafetyMonitor : public rclcpp::Node {
public:
    SafetyMonitor() : Node("safety_monitor") {
        // Franka Panda 7-DOF 默认限位
        std::vector<double> default_lower = {
            -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
        std::vector<double> default_upper = {
             2.8973,  1.7628,  2.8973,  3.0718,  2.8973,  5.0265,  2.8973};

        this->declare_parameter("joint_lower", default_lower);
        this->declare_parameter("joint_upper", default_upper);
        this->declare_parameter("max_velocity", 2.0);
        this->declare_parameter("max_effort", 3.0);

        joint_lower_ = this->get_parameter("joint_lower").as_double_array();
        joint_upper_ = this->get_parameter("joint_upper").as_double_array();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        max_effort_ = this->get_parameter("max_effort").as_double();

        estop_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/safety/estop", 1);

        reset_srv_ = create_service<std_srvs::srv::SetBool>(
            "/safety/reset",
            [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                    const std_srvs::srv::SetBool::Response::SharedPtr res) {
                if (req->data) {
                    estop_triggered_ = false;
                    first_check_ = true;
                    res->success = true;
                    res->message = "E-stop latched state cleared";
                    RCLCPP_INFO(get_logger(), "E-stop manually reset");
                } else {
                    res->success = false;
                    res->message = "Set data=true to clear e-stop latch";
                }
            });

        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/control/joint_cmd", 10,
            [this](sensor_msgs::msg::JointState::SharedPtr msg) {
                this->check(msg);
            });

        RCLCPP_INFO(get_logger(),
            "Safety monitor active: %zu joints, v_max=%.2f rad/s, tau_max=%.2f Nm",
            joint_lower_.size(), max_velocity_, max_effort_);
    }

private:
    void check(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (estop_triggered_) {
            return;
        }

        bool estop = false;
        size_t n = msg->position.size();

        // 确定当前时间戳：优先使用消息头，否则用节点当前时间
        rclcpp::Time current_time =
            (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0)
                ? rclcpp::Time(msg->header.stamp, RCL_ROS_TIME)
                : this->now();

        // 1. 关节限位检查
        for (size_t i = 0; i < n && i < joint_lower_.size(); ++i) {
            if (msg->position[i] < joint_lower_[i] ||
                msg->position[i] > joint_upper_[i]) {
                RCLCPP_ERROR(get_logger(),
                    "Joint %zu limit violation: pos=%.3f [%.3f, %.3f]",
                    i, msg->position[i], joint_lower_[i], joint_upper_[i]);
                estop = true;
            }
        }

        // 2. 速度检查（使用实际 dt）
        if (!first_check_ && !prev_positions_.empty() &&
            prev_positions_.size() == n) {
            double dt = (current_time - last_time_).seconds();
            if (dt > 0.0 && dt < 1.0) {
                for (size_t i = 0; i < n; ++i) {
                    double vel =
                        std::abs((msg->position[i] - prev_positions_[i]) / dt);
                    if (vel > max_velocity_) {
                        RCLCPP_ERROR(get_logger(),
                            "Joint %zu velocity limit: %.3f rad/s (max %.2f, dt=%.4f)",
                            i, vel, max_velocity_, dt);
                        estop = true;
                    }
                }
            }
        }
        prev_positions_ = msg->position;
        last_time_ = current_time;
        first_check_ = false;

        // 3. 力矩检查
        for (size_t i = 0; i < msg->effort.size(); ++i) {
            if (std::abs(msg->effort[i]) > max_effort_) {
                RCLCPP_ERROR(get_logger(),
                    "Joint %zu torque limit: %.3f Nm (max %.2f)",
                    i, std::abs(msg->effort[i]), max_effort_);
                estop = true;
            }
        }

        if (estop) {
            estop_triggered_ = true;
            std_msgs::msg::Bool estop_msg;
            estop_msg.data = true;
            estop_pub_->publish(estop_msg);
            RCLCPP_ERROR(get_logger(),
                "*** EMERGENCY STOP TRIGGERED (latched, call /safety/reset to clear) ***");
        }

        // 每2秒打印一次健康状态
        static int counter = 0;
        if (++counter >= 2000) {
            counter = 0;
            RCLCPP_DEBUG(get_logger(), "Safety check OK: %zu joints monitored", n);
        }
    }

    std::vector<double> joint_lower_, joint_upper_;
    double max_velocity_{2.0};
    double max_effort_{3.0};
    std::vector<double> prev_positions_;
    bool estop_triggered_{false};
    rclcpp::Time last_time_;
    bool first_check_{true};
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_srv_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitor>());
    rclcpp::shutdown();
    return 0;
}
