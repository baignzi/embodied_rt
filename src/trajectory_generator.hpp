// trajectory_generator.hpp — VLA动作→关节轨迹生成（声明）
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <string>

// ---- TrajectoryGenerator ----
// 接收VLA输出的7维动作指令(x,y,z,rx,ry,rz,gripper)，
// 通过RRT*(MoveIt2)或关节空间插值生成关节轨迹，
// 重采样为100Hz后发给实时控制器。
class TrajectoryGenerator : public rclcpp::Node {
public:
    TrajectoryGenerator();

    /// 构造完成后调用（shared_from_this 此时安全）
    void init();

private:
    void on_action(const std_msgs::msg::String::SharedPtr msg);

    /// 从JSON字符串解析7维动作
    static std::vector<double> parse_action(const std::string& json_str);

    /// MoveIt2 RRT*规划（有MoveIt时调用）
    bool plan_with_moveit(const std::vector<double>& action,
                          trajectory_msgs::msg::JointTrajectory& out_traj);

    /// 关节空间线性插值（standalone降级模式）
    void plan_linear_fallback(const std::vector<double>& action,
                              trajectory_msgs::msg::JointTrajectory& out_traj);

    /// 重采样到固定dt
    static void resample(trajectory_msgs::msg::JointTrajectory& traj, double dt);

    // MoveIt2接口（可选，编译期决定）
    struct MoveItImpl;
    std::shared_ptr<MoveItImpl> moveit_;

    // 当前关节角（用于线性插值起点）
    std::vector<double> current_joints_;

    // Linear fallback 参数
    double fallback_scale_;          ///< 动作增量缩放因子
    double fallback_duration_;       ///< 轨迹持续时间（秒）
    double fallback_blend_ratio_;    ///< 加减速段占比（0~0.5）
    double traj_dt_;                 ///< 轨迹时间步长（秒），即重采样频率

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
};
