/**
 * @file trajectory_generator.hpp
 * @brief VLA 动作 → 关节轨迹生成（声明）
 *
 * 接收 VLA 输出的 7 维动作指令 (x, y, z, rx, ry, rz, gripper)，
 * 通过 RRT*（MoveIt2）或关节空间插值生成关节轨迹，
 * 重采样为 100Hz 后发给实时控制器。
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <string>

/**
 * @class TrajectoryGenerator
 * @brief ROS2 节点：将 VLA 动作指令转换为关节空间轨迹
 *
 * 工作流程：
 * 1. 订阅 VLA 输出的 7 维动作 JSON 字符串
 * 2. 优先使用 MoveIt2 RRT* 规划；不可用时降级为线性插值
 * 3. 将轨迹重采样为固定 dt（默认 100Hz）
 * 4. 发布到 /trajectory/joint 目标供实时控制器消费
 */
class TrajectoryGenerator : public rclcpp::Node {
public:
    /**
     * @brief 构造函数，声明参数和 topic 接口
     */
    TrajectoryGenerator();

    /**
     * @brief 构造完成后调用（shared_from_this 此时安全）
     *
     * 在此方法中创建 MoveIt2 相关对象和订阅者，避免在构造函数中
     * 调用 shared_from_this 导致的 UB。
     */
    void init();

    /**
     * @brief 从 JSON 字符串解析 7 维动作
     * @param json_str VLA 输出的 JSON 字符串
     * @return 解析出的动作向量 [x, y, z, rx, ry, rz, gripper]
     */
    static std::vector<double> parse_action(const std::string& json_str);

    /**
     * @brief 将轨迹重采样到固定时间步长 dt
     * @param traj 待重采样的轨迹（原地修改）
     * @param dt   目标时间步长（秒）
     */
    static void resample(trajectory_msgs::msg::JointTrajectory& traj, double dt);

private:
    /**
     * @brief 动作消息回调，解析并规划轨迹后发布
     * @param msg VLA 输出的 JSON 字符串消息
     */
    void on_action(const std_msgs::msg::String::SharedPtr msg);

    /**
     * @brief MoveIt2 RRT* 规划（有 MoveIt 时调用）
     * @param action   7 维动作向量
     * @param out_traj 输出规划结果
     * @return 规划是否成功
     */
    bool plan_with_moveit(const std::vector<double>& action,
                          trajectory_msgs::msg::JointTrajectory& out_traj);

    /**
     * @brief 关节空间线性插值（standalone 降级模式）
     * @param action   7 维动作向量
     * @param out_traj 输出插值结果
     */
    void plan_linear_fallback(const std::vector<double>& action,
                              trajectory_msgs::msg::JointTrajectory& out_traj);

    /// MoveIt2 接口（可选，编译期决定）
    struct MoveItImpl;
    std::shared_ptr<MoveItImpl> moveit_;  ///< MoveIt2 pimpl 指针

    std::vector<double> current_joints_;  ///< 当前关节角（用于线性插值起点）

    /// Linear fallback 参数
    double fallback_scale_;          ///< 动作增量缩放因子
    double fallback_duration_;       ///< 轨迹持续时间（秒）
    double fallback_blend_ratio_;    ///< 加减速段占比（0~0.5）
    double traj_dt_;                 ///< 轨迹时间步长（秒），即重采样频率

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;   ///< 轨迹发布者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;              ///< 动作订阅者
};
