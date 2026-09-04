// trajectory_generator.cpp — VLA动作→关节轨迹生成（实现）
#include "trajectory_generator.hpp"

#include <sstream>
#include <cmath>
#include <algorithm>
#include <memory>

// ===== MoveIt2 可选依赖 =====
// HAS_MOVEIT 由 CMake 控制，避免头文件存在但链接失败的问题
#ifndef HAS_MOVEIT
  #define HAS_MOVEIT 0
#endif

#if HAS_MOVEIT
// CMake 检测头文件扩展名：Humble 用 .h，Iron+ 用 .hpp
#  ifdef MOVEIT_HEADER_HPP
#    include <moveit/move_group_interface/move_group_interface.hpp>
#  else
#    include <moveit/move_group_interface/move_group_interface.h>
#  endif
#  include <tf2/LinearMath/Quaternion.hpp>
#  include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

// ===== 简易 JSON 解析（避免依赖 nlohmann/json） =====
// 解析格式: {"action": [x,y,z,rx,ry,rz,g]}
static std::vector<double> simple_parse_action(const std::string& json_str) {
    std::vector<double> result;
    // 找 "action": [ ... ]
    size_t arr_start = json_str.find('[');
    size_t arr_end = json_str.find(']', arr_start);
    if (arr_start == std::string::npos || arr_end == std::string::npos)
        return result;

    std::string inner = json_str.substr(arr_start + 1, arr_end - arr_start - 1);
    std::stringstream ss(inner);
    std::string token;
    while (std::getline(ss, token, ',')) {
        // 去掉空格
        token.erase(0, token.find_first_not_of(" \t\r\n"));
        token.erase(token.find_last_not_of(" \t\r\n") + 1);
        if (!token.empty()) {
            try {
                result.push_back(std::stod(token));
            } catch (...) {
                // 跳过非数字
            }
        }
    }
    return result;
}

// ===== MoveIt 实现指针（pimpl） =====
#if HAS_MOVEIT
struct TrajectoryGenerator::MoveItImpl {
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
};
#endif

// ===== 构造函数 =====
TrajectoryGenerator::TrajectoryGenerator()
    : Node("trajectory_generator"),
      current_joints_{0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785}  // Franka 就绪位
{
    // ---- 参数声明 ----
    declare_parameter("fallback_scale", 0.3);
    declare_parameter("fallback_duration", 2.0);
    declare_parameter("fallback_blend_ratio", 0.2);
    declare_parameter("traj_dt", 0.01);

    fallback_scale_ = get_parameter("fallback_scale").as_double();
    fallback_duration_ = get_parameter("fallback_duration").as_double();
    fallback_blend_ratio_ = get_parameter("fallback_blend_ratio").as_double();
    traj_dt_ = get_parameter("traj_dt").as_double();

    // 合法性校验
    if (fallback_blend_ratio_ < 0.0 || fallback_blend_ratio_ > 0.5) {
        RCLCPP_WARN(get_logger(),
            "fallback_blend_ratio %.3f out of range [0, 0.5], clamping",
            fallback_blend_ratio_);
        fallback_blend_ratio_ = std::clamp(fallback_blend_ratio_, 0.0, 0.5);
    }
    if (traj_dt_ <= 0.0) {
        RCLCPP_WARN(get_logger(), "traj_dt must be positive, using 0.01");
        traj_dt_ = 0.01;
    }
    if (fallback_duration_ <= 0.0) {
        RCLCPP_WARN(get_logger(), "fallback_duration must be positive, using 2.0");
        fallback_duration_ = 2.0;
    }

    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/planning/trajectory", 1);

    action_sub_ = create_subscription<std_msgs::msg::String>(
        "/vla/action_cmd", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            this->on_action(msg);
        });
}

// ===== init：构造完成后初始化 MoveIt =====
void TrajectoryGenerator::init() {
#if HAS_MOVEIT
    try {
        moveit_ = std::make_shared<MoveItImpl>();
        moveit_->move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "panda_arm");

        moveit_->move_group->setPlanningTime(0.05);
        moveit_->move_group->setPlannerId("RRTstar");
        moveit_->move_group->setNumPlanningAttempts(3);
        moveit_->move_group->setMaxVelocityScalingFactor(0.3);
        moveit_->move_group->setMaxAccelerationScalingFactor(0.2);

        RCLCPP_INFO(get_logger(), "TrajectoryGenerator: MoveIt2 RRT* ready");
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(),
            "MoveIt2 init failed: %s — using standalone linear fallback", e.what());
        moveit_.reset();
    }
#else
    RCLCPP_WARN(get_logger(),
        "MoveIt2 NOT compiled in — using standalone linear interpolation fallback");
#endif
}

// ===== 解析动作 =====
std::vector<double> TrajectoryGenerator::parse_action(const std::string& json_str) {
    return simple_parse_action(json_str);
}

// ===== MoveIt 规划 =====
bool TrajectoryGenerator::plan_with_moveit(
        const std::vector<double>& action,
        trajectory_msgs::msg::JointTrajectory& out_traj) {
#if HAS_MOVEIT
    if (!moveit_ || !moveit_->move_group) return false;

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = action[0];
    target_pose.position.y = action[1];
    target_pose.position.z = action[2];

    tf2::Quaternion q;
    q.setRPY(action[3], action[4], action[5]);
    target_pose.orientation = tf2::toMsg(q);

    moveit_->move_group->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (moveit_->move_group->plan(plan) ==
        moveit::core::MoveItErrorCode::SUCCESS);

    if (!ok) {
        RCLCPP_WARN(get_logger(), "RRT* planning failed");
        return false;
    }

    auto traj_msg = plan.trajectory_.joint_trajectory;

    resample(traj_msg, traj_dt_);
    out_traj = traj_msg;

    // 更新当前关节角为轨迹终点
    if (!out_traj.points.empty()) {
        current_joints_ = out_traj.points.back().positions;
    }
    return true;
#else
    (void)action;
    (void)out_traj;
    return false;
#endif
}

// ===== 线性插值降级 =====
void TrajectoryGenerator::plan_linear_fallback(
        const std::vector<double>& action,
        trajectory_msgs::msg::JointTrajectory& out_traj) {
    // 简化：用逆运动学近似，直接把动作的前6维映射为关节角增量
    // （真实系统用IK，这里standalone模式做简化演示）
    std::vector<double> target_joints = current_joints_;
    for (size_t i = 0; i < std::min(action.size(), target_joints.size()); ++i) {
        target_joints[i] += action[i] * fallback_scale_;
    }

    out_traj.joint_names = {
        "panda_joint1", "panda_joint2", "panda_joint3",
        "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
    };

    double blend = fallback_blend_ratio_;
    double one_minus_blend = 1.0 - blend;
    // 梯形速度曲线：v_max = 1 / (1 - blend)，保证总位移为 1.0
    double denom = blend * one_minus_blend;  // b * (1-b)

    for (double t = 0.0; t <= fallback_duration_ + 1e-6; t += traj_dt_) {
        double alpha = t / fallback_duration_;
        // 平滑加减速（梯形速度曲线，位置与速度均连续）
        if (alpha < blend) {
            // 加速段：二次曲线
            alpha = 0.5 * alpha * alpha / denom;
        } else if (alpha > one_minus_blend) {
            // 减速段：二次曲线
            double u = 1.0 - alpha;
            alpha = 1.0 - 0.5 * u * u / denom;
        } else {
            // 匀速段：线性
            alpha = (alpha - 0.5 * blend) / one_minus_blend;
        }

        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.time_from_start = rclcpp::Duration::from_seconds(t);
        for (size_t j = 0; j < target_joints.size(); ++j) {
            pt.positions.push_back(
                current_joints_[j] + alpha * (target_joints[j] - current_joints_[j]));
        }
        out_traj.points.push_back(pt);
    }

    current_joints_ = target_joints;
}

// ===== 重采样 =====
void TrajectoryGenerator::resample(
        trajectory_msgs::msg::JointTrajectory& traj, double dt) {
    if (traj.points.size() < 2) return;

    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> new_pts;
    for (size_t i = 0; i < traj.points.size() - 1; ++i) {
        auto& p0 = traj.points[i];
        auto& p1 = traj.points[i + 1];
        double t0 = rclcpp::Duration(p0.time_from_start).seconds();
        double t1 = rclcpp::Duration(p1.time_from_start).seconds();
        if (t1 <= t0) continue;

        for (double t = t0; t < t1 - 1e-9; t += dt) {
            double alpha = (t - t0) / (t1 - t0);
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.time_from_start = rclcpp::Duration::from_seconds(t);
            for (size_t j = 0; j < p0.positions.size(); ++j) {
                pt.positions.push_back(
                    p0.positions[j] + alpha * (p1.positions[j] - p0.positions[j]));
            }
            new_pts.push_back(pt);
        }
    }
    new_pts.push_back(traj.points.back());
    traj.points = std::move(new_pts);
}

// ===== 动作回调 =====
void TrajectoryGenerator::on_action(const std_msgs::msg::String::SharedPtr msg) {
    auto action = parse_action(msg->data);
    if (action.size() < 6) {
        RCLCPP_WARN(get_logger(), "Invalid action: need at least 6 DOF");
        return;
    }

    trajectory_msgs::msg::JointTrajectory traj;
    bool ok = plan_with_moveit(action, traj);
    if (!ok) {
        RCLCPP_INFO(get_logger(), "Using linear fallback for trajectory");
        plan_linear_fallback(action, traj);
    }

    RCLCPP_INFO(get_logger(),
        "Trajectory generated: %zu points, %.2fs total",
        traj.points.size(),
        traj.points.empty() ? 0.0 :
        rclcpp::Duration(traj.points.back().time_from_start).seconds());

    traj_pub_->publish(traj);
}

// ===== 入口 =====
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryGenerator>();
    node->init();  // 构造后初始化（shared_from_this 安全）
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}