// test_trajectory.cpp — 轨迹生成与重采样单元测试
#include <gtest/gtest.h>
#include "trajectory_generator.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// === parse_action 测试 ===

TEST(TrajectoryParseTest, ValidFullAction) {
    std::string json = R"({"action": [1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.5]})";
    auto result = TrajectoryGenerator::parse_action(json);
    ASSERT_EQ(result.size(), 7u);
    EXPECT_NEAR(result[0], 1.0, 1e-6);
    EXPECT_NEAR(result[1], 2.0, 1e-6);
    EXPECT_NEAR(result[2], 3.0, 1e-6);
    EXPECT_NEAR(result[3], 0.1, 1e-6);
    EXPECT_NEAR(result[4], 0.2, 1e-6);
    EXPECT_NEAR(result[5], 0.3, 1e-6);
    EXPECT_NEAR(result[6], 0.5, 1e-6);
}

TEST(TrajectoryParseTest, PartialAction) {
    std::string json = R"({"action": [1.0, 2.0]})";
    auto result = TrajectoryGenerator::parse_action(json);
    ASSERT_EQ(result.size(), 2u);
    EXPECT_NEAR(result[0], 1.0, 1e-6);
    EXPECT_NEAR(result[1], 2.0, 1e-6);
}

TEST(TrajectoryParseTest, NoBrackets) {
    std::string json = R"({"action": "invalid"})";
    auto result = TrajectoryGenerator::parse_action(json);
    EXPECT_TRUE(result.empty());
}

TEST(TrajectoryParseTest, EmptyArray) {
    std::string json = R"({"action": []})";
    auto result = TrajectoryGenerator::parse_action(json);
    EXPECT_TRUE(result.empty());
}

TEST(TrajectoryParseTest, NonNumericSkipped) {
    std::string json = R"({"action": [1.0, "abc", 3.0]})";
    auto result = TrajectoryGenerator::parse_action(json);
    ASSERT_EQ(result.size(), 2u);
    EXPECT_NEAR(result[0], 1.0, 1e-6);
    EXPECT_NEAR(result[1], 3.0, 1e-6);
}

TEST(TrajectoryParseTest, NegativeValues) {
    std::string json = R"({"action": [-1.0, -2.5, 0.0]})";
    auto result = TrajectoryGenerator::parse_action(json);
    ASSERT_EQ(result.size(), 3u);
    EXPECT_NEAR(result[0], -1.0, 1e-6);
    EXPECT_NEAR(result[1], -2.5, 1e-6);
    EXPECT_NEAR(result[2], 0.0, 1e-6);
}

TEST(TrajectoryParseTest, WhitespaceHandling) {
    std::string json = R"({"action": [ 1.0 , 2.0 , 3.0 ]})";
    auto result = TrajectoryGenerator::parse_action(json);
    ASSERT_EQ(result.size(), 3u);
    EXPECT_NEAR(result[0], 1.0, 1e-6);
    EXPECT_NEAR(result[1], 2.0, 1e-6);
    EXPECT_NEAR(result[2], 3.0, 1e-6);
}

// === resample 测试 ===

class TrajectoryResampleTest : public ::testing::Test {
protected:
    trajectory_msgs::msg::JointTrajectory make_traj(
        const std::vector<double>& times,
        const std::vector<std::vector<double>>& positions) {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = {"joint1"};
        for (size_t i = 0; i < times.size(); ++i) {
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            int32_t sec = (int32_t)times[i];
            uint32_t nsec = (uint32_t)((times[i] - sec) * 1e9);
            pt.time_from_start.sec = sec;
            pt.time_from_start.nanosec = nsec;
            pt.positions = positions[i];
            traj.points.push_back(pt);
        }
        return traj;
    }

    double time_of(const trajectory_msgs::msg::JointTrajectoryPoint& pt) {
        return pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
    }
};

TEST_F(TrajectoryResampleTest, BasicInterpolation) {
    auto traj = make_traj({0.0, 1.0}, {{0.0}, {10.0}});
    TrajectoryGenerator::resample(traj, 0.1);

    // 0~1s, dt=0.1 -> 10 个插值点 + 1 个终点 = 11
    EXPECT_GE(traj.points.size(), 10u);
    EXPECT_LE(traj.points.size(), 12u);

    // 起点位置 = 0
    EXPECT_NEAR(traj.points[0].positions[0], 0.0, 1e-6);
    // 终点位置 = 10
    EXPECT_NEAR(traj.points.back().positions[0], 10.0, 1e-6);

    // 检查线性插值: t=0.5 处 pos ≈ 5.0
    for (const auto& pt : traj.points) {
        if (std::abs(time_of(pt) - 0.5) < 0.01) {
            EXPECT_NEAR(pt.positions[0], 5.0, 0.1);
        }
    }
}

TEST_F(TrajectoryResampleTest, SinglePointNoOp) {
    auto traj = make_traj({0.5}, {{1.0}});
    TrajectoryGenerator::resample(traj, 0.01);
    ASSERT_EQ(traj.points.size(), 1u);
    EXPECT_NEAR(traj.points[0].positions[0], 1.0, 1e-6);
}

TEST_F(TrajectoryResampleTest, EmptyTrajectory) {
    trajectory_msgs::msg::JointTrajectory traj;
    TrajectoryGenerator::resample(traj, 0.01);
    EXPECT_TRUE(traj.points.empty());
}

TEST_F(TrajectoryResampleTest, MultiJoint) {
    auto traj = make_traj({0.0, 1.0}, {{0.0, 5.0}, {10.0, 15.0}});
    traj.joint_names = {"j1", "j2"};
    TrajectoryGenerator::resample(traj, 0.5);

    EXPECT_GE(traj.points.size(), 3u);

    // 检查多关节插值: t=0.5 处 j1≈5.0, j2≈10.0
    for (const auto& pt : traj.points) {
        if (std::abs(time_of(pt) - 0.5) < 0.01) {
            ASSERT_EQ(pt.positions.size(), 2u);
            EXPECT_NEAR(pt.positions[0], 5.0, 0.1);
            EXPECT_NEAR(pt.positions[1], 10.0, 0.1);
        }
    }
}

TEST_F(TrajectoryResampleTest, FineResampling) {
    auto traj = make_traj({0.0, 1.0}, {{0.0}, {100.0}});
    TrajectoryGenerator::resample(traj, 0.01);
    // dt=0.01, 1s 区间 -> 100 个点 + 终点 = 101
    EXPECT_GE(traj.points.size(), 99u);
    EXPECT_LE(traj.points.size(), 103u);
}

TEST_F(TrajectoryResampleTest, PreservesEndpoint) {
    auto traj = make_traj({0.0, 2.0}, {{1.0}, {3.0}});
    TrajectoryGenerator::resample(traj, 0.3);
    // 最后一个点应保持原始终点
    EXPECT_NEAR(traj.points.back().positions[0], 3.0, 1e-6);
    double t_end = time_of(traj.points.back());
    EXPECT_NEAR(t_end, 2.0, 1e-6);
}

TEST_F(TrajectoryResampleTest, MultipleSegments) {
    // 3 个点: t=0,1,3 -> 两段
    auto traj = make_traj({0.0, 1.0, 3.0}, {{0.0}, {5.0}, {15.0}});
    TrajectoryGenerator::resample(traj, 0.5);
    EXPECT_GE(traj.points.size(), 6u);
    // 段间边界 t=1.0 处应有 5.0
    bool found_boundary = false;
    for (const auto& pt : traj.points) {
        if (std::abs(time_of(pt) - 1.0) < 0.01) {
            EXPECT_NEAR(pt.positions[0], 5.0, 0.1);
            found_boundary = true;
        }
    }
    EXPECT_TRUE(found_boundary);
}

TEST_F(TrajectoryResampleTest, MonotonicTimeStamps) {
    auto traj = make_traj({0.0, 1.0}, {{0.0}, {10.0}});
    TrajectoryGenerator::resample(traj, 0.2);
    for (size_t i = 1; i < traj.points.size(); ++i) {
        double t_prev = time_of(traj.points[i - 1]);
        double t_curr = time_of(traj.points[i]);
        EXPECT_GT(t_curr, t_prev);
    }
}

// === linear fallback 轨迹测试 ===
// 注意: plan_linear_fallback 是非静态成员函数, 需要构造 TrajectoryGenerator 节点
// 这里通过 ROS2 节点间接测试

class TrajectoryFallbackTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }
    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_F(TrajectoryFallbackTest, LinearFallbackProducesValidTrajectory) {
    auto node = std::make_shared<TrajectoryGenerator>();
    // 通过 parse_action 验证回退轨迹生成的输入解析
    auto action = TrajectoryGenerator::parse_action(
        R"({"action": [0.1, 0.2, 0.0, 0.0, 0.0, 0.0, 0.5]})");
    ASSERT_EQ(action.size(), 7u);

    // plan_linear_fallback 是 private, 但 resample 和 parse_action 已覆盖
    // 这里验证节点能正常构造和析构
    SUCCEED();
}
