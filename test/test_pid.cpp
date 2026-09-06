// test_pid.cpp — PID 控制器单元测试
#include <gtest/gtest.h>
#include "pid_controller.hpp"
#include <cmath>

// === PID 比例项测试 ===
TEST(PIDTest, ProportionalResponse) {
    PIDParams params{10.0, 0.0, 0.0, 0.0, 100.0};
    PIDState state;
    double output = compute_pid_output(params, state, 1.0, 0.0, 0.001);
    EXPECT_NEAR(output, 10.0, 1e-6);
}

TEST(PIDTest, ProportionalZeroError) {
    PIDParams params{50.0, 0.0, 0.0, 0.0, 100.0};
    PIDState state;
    double output = compute_pid_output(params, state, 5.0, 5.0, 0.001);
    EXPECT_NEAR(output, 0.0, 1e-6);
}

// === PID 积分项测试 ===
TEST(PIDTest, IntegralAccumulation) {
    PIDParams params{0.0, 5.0, 0.0, 0.0, 100.0};
    PIDState state;
    // 第一步: error=1.0, integral = 1.0*0.001 = 0.001
    double output1 = compute_pid_output(params, state, 1.0, 0.0, 0.001);
    EXPECT_NEAR(output1, 5.0 * 0.001, 1e-6);
    // 第二步: 相同误差, integral = 0.002
    double output2 = compute_pid_output(params, state, 1.0, 0.0, 0.001);
    EXPECT_NEAR(output2, 5.0 * 0.002, 1e-6);
}

TEST(PIDTest, IntegralNegativeError) {
    PIDParams params{0.0, 5.0, 0.0, 0.0, 100.0};
    PIDState state;
    double output = compute_pid_output(params, state, -1.0, 0.0, 0.001);
    EXPECT_NEAR(output, -5.0 * 0.001, 1e-6);
    EXPECT_NEAR(state.integral, -0.001, 1e-9);
}

// === PID 微分项测试 ===
TEST(PIDTest, DerivativeResponse) {
    PIDParams params{0.0, 0.0, 2.0, 0.0, 100.0};
    PIDState state;
    // 第一步: error=0.5, deriv = (0.5-0)/0.001 = 500
    double output1 = compute_pid_output(params, state, 0.5, 0.0, 0.001);
    EXPECT_NEAR(output1, 2.0 * 500.0, 1e-3);
    // 第二步: 相同误差, deriv = 0
    double output2 = compute_pid_output(params, state, 0.5, 0.0, 0.001);
    EXPECT_NEAR(output2, 0.0, 1e-6);
}

TEST(PIDTest, DerivativeSignChange) {
    PIDParams params{0.0, 0.0, 1.0, 0.0, 100.0};
    PIDState state;
    // 先建立正向误差
    compute_pid_output(params, state, 1.0, 0.0, 0.001);
    // 误差减小: deriv 应为负
    double output = compute_pid_output(params, state, 0.5, 0.0, 0.001);
    // deriv = (0.5 - 1.0) / 0.001 = -500
    EXPECT_NEAR(output, -500.0, 1e-3);
}

// === 输出限幅测试 ===
TEST(PIDTest, OutputClampingPositive) {
    PIDParams params{1000.0, 0.0, 0.0, 0.0, 5.0};
    PIDState state;
    double output = compute_pid_output(params, state, 1.0, 0.0, 0.001);
    EXPECT_NEAR(output, 5.0, 1e-6);
}

TEST(PIDTest, OutputClampingNegative) {
    PIDParams params{1000.0, 0.0, 0.0, 0.0, 5.0};
    PIDState state;
    double output = compute_pid_output(params, state, -1.0, 0.0, 0.001);
    EXPECT_NEAR(output, -5.0, 1e-6);
}

// === 速度前馈测试 ===
TEST(PIDTest, VelocityFeedforward) {
    PIDParams params{0.0, 0.0, 0.0, 3.0, 100.0};
    PIDState state;
    double output = compute_pid_output(params, state, 0.0, 0.0, 0.001, true, 2.0);
    EXPECT_NEAR(output, 6.0, 1e-6);
}

TEST(PIDTest, FeedforwardDisabledWithoutVelocity) {
    PIDParams params{0.0, 0.0, 0.0, 3.0, 100.0};
    PIDState state;
    double output = compute_pid_output(params, state, 0.0, 0.0, 0.001, false, 2.0);
    EXPECT_NEAR(output, 0.0, 1e-6);
}

// === 积分抗饱和测试 ===
TEST(PIDTest, AntiWindupClamping) {
    PIDParams params{0.0, 10.0, 0.0, 0.0, 1.0};
    PIDState state;
    // integral 限幅 = max_output / ki = 1.0 / 10.0 = 0.1
    for (int i = 0; i < 10000; ++i) {
        compute_pid_output(params, state, 100.0, 0.0, 0.001);
    }
    EXPECT_NEAR(state.integral, 0.1, 1e-6);
}

// === 综合测试: P+I+D+FF ===
TEST(PIDTest, CombinedPIDFF) {
    PIDParams params{10.0, 5.0, 2.0, 0.5, 50.0};
    PIDState state;
    // error = 2.0 - 0.0 = 2.0
    // deriv = (2.0 - 0.0) / 0.001 = 2000
    // integral = 2.0 * 0.001 = 0.002
    // ff = 0.5 * 3.0 = 1.5
    // output = 10*2 + 5*0.002 + 2*2000 + 1.5 = 20 + 0.01 + 4000 + 1.5 = 4021.51
    // 但被限幅到 50.0
    double output = compute_pid_output(params, state, 2.0, 0.0, 0.001, true, 3.0);
    EXPECT_NEAR(output, 50.0, 1e-6);
}

// === 状态更新验证 ===
TEST(PIDTest, StateUpdates) {
    PIDParams params{1.0, 1.0, 1.0, 0.0, 100.0};
    PIDState state;
    compute_pid_output(params, state, 5.0, 0.0, 0.01);
    EXPECT_NEAR(state.prev_error, 5.0, 1e-6);
    EXPECT_NEAR(state.integral, 0.05, 1e-6);
    // 滤波后的微分值也应被更新
    // raw_deriv = (5.0 - 0.0) / 0.01 = 500; alpha=1.0 so filtered=500
    EXPECT_NEAR(state.prev_filtered_deriv, 500.0, 1e-3);
}

// === PID 微分低通滤波测试 ===
TEST(PIDTest, DerivativeFilterAttenuation) {
    // alpha=0.5: 半衰滤波
    PIDParams params{0.0, 0.0, 1.0, 0.0, 100.0, 0.5};
    PIDState state;
    // 第一步: error=1.0, raw_deriv=(1.0-0)/0.001=1000
    // filtered = 0.5*1000 + 0.5*0 = 500
    double output1 = compute_pid_output(params, state, 1.0, 0.0, 0.001);
    EXPECT_NEAR(output1, 500.0, 1e-3);
    // 第二步: error=1.0, raw_deriv=0
    // filtered = 0.5*0 + 0.5*500 = 250
    double output2 = compute_pid_output(params, state, 1.0, 0.0, 0.001);
    EXPECT_NEAR(output2, 250.0, 1e-3);
}

TEST(PIDTest, DerivativeFilterNoFilterWhenAlphaOne) {
    // alpha=1.0: 不滤波，行为与无滤波一致
    PIDParams params{0.0, 0.0, 2.0, 0.0, 100.0, 1.0};
    PIDState state;
    // 第一步: error=0.5, deriv = (0.5-0)/0.001 = 500
    double output1 = compute_pid_output(params, state, 0.5, 0.0, 0.001);
    EXPECT_NEAR(output1, 2.0 * 500.0, 1e-3);
    // 第二步: 相同误差, deriv = 0
    double output2 = compute_pid_output(params, state, 0.5, 0.0, 0.001);
    EXPECT_NEAR(output2, 0.0, 1e-6);
}
