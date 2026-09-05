// pid_controller.hpp — PID 控制算法（可测试的纯函数）
#pragma once
#include <algorithm>
#include <cmath>

struct PIDParams {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double kff{0.0};       ///< 速度前馈增益
    double max_output{1.0}; ///< 输出限幅
};

struct PIDState {
    double integral{0.0};
    double prev_error{0.0};
};

/// 单步 PID 计算（含积分抗饱和、速度前馈）
inline double compute_pid_output(const PIDParams& params, PIDState& state,
                                 double setpoint, double current, double dt,
                                 bool has_velocity = false,
                                 double velocity = 0.0) {
    const double error = setpoint - current;
    const double deriv = (error - state.prev_error) / dt;
    state.integral += error * dt;

    if (params.ki > 0) {
        state.integral = std::clamp(state.integral,
            -params.max_output / params.ki,
            params.max_output / params.ki);
    }

    const double ff = has_velocity ? params.kff * velocity : 0.0;
    const double output = std::clamp(
        params.kp * error
        + params.ki * state.integral
        + params.kd * deriv
        + ff,
        -params.max_output, params.max_output);
    state.prev_error = error;
    return output;
}
