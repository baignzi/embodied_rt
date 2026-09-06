/**
 * @file pid_controller.hpp
 * @brief PID 控制算法（可测试的纯函数）
 *
 * 提供参数与状态分离的 PID 控制器，支持积分抗饱和和速度前馈。
 * 全部以内联函数实现，便于单元测试和跨编译单元复用。
 */
#pragma once
#include <algorithm>
#include <cmath>

/**
 * @struct PIDParams
 * @brief PID 控制器参数（只读，在控制循环中不修改）
 */
struct PIDParams {
    double kp{0.0};        ///< 比例增益
    double ki{0.0};        ///< 积分增益
    double kd{0.0};        ///< 微分增益
    double kff{0.0};       ///< 速度前馈增益
    double max_output{1.0}; ///< 输出限幅
};

/**
 * @struct PIDState
 * @brief PID 控制器状态（在控制循环中累积更新）
 */
struct PIDState {
    double integral{0.0};  ///< 积分累积项
    double prev_error{0.0}; ///< 上一时刻的误差值
};

/**
 * @brief 单步 PID 计算（含积分抗饱和、速度前馈）
 *
 * 计算流程：
 * 1. 误差 = 设定值 - 当前值
 * 2. 微分项 = (误差 - 上次误差) / dt
 * 3. 积分项累积并做抗饱和限幅
 * 4. 可选速度前馈：kff * velocity
 * 5. 输出限幅到 [-max_output, max_output]
 *
 * @param params        PID 参数（kp/ki/kd/kff/max_output）
 * @param state         PID 状态（会被原地更新）
 * @param setpoint      目标值
 * @param current       当前测量值
 * @param dt            时间步长（秒）
 * @param has_velocity  是否提供速度前馈源（默认 false）
 * @param velocity      前馈速度源（默认 0.0）
 * @return              控制输出（已限幅）
 */
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
