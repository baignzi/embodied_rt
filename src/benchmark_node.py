#!/usr/bin/env python3
"""
benchmark_node.py — EmbodiedRT 性能基准测试节点

订阅所有 topic，测量以下指标:
  1. 各 topic 消息频率 (Hz)
  2. 控制环抖动 (jitter, ms)
  3. 端到端延迟 (VLA发布→控制输出, ms)
  4. 轨迹生成延迟 (收到VLA→发布轨迹, ms)
  5. PID 跟踪误差 (目标位置 vs 实际位置)
  6. 吞吐量 (每秒处理动作数)

用法:
  终端1: ros2 launch embodied_rt embodied_rt.launch.py
  终端2: source install/setup.bash && python3 benchmark_node.py --duration 30
  结果保存到 /tmp/embodied_rt_benchmark.json
"""

import argparse
import time
import json
import statistics
import threading
import sys
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class BenchmarkNode(Node):
    def __init__(self, duration=30):
        super().__init__('benchmark_node')
        self.duration = duration
        self.start_time = time.monotonic()

        # ===== 数据存储 =====
        # 频率统计
        self.topic_timestamps = {
            '/vla/action_cmd': deque(maxlen=10000),
            '/planning/trajectory': deque(maxlen=10000),
            '/joint_states': deque(maxlen=50000),
        }
        # 延迟统计
        self.vla_receive_times = {}  # msg_id -> timestamp
        self.traj_receive_times = {}
        self.control_receive_times = {}

        # 端到端延迟: VLA发布 -> 控制输出
        self.e2e_latencies = deque(maxlen=5000)
        # 轨迹生成延迟: VLA接收 -> 轨迹发布
        self.traj_gen_latencies = deque(maxlen=5000)

        # 控制环抖动
        self.control_intervals = deque(maxlen=50000)

        # PID 跟踪误差
        self.tracking_errors = deque(maxlen=50000)
        self.last_trajectory = None

        # 动作计数
        self.action_count = 0

        # ===== 订阅 =====
        self.create_subscription(String, '/vla/action_cmd',
                                  self.on_vla_action, 10)
        self.create_subscription(JointTrajectory, '/planning/trajectory',
                                  self.on_trajectory, 10)
        self.create_subscription(JointState, '/joint_states',
                                  self.on_joint_state, 10)

        # ===== 定时器: 每5秒打印一次中间结果 =====
        self.create_timer(5.0, self.print_intermediate)
        # ===== 定时器: duration 到了就退出 =====
        self.create_timer(duration, self.finish)

        self.get_logger().info(f'=' * 60)
        self.get_logger().info(f'  EmbodiedRT Benchmark — {duration}s duration')
        self.get_logger().info(f'  Listening on:')
        self.get_logger().info(f'    /vla/action_cmd    (VLA actions)')
        self.get_logger().info(f'    /planning/trajectory (trajectories)')
        self.get_logger().info(f'    /joint_states      (control output)')
        self.get_logger().info(f'=' * 60)

    # ===== VLA 动作回调 =====
    def on_vla_action(self, msg):
        t = time.monotonic()
        msg_id = self.action_count
        self.action_count += 1
        self.vla_receive_times[msg_id] = t
        self.topic_timestamps['/vla/action_cmd'].append(t)
        self.get_logger().debug(f'VLA action #{msg_id} received')

    # ===== 轨迹回调 =====
    def on_trajectory(self, msg):
        t = time.monotonic()
        self.topic_timestamps['/planning/trajectory'].append(t)

        # 找最近的 VLA 动作, 计算轨迹生成延迟
        if self.vla_receive_times:
            latest_id = max(self.vla_receive_times.keys())
            if latest_id in self.vla_receive_times:
                vla_time = self.vla_receive_times[latest_id]
                latency_ms = (t - vla_time) * 1000
                self.traj_gen_latencies.append(latency_ms)

            # 保存轨迹用于跟踪误差计算
            if msg.points:
                self.last_trajectory = msg

    # ===== 控制器输出回调 =====
    def on_joint_state(self, msg):
        t = time.monotonic()
        self.topic_timestamps['/joint_states'].append(t)

        # 控制环间隔 (jitter)
        ts = self.topic_timestamps['/joint_states']
        if len(ts) >= 2:
            interval_ms = (t - ts[-2]) * 1000
            self.control_intervals.append(interval_ms)

        # 端到端延迟: VLA -> 控制
        if self.vla_receive_times:
            latest_id = max(self.vla_receive_times.keys())
            if latest_id in self.vla_receive_times:
                vla_time = self.vla_receive_times[latest_id]
                e2e_ms = (t - vla_time) * 1000
                if e2e_ms < 10000:  # 过滤掉异常值
                    self.e2e_latencies.append(e2e_ms)

        # 跟踪误差: 目标位置 vs 实际位置
        if self.last_trajectory and self.last_trajectory.points:
            # 找当前时间对应的轨迹点
            traj = self.last_trajectory
            if traj.points and msg.position:
                target = traj.points[0].positions  # 轨迹起点
                actual = msg.position
                n = min(len(target), len(actual))
                if n > 0:
                    error = sum(abs(target[i] - actual[i]) for i in range(n)) / n
                    self.tracking_errors.append(error)

    # ===== 中间结果打印 =====
    def print_intermediate(self):
        elapsed = time.monotonic() - self.start_time
        self.get_logger().info(f'--- {elapsed:.0f}s / {self.duration}s ---')

        # 控制频率
        ctrl_ts = self.topic_timestamps['/joint_states']
        if len(ctrl_ts) >= 2:
            ctrl_hz = (len(ctrl_ts) - 1) / (ctrl_ts[-1] - ctrl_ts[0])
            self.get_logger().info(f'  Control freq: {ctrl_hz:.1f} Hz')

        # VLA 频率
        vla_ts = self.topic_timestamps['/vla/action_cmd']
        if len(vla_ts) >= 2:
            vla_hz = (len(vla_ts) - 1) / (vla_ts[-1] - vla_ts[0])
            self.get_logger().info(f'  VLA freq:     {vla_hz:.2f} Hz')

        # 抖动
        if len(self.control_intervals) >= 10:
            jitter = statistics.stdev(self.control_intervals)
            mean_interval = statistics.mean(self.control_intervals)
            self.get_logger().info(f'  Jitter:       {jitter:.3f} ms (mean={mean_interval:.3f} ms)')

        # 延迟
        if self.traj_gen_latencies:
            avg_traj = statistics.mean(self.traj_gen_latencies)
            self.get_logger().info(f'  Traj gen:     {avg_traj:.2f} ms')

        if self.e2e_latencies:
            avg_e2e = statistics.mean(self.e2e_latencies)
            self.get_logger().info(f'  E2E latency:  {avg_e2e:.2f} ms')

        # 跟踪误差
        if self.tracking_errors:
            avg_err = statistics.mean(self.tracking_errors)
            max_err = max(self.tracking_errors)
            self.get_logger().info(f'  Track error:  {avg_err:.4f} rad (max={max_err:.4f})')

    # ===== 最终报告 =====
    def finish(self):
        elapsed = time.monotonic() - self.start_time
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  FINAL BENCHMARK REPORT')
        self.get_logger().info('=' * 60)

        report = {
            'test_date': datetime.now().isoformat(),
            'duration_s': round(elapsed, 2),
            'metrics': {}
        }

        # 1. 控制频率
        ctrl_ts = list(self.topic_timestamps['/joint_states'])
        if len(ctrl_ts) >= 2:
            ctrl_hz = (len(ctrl_ts) - 1) / (ctrl_ts[-1] - ctrl_ts[0])
            report['metrics']['control_frequency_hz'] = round(ctrl_hz, 2)
            self.get_logger().info(f'\n  Control Loop Frequency:')
            self.get_logger().info(f'    Average:  {ctrl_hz:.2f} Hz')
            if len(self.control_intervals) >= 2:
                intervals = list(self.control_intervals)
                mean_int = statistics.mean(intervals)
                std_int = statistics.stdev(intervals) if len(intervals) > 1 else 0
                min_int = min(intervals)
                max_int = max(intervals)
                p95 = sorted(intervals)[int(len(intervals) * 0.95)] if intervals else 0
                report['metrics']['control_period_ms'] = {
                    'mean': round(mean_int, 4),
                    'std': round(std_int, 4),
                    'min': round(min_int, 4),
                    'max': round(max_int, 4),
                    'p95': round(p95, 4),
                }
                self.get_logger().info(f'    Period:   {mean_int:.4f} ms (target: 1.0 ms)')
                self.get_logger().info(f'    Jitter:   +/-{std_int:.4f} ms')
                self.get_logger().info(f'    Min/Max:  {min_int:.4f} / {max_int:.4f} ms')
                self.get_logger().info(f'    P95:      {p95:.4f} ms')

        # 2. VLA 频率
        vla_ts = list(self.topic_timestamps['/vla/action_cmd'])
        if len(vla_ts) >= 2:
            vla_hz = (len(vla_ts) - 1) / (vla_ts[-1] - vla_ts[0])
            report['metrics']['vla_frequency_hz'] = round(vla_hz, 2)
            self.get_logger().info(f'\n  VLA Action Frequency:')
            self.get_logger().info(f'    Average:  {vla_hz:.2f} Hz')

        # 3. 轨迹生成延迟
        if self.traj_gen_latencies:
            lat = list(self.traj_gen_latencies)
            mean_lat = statistics.mean(lat)
            std_lat = statistics.stdev(lat) if len(lat) > 1 else 0
            max_lat = max(lat)
            report['metrics']['trajectory_generation_latency_ms'] = {
                'mean': round(mean_lat, 3),
                'std': round(std_lat, 3),
                'max': round(max_lat, 3),
                'samples': len(lat),
            }
            self.get_logger().info(f'\n  Trajectory Generation Latency:')
            self.get_logger().info(f'    Mean:     {mean_lat:.3f} ms')
            self.get_logger().info(f'    Std:      +/-{std_lat:.3f} ms')
            self.get_logger().info(f'    Max:      {max_lat:.3f} ms')
            self.get_logger().info(f'    Samples:  {len(lat)}')

        # 4. 端到端延迟
        if self.e2e_latencies:
            e2e = list(self.e2e_latencies)
            mean_e2e = statistics.mean(e2e)
            std_e2e = statistics.stdev(e2e) if len(e2e) > 1 else 0
            p95_e2e = sorted(e2e)[int(len(e2e) * 0.95)] if e2e else 0
            report['metrics']['end_to_end_latency_ms'] = {
                'mean': round(mean_e2e, 3),
                'std': round(std_e2e, 3),
                'p95': round(p95_e2e, 3),
                'max': round(max(e2e), 3),
                'samples': len(e2e),
            }
            self.get_logger().info(f'\n  End-to-End Latency (VLA -> Control):')
            self.get_logger().info(f'    Mean:     {mean_e2e:.3f} ms')
            self.get_logger().info(f'    Std:      +/-{std_e2e:.3f} ms')
            self.get_logger().info(f'    P95:      {p95_e2e:.3f} ms')
            self.get_logger().info(f'    Max:      {max(e2e):.3f} ms')

        # 5. 跟踪误差
        if self.tracking_errors:
            errs = list(self.tracking_errors)
            mean_err = statistics.mean(errs)
            max_err = max(errs)
            report['metrics']['tracking_error_rad'] = {
                'mean': round(mean_err, 6),
                'max': round(max_err, 6),
                'samples': len(errs),
            }
            self.get_logger().info(f'\n  PID Tracking Error:')
            self.get_logger().info(f'    Mean:     {mean_err:.6f} rad ({mean_err * 57.2958:.4f} deg)')
            self.get_logger().info(f'    Max:      {max_err:.6f} rad ({max_err * 57.2958:.4f} deg)')

        # 6. 吞吐量
        report['metrics']['throughput'] = {
            'total_actions': self.action_count,
            'total_trajectories': len(self.topic_timestamps['/planning/trajectory']),
            'total_control_msgs': len(self.topic_timestamps['/joint_states']),
            'actions_per_second': round(self.action_count / elapsed, 2) if elapsed > 0 else 0,
        }
        self.get_logger().info(f'\n  Throughput:')
        self.get_logger().info(f'    Total VLA actions:     {self.action_count}')
        self.get_logger().info(f'    Total trajectories:     {len(self.topic_timestamps["/planning/trajectory"])}')
        self.get_logger().info(f'    Total control msgs:    {len(self.topic_timestamps["/joint_states"])}')
        self.get_logger().info(f'    Actions/sec:           {self.action_count / elapsed:.2f}')

        # 保存 JSON
        out_path = '/tmp/embodied_rt_benchmark.json'
        with open(out_path, 'w') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        self.get_logger().info(f'\n  Report saved to: {out_path}')
        self.get_logger().info('=' * 60)

        raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser(description='EmbodiedRT Benchmark')
    parser.add_argument('--duration', type=int, default=30,
                       help='Test duration in seconds (default: 30)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = BenchmarkNode(duration=args.duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.finish()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
