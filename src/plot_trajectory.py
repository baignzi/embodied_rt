#!/usr/bin/env python3
"""
plot_trajectory.py — 订阅轨迹并绘制关节曲线图

用法:
  终端1: ros2 launch embodied_rt embodied_rt.launch.py
  终端2: python3 plot_trajectory.py
"""
import sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

try:
    import rclpy
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("[ERROR] rclpy not available. Run: source /opt/ros/humble/setup.bash")
    sys.exit(1)


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        self.sub = self.create_subscription(
            JointTrajectory, '/planning/trajectory',
            self.callback, 10)
        self.trajectories = []
        self.get_logger().info('Waiting for trajectory on /planning/trajectory ...')

    def callback(self, msg):
        self.trajectories.append(msg)
        self.get_logger().info(
            f'Received trajectory #{len(self.trajectories)}: '
            f'{len(msg.points)} points, {len(msg.joint_names)} joints')
        self.plot(msg)

    def plot(self, traj):
        n_joints = len(traj.joint_names)
        times = []
        positions = [[] for _ in range(n_joints)]

        for pt in traj.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            times.append(t)
            for j in range(min(n_joints, len(pt.positions))):
                positions[j].append(pt.positions[j])

        fig, axes = plt.subplots(n_joints, 1, figsize=(10, 2 * n_joints),
                                 sharex=True)
        if n_joints == 1:
            axes = [axes]

        colors = ['#e74c3c', '#2ecc71', '#3498db', '#f39c12',
                  '#9b59b6', '#1abc9c', '#e67e22']

        for j in range(n_joints):
            axes[j].plot(times, positions[j], color=colors[j % len(colors)],
                        linewidth=2, marker='o', markersize=2)
            axes[j].set_ylabel(traj.joint_names[j], fontsize=9)
            axes[j].grid(True, alpha=0.3)
            axes[j].tick_params(labelsize=8)

        axes[-1].set_xlabel('Time (s)', fontsize=10)
        fig.suptitle(f'Trajectory #{len(self.trajectories)} — '
                     f'{len(traj.points)} points, '
                     f'{times[-1]:.2f}s duration',
                     fontsize=12, fontweight='bold')
        plt.tight_layout()
        plt.savefig(f'/tmp/trajectory_{len(self.trajectories)}.png', dpi=150)
        self.get_logger().info(f'Plot saved to /tmp/trajectory_{len(self.trajectories)}.png')
        plt.show()


def main():
    rclpy.init()
    node = TrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
