import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('embodied_rt'),
        'config', 'embodied_rt.yaml')

    # Panda URDF
    panda_urdf_path = '/opt/ros/humble/share/moveit_resources_panda_description/urdf/panda.urdf'
    urdf_content = ''
    if os.path.exists(panda_urdf_path):
        with open(panda_urdf_path, 'r') as f:
            urdf_content = f.read()

    return LaunchDescription([
        # --- 1. VLA推理节点 ---
        Node(
            package='embodied_rt',
            executable='vla_inference_node',
            name='vla_inference_node',
            parameters=[config_file, {'mock': True}],
            output='screen',
        ),

        # --- 2. 轨迹生成器 ---
        Node(
            package='embodied_rt',
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen',
        ),

        # --- 3. 实时控制器（同时发布到 /joint_states 供 RViz 显示） ---
        Node(
            package='embodied_rt',
            executable='real_time_controller',
            name='real_time_controller',
            parameters=[config_file],
            remappings=[('/control/joint_cmd', '/joint_states')],
            output='screen',
        ),

        # --- 4. 安全监控 ---
        Node(
            package='embodied_rt',
            executable='safety_monitor',
            name='safety_monitor',
            parameters=[config_file],
            remappings=[('/control/joint_cmd', '/joint_states')],
            output='screen',
        ),

        # --- 5. robot_state_publisher ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_content}],
            output='screen',
        ),

        # --- 6. RViz2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('embodied_rt'),
                'config', 'embodied_rt.rviz')],
            output='screen',
        ),
    ])
