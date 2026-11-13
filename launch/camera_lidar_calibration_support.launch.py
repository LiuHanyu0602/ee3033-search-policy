"""Camera TF and formal detector adapter for non-navigating calibration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ee3033_search_policy')
    params = LaunchConfiguration('params_file')
    labels = LaunchConfiguration('labels_file')
    calibration = LaunchConfiguration('calibration_file')
    actions = [
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                share, 'config', 'real_robot_workflow.yaml')),
        DeclareLaunchArgument('labels_file'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=os.path.join(
                share, 'calibration', 'calibration.npz')),
        ExecuteProcess(
            cmd=[
                '/usr/bin/python3', '-m',
                'ee3033_search_policy.euler_camera_tf_node',
                '--ros-args', '--params-file', params,
            ],
            output='screen',
            name='euler_camera_tf',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ee3033_camera_mount_tf',
            arguments=[
                '--x', '-0.035', '--y', '0', '--z', '0.2655',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'ee3033_balance_body',
                '--child-frame-id', 'ee3033_camera_mount',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ee3033_camera_optical_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.57079632679', '--pitch', '0',
                '--yaw', '-1.57079632679',
                '--frame-id', 'ee3033_camera_mount',
                '--child-frame-id', 'default_cam',
            ],
        ),
        ExecuteProcess(
            cmd=[
                '/usr/bin/python3', '-m',
                'ee3033_search_policy.continuous_observation_adapter',
                '--ros-args', '--params-file', params,
                '-p', ['labels_file:=', labels],
                '-p', ['calibration_file:=', calibration],
            ],
            output='screen',
            name='continuous_observation_adapter',
        ),
    ]
    return LaunchDescription(actions)
