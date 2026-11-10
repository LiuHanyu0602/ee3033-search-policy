"""Opt-in complete EE3033 target-search runtime.

The robot base, known-map Nav2, camera and deployed YOLO detector must already
be running. This launch adds the dynamic balancing-camera TF and all search
policy nodes.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ee3033_search_policy')
    params = LaunchConfiguration('params_file')
    nav = LaunchConfiguration('navigation_enabled')
    single_goal = LaunchConfiguration('single_goal_mode')
    max_goals = LaunchConfiguration('max_navigation_goals')
    labels = LaunchConfiguration('labels_file')
    calibration = LaunchConfiguration('calibration_file')
    processes = []

    # base_footprint already carries planar map pose/yaw. Add only live body
    # roll/pitch, then the measured camera translation and REP-103 optical axes.
    processes.append(ExecuteProcess(
        cmd=[
            '/usr/bin/python3', '-m',
            'ee3033_search_policy.euler_camera_tf_node',
            '--ros-args', '--params-file', params,
        ],
        output='screen',
        name='euler_camera_tf',
    ))
    processes.extend([
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
    ])

    modules = [
        'continuous_observation_adapter',
        'stamped_target_localizer_node',
        'target_mission_coordinator',
        'guarded_search_manager',
    ]
    for module in modules:
        cmd = [
            '/usr/bin/python3', '-m', 'ee3033_search_policy.' + module,
            '--ros-args', '--params-file', params,
        ]
        if module == 'guarded_search_manager':
            cmd += [
                '-p', ['navigation_enabled:=', nav],
                '-p', 'return_home_enabled:=false',
                '-p', ['single_goal_mode:=', single_goal],
                '-p', ['max_navigation_goals:=', max_goals],
            ]
        if module in ('continuous_observation_adapter', 'stamped_target_localizer_node'):
            cmd += ['-p', ['calibration_file:=', calibration]]
        if module == 'continuous_observation_adapter':
            cmd += ['-p', ['labels_file:=', labels]]
        processes.append(ExecuteProcess(cmd=cmd, output='screen', name=module))

    actions = [DeclareLaunchArgument('params_file', default_value=os.path.join(
        share,'config','yolo_complete_validation.yaml')),
        DeclareLaunchArgument('navigation_enabled', default_value='false'),
        DeclareLaunchArgument('single_goal_mode', default_value='true'),
        DeclareLaunchArgument('max_navigation_goals', default_value='1'),
        DeclareLaunchArgument('labels_file', description='Actual deployed model label JSON, mandatory'),
        DeclareLaunchArgument('calibration_file', default_value=os.path.join(
            share,'calibration','calibration.npz'))]
    return LaunchDescription(actions+processes)
