"""Launch every Stage-2 runtime interface with safe navigation opt-in."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ee3033_search_policy'))
    params = str(share / 'config' / 'search_policy.yaml')
    common = LaunchConfiguration('params_file')
    navigation = LaunchConfiguration('navigation_enabled')
    weights = LaunchConfiguration('weights_path')
    single_goal_mode = LaunchConfiguration('single_goal_mode')
    max_navigation_goals = LaunchConfiguration('max_navigation_goals')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('weights_path', default_value=''),
        DeclareLaunchArgument(
            'navigation_enabled',
            default_value='false',
            description='Must be explicitly true before SearchManager sends goals',
        ),
        DeclareLaunchArgument(
            'single_goal_mode',
            default_value='true',
            description='Stop after the configured number of navigation goal attempts',
        ),
        DeclareLaunchArgument(
            'max_navigation_goals',
            default_value='1',
            description='Maximum NavigateToPose dispatches in single-goal mode',
        ),
        Node(
            package='ee3033_search_policy',
            executable='yolo_ncnn_detector',
            name='yolo_ncnn_detector',
            output='screen',
            parameters=[common],
        ),
        Node(
            package='ee3033_search_policy',
            executable='target_checker',
            name='target_checker',
            output='screen',
            parameters=[common],
        ),
        Node(
            package='ee3033_search_policy',
            executable='target_localizer',
            name='target_localizer',
            output='screen',
            parameters=[common],
        ),
        Node(
            package='ee3033_search_policy',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[common],
        ),
        Node(
            package='ee3033_search_policy',
            executable='search_manager',
            name='search_manager',
            output='screen',
            parameters=[
                common,
                {
                    'weights_path': weights,
                    'navigation_enabled': navigation,
                    'single_goal_mode': single_goal_mode,
                    'max_navigation_goals': max_navigation_goals,
                },
            ],
        ),
    ])
