"""Launch the non-actuating search-policy readiness node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("ee3033_search_policy"))
    default_params = str(share / "config" / "search_policy.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
        ),
        DeclareLaunchArgument(
            "weights_path",
            default_value="",
        ),
        Node(
            package="ee3033_search_policy",
            executable="search_policy_dry_run",
            name="search_policy_dry_run",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"weights_path": LaunchConfiguration("weights_path")},
            ],
        ),
    ])
