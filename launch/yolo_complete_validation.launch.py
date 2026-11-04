"""New opt-in entry point. Does not launch base, Nav2, SLAM, camera or YOLO."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('ee3033_search_policy')
    params = LaunchConfiguration('params_file')
    nav = LaunchConfiguration('navigation_enabled')
    labels = LaunchConfiguration('labels_file')
    calibration = LaunchConfiguration('calibration_file')
    processes = []
    modules = ['continuous_observation_adapter', 'stamped_target_localizer_node',
               'target_mission_coordinator', 'guarded_search_manager']
    for module in modules:
        cmd = ['/usr/bin/python3', '-m', 'ee3033_search_policy.'+module,
               '--ros-args', '--params-file', params]
        if module == 'guarded_search_manager':
            cmd += ['-p', ['navigation_enabled:=', nav],
                    '-p', 'return_home_enabled:=false',
                    '-p', 'single_goal_mode:=true', '-p', 'max_navigation_goals:=1']
        if module in ('continuous_observation_adapter','stamped_target_localizer_node'):
            cmd += ['-p', ['calibration_file:=',calibration]]
        if module == 'continuous_observation_adapter':
            cmd += ['-p', ['labels_file:=',labels]]
        processes.append(ExecuteProcess(cmd=cmd, output='screen', name=module))
    actions = [DeclareLaunchArgument('params_file', default_value=os.path.join(
        share,'config','yolo_complete_validation.yaml')),
        DeclareLaunchArgument('navigation_enabled', default_value='false'),
        DeclareLaunchArgument('labels_file', description='Actual deployed model label JSON, mandatory'),
        DeclareLaunchArgument('calibration_file', default_value=os.path.join(
            share,'calibration','calibration.npz'))]
    # Keep guard alive if another child dies: its lease watchdog can then cancel.
    # Guard-process or host failure still requires physical emergency intervention.
    return LaunchDescription(actions+processes)
