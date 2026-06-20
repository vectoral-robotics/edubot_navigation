"""
Navigation bringup for EduBot.

Launches:
  - nav2_bringup (map_server, AMCL, planner, controller, recoveries, etc.)
  - optional RViz2 visualization (via edubot_viz)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # -----------------------------
    # Launch arguments
    # -----------------------------
    args = {
        'namespace': ('', 'Namespace for all navigation nodes'),
        'use_sim_time': ('false', 'Use simulation clock if true'),
        'use_rviz': ('true', 'Start RViz automatically'),
        'map': (
            PathJoinSubstitution([
                FindPackageShare('edubot_navigation'),
                'maps',
                'default.yaml',
            ]),
            'Full path to map YAML file'
        ),
        'params_file': (
            PathJoinSubstitution([
                FindPackageShare('edubot_navigation'),
                'param',
                'edubot_nav2.yaml',
            ]),
            'Full path to Nav2 parameters YAML file'
        ),
    }

    declare_args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, (default, desc) in args.items()
    ]

    # -----------------------------
    # Launch Configs
    # -----------------------------
    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # -----------------------------
    # Nav2 Bringup
    # -----------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'namespace': ns,
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': params_file,
        }.items(),
    )

    # -----------------------------
    # Visualization (from edubot_viz)
    # -----------------------------
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('edubot_viz'),
                'launch',
                'navigation_view.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items(),
        condition=IfCondition(use_rviz)
    )

    # -----------------------------
    # Return LaunchDescription
    # -----------------------------
    return LaunchDescription([
        *declare_args,
        nav2_launch,
        viz_launch,
    ])
