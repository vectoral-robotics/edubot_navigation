"""
Navigation bringup for OmniBot.

Launches:
  - nav2_bringup (map_server, AMCL, planner, controller, recoveries, etc.)
  - optional RViz2

All arguments are exposed for flexibility.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
            os.path.join(
                get_package_share_directory('omnibot_navigation'),
                'maps',
                'default.yaml'
            ),
            'Full path to map YAML file'
        ),
        'params_file': (
            os.path.join(
                get_package_share_directory('omnibot_navigation'),
                'param',
                'omnibot_nav2.yaml'
            ),
            'Full path to Nav2 parameters YAML file'
        ),
        'rviz_config': (
            os.path.join(
                get_package_share_directory('omnibot_navigation'),
                'rviz',
                'nav2_default.rviz'
            ),
            'Path to RViz configuration file'
        ),
    }

    declare_args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, (default, desc) in args.items()
    ]

    # -----------------------------
    # LaunchConfigurations
    # -----------------------------
    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    # -----------------------------
    # Nav2 bringup
    # -----------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]
        ),
        launch_arguments={
            'namespace': ns,
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': params_file,
        }.items(),
    )

    # -----------------------------
    # RViz2 (optional)
    # -----------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # -----------------------------
    # Final LaunchDescription
    # -----------------------------
    return LaunchDescription([
        *declare_args,
        nav2_launch,
        rviz_node,
    ])
