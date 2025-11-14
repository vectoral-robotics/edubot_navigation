"""
SLAM bringup for OmniBot using slam_toolbox.

Launches:
  - slam_toolbox (online_sync) with OmniBot-specific parameters
  - optional RViz2 visualization (navigation view)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -------------------------------------------------------------------------
    # Launch arguments (mirrors omnibot_navigation style)
    # -------------------------------------------------------------------------
    args = {
        'namespace': ('', 'Namespace applied to all SLAM nodes'),
        'use_sim_time': ('false', 'Use simulated time from /clock if true'),
        'use_rviz': ('true', 'Start RViz2 navigation view for mapping'),
        'slam_params_file': (
            PathJoinSubstitution([
                FindPackageShare('omnibot_navigation'),
                'param',
                'omnibot_slam.yaml',
            ]),
            'Full path to the slam_toolbox parameter YAML file',
        ),
    }

    declare_args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, (default, desc) in args.items()
    ]

    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    slam_params = LaunchConfiguration('slam_params_file')

    # -------------------------------------------------------------------------
    # SLAM Toolbox (online sync)
    # -------------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_sync_launch.py',
            ])
        ),
        launch_arguments={
            'namespace': ns,
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Optional RViz visualization (navigation view)
    # -------------------------------------------------------------------------
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('omnibot_viz'),
                'launch',
                'navigation_view.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items(),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        *declare_args,
        slam_launch,
        viz_launch,
    ])
