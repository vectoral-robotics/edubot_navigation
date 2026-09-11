"""Saved-map navigation with all autonomous motion routed through the manager."""

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare("edubot_navigation"), "param", "edubot_differential.yaml"]
    )
    sim_time = LaunchConfiguration("use_sim_time")
    nodes = [
        ("nav2_controller", "controller_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_smoother", "smoother_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Absolute path to the saved map YAML"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"]
                    )
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "params_file": params,
                    "use_sim_time": sim_time,
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            ),
            *[
                Node(
                    package=package,
                    executable=name,
                    name=name,
                    output="screen",
                    parameters=[params, {"use_sim_time": sim_time}],
                    remappings=[("cmd_vel", "/navigation/cmd_vel")],
                )
                for package, name in nodes
            ],
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": sim_time,
                        "autostart": True,
                        "node_names": [name for _, name in nodes],
                    }
                ],
            ),
        ]
    )
