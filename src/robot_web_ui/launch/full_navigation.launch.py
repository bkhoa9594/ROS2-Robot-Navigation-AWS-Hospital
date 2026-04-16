"""
Full pipeline launch: Gazebo + Navigation + Web UI
Usage:
  # Step 1: Create map first
  ros2 launch robot_omni create_map.launch.py

  # Step 2: Save map
  ros2 run nav2_map_server map_saver_cli -f ~/maps/hospital

  # Step 3: Run full navigation with web UI
  ros2 launch robot_web_ui full_navigation.launch.py map:=~/maps/hospital.yaml
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    port = LaunchConfiguration('port')

    # 1. Gazebo + robot + controllers
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_omni'), 'launch', 'gazebo_control.launch.py'
            ])
        ),
    )

    # 2. Nav2 with map (delayed to let sim start)
    nav2_bringup = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('nav2_simple_navigation'),
                        'launch', 'nav2_map_navigation.launch.py'
                    ])
                ),
                launch_arguments={'map': map_file}.items(),
            ),
        ],
    )

    # 3. Web UI (delayed)
    web_ui_bringup = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('robot_web_ui'),
                        'launch', 'web_ui.launch.py'
                    ])
                ),
                launch_arguments={'port': port}.items(),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Path to map yaml file'),
        DeclareLaunchArgument('port', default_value='8080'),
        sim_bringup,
        nav2_bringup,
        web_ui_bringup,
    ])
