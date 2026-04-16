"""
SLAM mapping with Web UI for manual control.
Use the joystick/keyboard in the web UI to drive around and build the map.
Then save: ros2 run nav2_map_server map_saver_cli -f ~/maps/hospital
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
    port = LaunchConfiguration('port')

    # 1. Gazebo + SLAM (Cartographer)
    slam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_omni'), 'launch', 'create_map.launch.py'
            ])
        ),
        launch_arguments={'start_rviz': 'false'}.items(),
    )

    # 2. Web UI for manual control during mapping
    web_ui_bringup = TimerAction(
        period=15.0,
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
        DeclareLaunchArgument('port', default_value='8080'),
        slam_bringup,
        web_ui_bringup,
    ])
