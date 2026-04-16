"""
Unified launch: Gazebo + Web UI + NavManager.
SLAM and Nav2 are managed dynamically from the web UI.

Usage:
    ros2 launch robot_web_ui unified.launch.py
    Open http://localhost:8080
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port = LaunchConfiguration('port')
    web_ui_pkg = get_package_share_directory('robot_web_ui')
    rooms_file = os.path.join(web_ui_pkg, 'config', 'rooms.yaml')

    # 1. Gazebo + robot + controllers
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'), 'launch', 'sim_bridge.launch.py'            
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # 2. NavManager (delayed - wait for Gazebo controllers)
    nav_manager_node = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='robot_web_ui',
                executable='nav_manager',
                name='nav_manager',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ],
    )

    # 3. Web server (delayed)
    web_server_node = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='robot_web_ui',
                executable='web_server',
                name='web_server',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'port': port},
                    {'rooms_file': rooms_file},
                ],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        sim_bringup,
        nav_manager_node,
        web_server_node,
    ])
