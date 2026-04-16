"""
Launch the web UI server and NavManager nodes.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_web_ui')
    rooms_file = os.path.join(pkg_share, 'config', 'rooms.yaml')

    port = LaunchConfiguration('port')

    nav_manager_node = Node(
        package='robot_web_ui',
        executable='nav_manager',
        name='nav_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    web_server_node = Node(
        package='robot_web_ui',
        executable='web_server',
        name='web_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'port': port},
            {'rooms_file': rooms_file},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        nav_manager_node,
        web_server_node,
    ])
