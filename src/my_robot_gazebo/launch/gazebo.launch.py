from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    pkg_share = FindPackageShare('my_robot_gazebo')

    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'maze1.world'
    ])

    # model_path = PathJoinSubstitution([
    #     pkg_share,
    #     'models'
    # ])

    model_path = os.path.expanduser(
        '~/ros2_ws/src/my_robot_gazebo/models'
    )

    return LaunchDescription([

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=model_path
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        )

    ])
