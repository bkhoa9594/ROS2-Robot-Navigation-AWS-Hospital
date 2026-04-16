from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    robot_path = get_package_share_directory('my_robot')
    urdf_file = os.path.join(robot_path, 'urdf', 'my_robot_gazebo.urdf')
    
    pkg_share = FindPackageShare('my_robot_gazebo')

    world_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'maze1.world'
    ])

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
        ),

         # Spawn robot
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create', # tao them 1 doi tuong
                '-name', 'my_robot',                   # ten doi tuong do
                '-file', urdf_file,                    # ten file doi tuong
                '-x', '0',   #toa do xyz ben trong moi truong
                '-y', '0',
                '-z', '0.1' #base_link nam duoi banh xe 1 khuc, de 0.1 de vua cham dat
            ],
            output='screen')

    ])
