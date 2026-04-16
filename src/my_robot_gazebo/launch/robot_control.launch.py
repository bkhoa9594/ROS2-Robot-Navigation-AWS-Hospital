from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    robot_path = get_package_share_directory('my_robot')
    urdf_file = os.path.join(robot_path, 'urdf', 'my_robot_gazebo.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
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
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'my_robot',
                '-file', urdf_file,
                '-x', '0',
                '-y', '0',
                '-z', '0.1'
            ],
            output='screen'
            ),
        
        # Publish robot TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        
        # ROS <-> Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # Velocity commands (ROS 2 -> GZ) 
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                # Odometry (GZ -> ROS 2) 
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                # Camera Image (GZ -> ROS 2) 
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                # Camera Info (GZ -> ROS 2) thong so noi tai camera: width height K matrix distortion focal length
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                # Joint States (GZ -> ROS 2), gazebo gui trang thai khop
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
            ],
            remappings=[ #Doi ten thanh '/camera/image_raw' -> '/my_robot/camera1/image_raw'
                ('/camera/image_raw', '/my_robot/camera1/image_raw'),
            ], #khi co nhieu camera, tranh trung topic cho moi camera
            output='screen'
        )

    ])
