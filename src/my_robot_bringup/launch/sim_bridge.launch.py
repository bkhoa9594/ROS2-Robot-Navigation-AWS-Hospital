import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time      = LaunchConfiguration('use_sim_time')
    slam_params_file  = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('my_robot_bringup'),
            'config',
            'mapper_params_online_async.yaml'
        ),
        description='Full path to the SLAM Toolbox parameters file'
    )

    bringup_path = get_package_share_directory('my_robot_bringup')
    ekf_config = os.path.join(bringup_path, 'config', 'ekf.yaml')

    urdf_file   = os.path.join(bringup_path, 'urdf', 'omni_base_simu.urdf')
    models_path = os.path.join(bringup_path, 'models')
    
    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_description = infp.read()

    pkg_share  = FindPackageShare('my_robot_bringup')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'hospital_full.world'])

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_slam_params_file_cmd,

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/jazzy/lib'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),

        # 1) Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),

        # 2) Publish robot description / static TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),

        # 3) Bridge sensors and clock from Gazebo to ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan_front_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/scan_rear_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/base_imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/base_rgbd_camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/base_rgbd_camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/base_rgbd_camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/base_rgbd_camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/roof_rgbd_camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/roof_rgbd_camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/roof_rgbd_camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/roof_rgbd_camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            parameters=[{'use_sim_time': True}], 
            remappings=[('/base_imu', '/imu')],
            output='screen'
        ),

        # 4) Spawn robot after Gazebo has started (3s delay)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_gz_sim', 'create',
                        '-name', 'my_robot',
                        '-file', urdf_file,
                        '-x', '0', '-y', '15', '-z', '0.1', '-Y', '-1.57'
                    ],
                    output='screen'
                ),
            ]
        ),

        # 5) Spawn ros2_control controllers after robot exists (8s delay)
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    parameters=[{'use_sim_time': True}], 
                    output='screen'
                ),
                # Node(
                #     package='controller_manager',
                #     executable='spawner',
                #     arguments=['imu_sensor_broadcaster'],
                #     output='screen'
                # ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['mobile_base_controller'],
                    output='screen'
                    #, 'reference_timeout:=1.0'
                ),
            ]
        ),

        # 6) TF relay: /mobile_base_controller/tf_odometry → /tf
        #    MecanumDriveController publishes TF to its own topic instead of /tf.
        #    Using topic_tools relay (C++) instead of Python for near-zero latency.
        #    Delay 13s: wait for mobile_base_controller to be fully active first.
        # TimerAction(
        #     period=13.0,
        #     actions=[
        #         Node(
        #             package='topic_tools',
        #             executable='relay',
        #             name='tf_odom_relay',
        #             arguments=[
        #                 '/mobile_base_controller/tf_odometry',
        #                 '/tf'
        #             ],
        #             output='screen'
        #         ),
        #     ]
        # ),
        # TimerAction(
        #     period=13.0,
        #     actions=[
        #         Node(
        #             package='topic_tools',
        #             executable='relay',
        #             name='tf_odom_relay',
        #             arguments=[
        #                 '/mobile_base_controller/tf_odometry',
        #                 '/tf'
        #             ],
        #             parameters=[{'use_sim_time': True}], 
        #             output='screen'
        #         ),
        #         # Node(
        #         #     package='topic_tools',
        #         #     executable='relay',
        #         #     name='cmd_vel_relay',
        #         #     arguments=[
        #         #         '/cmd_vel_nav',   
        #         #         '/mobile_base_controller/reference'
        #         #     ],
        #         #     output='screen'
        #         # ),
        #     ]
        # ),
        
        # # 7) SLAM Toolbox (14s delay: waits for TF relay to be up)
        # TimerAction(
        #     period=14.0,
        #     actions=[
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource([
        #                 os.path.join(
        #                     get_package_share_directory('slam_toolbox'),
        #                     'launch',
        #                     'online_async_launch.py'
        #                 )
        #             ]),
        #             launch_arguments={
        #                 'slam_params_file': slam_params_file,
        #                 'use_sim_time':     use_sim_time
        #             }.items()
        #         )
        #     ]
        # ),

        # 8) RViz2 (16s delay: waits for SLAM to initialize)
        TimerAction(
            period=16.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    parameters=[{'use_sim_time': True}],
                    output='screen'
                ),
            ]
        ),
    ])