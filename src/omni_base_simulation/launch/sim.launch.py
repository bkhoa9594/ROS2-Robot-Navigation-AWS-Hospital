from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import Command, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    description_pkg = get_package_share_directory('omni_base_description')
    sim_pkg = get_package_share_directory('omni_base_simulation')
    controller_pkg = get_package_share_directory('omni_base_controller_configuration')

    xacro_file = os.path.join(description_pkg, 'robots', 'omni_base.urdf.xacro')
    world_file = os.path.join(sim_pkg, 'worlds', 'empty.sdf')
    controller_yaml = os.path.join(
        controller_pkg,
        'config',
        'mobile_base_controllers.yaml'
    )

    # Để Gazebo Sim tìm thấy plugin gz_ros2_control
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            '/opt/ros/jazzy/lib',
            ':',
            EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value='')
        ]
    )

    # Sinh robot_description từ xacro
    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' use_sim_time:=true',
            ' add_on_module:=no-add-on'
        ]),
        value_type=str
    )

    # Publish robot_description + TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    # Mở Gazebo Sim
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Spawn robot từ topic robot_description
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'omni_base',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    # Spawn trễ để chắc Gazebo đã mở
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    # Spawn controller: joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/controller_manager',
            '--param-file', controller_yaml
        ],
        output='screen'
    )

    # Spawn controller: imu_sensor_broadcaster
    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'imu_sensor_broadcaster',
            '-c', '/controller_manager',
            '--param-file', controller_yaml
        ],
        output='screen'
    )

    # Spawn controller: mobile_base_controller
    mobile_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mobile_base_controller',
            '-c', '/controller_manager',
            '--param-file', controller_yaml
        ],
        output='screen'
    )

    # Delay để chờ controller_manager sẵn sàng
    delayed_joint_state = TimerAction(
        period=6.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_imu = TimerAction(
        period=8.0,
        actions=[imu_sensor_broadcaster_spawner]
    )

    delayed_mobile_base = TimerAction(
        period=10.0,
        actions=[mobile_base_controller_spawner]
    )

    return LaunchDescription([
        gz_plugin_path,
        gazebo,
        robot_state_publisher_node,
        delayed_spawn,
        delayed_joint_state,
        delayed_imu,
        delayed_mobile_base
    ])
