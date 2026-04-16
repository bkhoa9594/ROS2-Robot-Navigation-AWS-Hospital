from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    description_pkg = get_package_share_directory("omni_base_description")

    xacro_file = os.path.join(
        description_pkg,
        "robots",
        "omni_base.urdf.xacro"
    )

    robot_description = Command(["xacro ", xacro_file])

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", "empty.sdf"],
        output="screen"
    )

    # Publish robot state
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description
        }]
    )

    # Spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "omni_base"
        ],
        output="screen"
    )

    # Load joint_state_broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # Load omni controller
    mobile_base_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mobile_base_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        mobile_base_controller
    ])