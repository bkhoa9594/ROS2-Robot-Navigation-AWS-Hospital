import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import SetEnvironmentVariable, TimerAction
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import LaunchConfiguration 
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_bringup')

    nav2_yaml = os.path.join(pkg, 'config', 'nav2_params.yaml')
    #map_file  = os.path.join(pkg, 'maps',   'map_final_hospital.yaml')
    bt_xml    = os.path.join(pkg, 'config', 'mecanum_nav_bt.xml')
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')
    
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg, 'maps', 'map_final_hospital.yaml'),
        description='Full path to map yaml file to load'
    )
    map_file = LaunchConfiguration('map')

    # Jazzy lifecycle nodes — behavior_server replaces recoveries_server
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
        'collision_monitor',
    ]

    # cmd_vel routing:
    #   Nav2 controller_server → /cmd_vel_nav
    #   velocity_smoother      → /mobile_base_controller/cmd_vel
    # This allows future teleop to bypass the smoother cleanly.
    # nav2_cmd_vel_remap = [('/cmd_vel', '/mobile_base_controller/reference')]
    # smoother_remap = [
    #     ('cmd_vel',          '/cmd_vel_nav'),
    #     ('cmd_vel_smoothed', '/mobile_base_controller/reference'),
    # ]

    return LaunchDescription([
        declare_map_cmd,
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # ── Map Server ────────────────────────────────────────────────────────
        # Loads map_final_hospital and publishes /map latched.
        # AMCL and global_costmap both subscribe to this topic.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': True}], # Ép Sim Time ở đây [cite: 6, 11]
        ),
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                nav2_yaml,
                {'yaml_filename': map_file},
                {'use_sim_time': True},
            ],
        ),

        # ── AMCL ─────────────────────────────────────────────────────────────
        # Monte-Carlo localization using OmniMotionModel (required for mecanum).
        # Subscribes to /scan_front_raw and /tf; publishes /amcl_pose and TF.
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml],
            remappings=[('scan', '/scan_front_raw')],

        ),

        # ── Planner Server ────────────────────────────────────────────────────
        # NavFn A* global planner on the full hospital map.
        # use_astar=true is faster than Dijkstra on large maps.
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_yaml],
        ),

        # ── Controller Server ─────────────────────────────────────────────────
        # Runs DWB local planner with lateral velocity enabled (max_vel_y = 0.4).
        # Subscribes to /scan_front_raw for obstacle avoidance.
        # Publishes to /cmd_vel_nav (remapped above).
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml],
            #remappings=[('/cmd_vel', '/mobile_base_controller/reference')],
            remappings=[('/cmd_vel', '/cmd_vel_nav')],
        ),

        # ── Smoother Server ───────────────────────────────────────────────────
        # Post-processes the global path computed by planner_server.
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_yaml],
        ),

        # ── Behavior Server ───────────────────────────────────────────────────
        # Recovery behaviors: BackUp + Wait (Spin disabled for narrow corridors).
        # Jazzy renamed recoveries_server → behavior_server.
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_yaml],
            #remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),

        # ── BT Navigator ──────────────────────────────────────────────────────
        # Orchestrates the Nav2 pipeline via Behavior Tree (mecanum_nav_bt.xml).
        # Accepts NavigateToPose action goals from rviz2 or goal_client node.
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                nav2_yaml,
                {'default_nav_to_pose_bt_xml': bt_xml},
            ],
        ),

        # ── Velocity Smoother ─────────────────────────────────────────────────
        # Prevents mechanical jerk when mecanum switches direction abruptly.
        # Input:  /cmd_vel_nav  (from controller_server)
        # Output: /mobile_base_controller/cmd_vel  (to MecanumDriveController)
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_yaml],
            #remappings=smoother_remap,
            remappings=[
                ('/cmd_vel', '/cmd_vel_nav'), 
                ('/cmd_vel_smoothed', '/cmd_vel_smoothed')
            ],
        ),

        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[nav2_yaml],
            # remappings=[
            #     ('/cmd_vel_in_topic', '/cmd_vel_smoothed'),
            #     ('/cmd_vel_out_topic', '/mobile_base_controller/reference') # Dây nối thẳng xuống bánh xe của Khoa
            # ],
        ), 

        # ── Lifecycle Manager ─────────────────────────────────────────────────
        # Manages the lifecycle state of all Nav2 nodes.
        # autostart=True: transitions all nodes to Active on startup.
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[
                        nav2_yaml,
                        {'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}
                    ],
                ),
            ]
        ),
    ])
