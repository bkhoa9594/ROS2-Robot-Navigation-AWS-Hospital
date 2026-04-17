"""
ROS2 Navigation Manager Node — unified controller for SLAM / Nav2 / Idle modes.
Manages Cartographer and Nav2 as subprocesses, switchable from the web UI.
"""
import math
import json
import os
import signal
import subprocess
import threading
import glob as globmod
import atexit

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


MAPS_DIR = os.path.expanduser('~/ros2_ws/src/my_robot_bringup/maps')

class ProcessGroup:
    """Manages a group of subprocesses that can be killed together."""

    def __init__(self, logger):
        self._procs = []
        self._logger = logger

    def run(self, cmd, env=None):
        merged = dict(os.environ)
        if env:
            merged.update(env)
        self._logger.info(f'Spawning: {" ".join(cmd)}')
        proc = subprocess.Popen(
            cmd,
            preexec_fn=os.setsid,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=merged,
        )
        self._procs.append(proc)
        # Log subprocess output in background thread
        threading.Thread(
            target=self._drain_output,
            args=(proc, cmd[2] if len(cmd) > 2 else cmd[0]),
            daemon=True,
        ).start()
        return proc

    def _drain_output(self, proc, name):
        try:
            for line in proc.stdout:
                text = line.decode('utf-8', errors='replace').strip()
                if text:
                    self._logger.info(f'[{name}] {text}')
        except Exception:
            pass

    def kill_all(self):
        for proc in self._procs:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except (ProcessLookupError, OSError):
                pass
        for proc in self._procs:
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, OSError):
                    pass
        self._procs.clear()

    @property
    def alive(self):
        return any(p.poll() is None for p in self._procs)


class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')

        # Mode: idle | slam | nav
        self._mode = 'idle'
        self._robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self._current_path = []
        self.create_subscription(Path, '/plan', self._path_callback, 10)
        self._nav_status = 'idle'
        self._current_goal_name = ''
        self._map_data = None
        self._map_version = 0
        self._goal_handle = None
        self._lock = threading.Lock()

        # Subprocess groups
        self._slam_procs = None
        self._nav_procs = None

        # Ensure maps directory
        os.makedirs(MAPS_DIR, exist_ok=True)

        # Resolve package paths once
        # from ament_index_python.packages import get_package_share_directory
        # self._robot_omni_share = get_package_share_directory('robot_omni')
        # self._nav_pkg_share = get_package_share_directory('nav2_simple_navigation')
        
        # Resolve package paths once
        from ament_index_python.packages import get_package_share_directory
        self._my_bringup_share = get_package_share_directory('my_robot_bringup')  

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self._cmd_vel_pub = self.create_publisher(
            TwistStamped, 'cmd_vel', 10)
        self._status_pub = self.create_publisher(
            String, '/robot_web_ui/status', 10)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Map subscriber
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos)

        # Command listener
        self.create_subscription(
            String, '/robot_web_ui/command', self._command_callback, 10)

        # Timers
        self.create_timer(0.1, self._update_pose_from_tf)
        self.create_timer(0.5, self._publish_status)

        # Cleanup on exit
        atexit.register(self._cleanup)

        self.get_logger().info('NavManager ready (unified mode)')

    def _cleanup(self):
        if self._slam_procs:
            self._slam_procs.kill_all()
        if self._nav_procs:
            self._nav_procs.kill_all()

    # ===================== TF Pose =====================

    def _update_pose_from_tf(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            pos = t.transform.translation
            rot = t.transform.rotation
            yaw = self._quat_to_yaw(rot)
            with self._lock:
                self._robot_pose = {
                    'x': round(pos.x, 3),
                    'y': round(pos.y, 3),
                    'yaw': round(yaw, 3),
                }
        except Exception:
            # Fall back: try odom frame
            try:
                t = self._tf_buffer.lookup_transform(
                    'odom', 'base_link', rclpy.time.Time())
                pos = t.transform.translation
                rot = t.transform.rotation
                yaw = self._quat_to_yaw(rot)
                with self._lock:
                    self._robot_pose = {
                        'x': round(pos.x, 3),
                        'y': round(pos.y, 3),
                        'yaw': round(yaw, 3),
                    }
            except Exception:
                pass

    # ===================== Map Callback =====================

    def _map_callback(self, msg):
        info = msg.info
        with self._lock:
            self._map_version += 1
            self._map_data = {
                'version': self._map_version,
                'width': info.width,
                'height': info.height,
                'resolution': info.resolution,
                'origin_x': info.origin.position.x,
                'origin_y': info.origin.position.y,
                'data': list(msg.data),
            }
        self.get_logger().info(
            f'Map received: {info.width}x{info.height}')
    
    #BONUS
    def _path_callback(self, msg):
        path = []
        for i in range(0, len(msg.poses), 3):
            path.append({
                'x': round(msg.poses[i].pose.position.x, 3),
                'y': round(msg.poses[i].pose.position.y, 3)
            })
        if msg.poses and (len(msg.poses) - 1) % 3 != 0:
            path.append({
                'x': round(msg.poses[-1].pose.position.x, 3),
                'y': round(msg.poses[-1].pose.position.y, 3)
            })
        with self._lock:
            self._current_path = path
    # ===================== Command Dispatch =====================

    def _command_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        action = cmd.get('action', '')

        # Mode switching
        if action == 'start_slam':
            self._start_slam()
        elif action == 'stop_slam':
            self._stop_slam()
        elif action == 'save_map':
            self._save_map(cmd.get('name', 'map'))
        elif action == 'start_nav':
            self._start_nav(cmd.get('map', ''))
        elif action == 'stop_nav':
            self._stop_nav()

        # Navigation
        elif action == 'navigate':
            if self._mode == 'nav':
                self._send_nav_goal(
                    cmd['x'], cmd['y'], cmd.get('yaw', 0.0),
                    cmd.get('name', 'Custom'))
            else:
                self.get_logger().warn('Cannot navigate: not in NAV mode')
        elif action == 'cancel':
            self._cancel_nav()

        # Manual control (always available)
        elif action == 'cmd_vel':
            self._send_cmd_vel(cmd.get('linear', 0.0), cmd.get('angular', 0.0))
        elif action == 'stop':
            self._send_cmd_vel(0.0, 0.0)
        elif action == 'set_initial_pose':
            self._set_initial_pose(cmd['x'], cmd['y'], cmd.get('yaw', 0.0))

    # ===================== SLAM Mode =====================

    # def _start_slam(self):
    #     if self._mode != 'idle':
    #         self.get_logger().warn(f'Cannot start SLAM: mode is {self._mode}')
    #         return

    #     self.get_logger().info('Starting SLAM mode...')
    #     self._slam_procs = ProcessGroup(self.get_logger())

    #     config_dir = os.path.join(
    #         self._robot_omni_share, 'config', 'cartographer')

    #     # Cartographer node
    #     # Cartographer args come BEFORE --ros-args
    #     self._slam_procs.run([
    #         'ros2', 'run', 'cartographer_ros', 'cartographer_node',
    #         '-configuration_directory', config_dir,
    #         '-configuration_basename', 'omni_base_2d.lua',
    #         '--ros-args',
    #         '-p', 'use_sim_time:=true',
    #         '-r', 'scan:=/scan_front_raw',
    #     ])

    #     # Occupancy grid publisher
    #     self._slam_procs.run([
    #         'ros2', 'run', 'cartographer_ros',
    #         'cartographer_occupancy_grid_node',
    #         '--ros-args',
    #         '-p', 'use_sim_time:=true',
    #         '-p', 'resolution:=0.05',
    #         '-p', 'publish_period_sec:=1.0',
    #     ])

    #     # Odom frame bridge
    #     self._slam_procs.run([
    #         'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
    #         '--frame-id', 'odom',
    #         '--child-frame-id', 'robot_simulation/odom',
    #         '--ros-args', '-p', 'use_sim_time:=true',
    #     ])

    #     with self._lock:
    #         self._mode = 'slam'
    #     self.get_logger().info('SLAM mode started')
    
    def _start_slam(self):
        if self._mode != 'idle': return
        self.get_logger().info('Starting SLAM Toolbox mode...')
        self._slam_procs = ProcessGroup(self.get_logger())

        slam_params = os.path.join(self._my_bringup_share, 'config', 'mapper_params_online_async.yaml')

        self._slam_procs.run([
            'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
            f'slam_params_file:={slam_params}',
            'use_sim_time:=true'
        ])

        with self._lock:
            self._mode = 'slam'

    def _stop_slam(self):
        if self._slam_procs:
            self.get_logger().info('Stopping SLAM...')
            self._slam_procs.kill_all()
            self._slam_procs = None
        with self._lock:
            self._mode = 'idle'
        self.get_logger().info('SLAM stopped, mode: idle')

    def _save_map(self, name):
        name = name.strip().replace(' ', '_') or 'map'
        map_path = os.path.join(MAPS_DIR, name)
        self.get_logger().info(f'Saving map to {map_path}...')

        try:
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', map_path,
                 '--ros-args',
                 '-p', 'use_sim_time:=true',
                 '-p', 'map_subscribe_transient_local:=true',
                 '-p', 'save_map_timeout:=10000.0',
                 '-p', 'free_thresh_default:=0.25',
                 '-p', 'occupied_thresh_default:=0.65'],
                capture_output=True, text=True, timeout=30.0,
            )
            if result.returncode == 0:
                self.get_logger().info(f'Map saved: {map_path}.yaml')
            else:
                self.get_logger().error(
                    f'Map save failed: {result.stderr}')
        except subprocess.TimeoutExpired:
            self.get_logger().error('Map save timed out')

    # ===================== Navigation Mode =====================

    def _start_nav(self, map_yaml):
        if self._mode == 'slam':
            self._stop_slam()
            # Wait for TF to settle
            import time
            time.sleep(2.0)

        if self._mode != 'idle':
            self.get_logger().warn(f'Cannot start NAV: mode is {self._mode}')
            return

        map_path = os.path.join(MAPS_DIR, map_yaml)
        if not os.path.isfile(map_path):
            self.get_logger().error(f'Map file not found: {map_path}')
            return

        self.get_logger().info(f'Starting NAV mode with map: {map_path}')
        self._nav_procs = ProcessGroup(self.get_logger())

        # Use the existing launch file — it handles all node names,
        # params, remappings, and lifecycle manager correctly.
        # Remember current pose (from SLAM or odom) before switching
        with self._lock:
            last_pose = dict(self._robot_pose)

        # self._nav_procs.run([
        #     'ros2', 'launch', 'nav2_simple_navigation',
        #     'nav2_map_navigation.launch.py',
        #     f'map:={map_path}',
        # ])

        self._nav_procs.run([
            'ros2', 'launch', 'my_robot_bringup', 'navigation.launch.py', # Sửa ở đây
            f'map:={map_path}',
            'use_sim_time:=true',  
        ])

        with self._lock:
            self._mode = 'nav'
            self._nav_status = 'idle'
        self.get_logger().info('NAV mode started')

        # Auto-set initial pose after Nav2 boots (in background)
        def _set_init_pose():
            import time
            time.sleep(8.0)  # Wait for AMCL to be ready
            self.get_logger().info(
                f'Setting initial pose to last known: '
                f'x={last_pose["x"]}, y={last_pose["y"]}, yaw={last_pose["yaw"]}')
            self._set_initial_pose(
                last_pose['x'], last_pose['y'], last_pose['yaw'])

        threading.Thread(target=_set_init_pose, daemon=True).start()

    def _stop_nav(self):
        if self._nav_procs:
            self.get_logger().info('Stopping Nav2...')
            self._nav_procs.kill_all()
            self._nav_procs = None
        with self._lock:
            self._mode = 'idle'
            self._nav_status = 'idle'
        self.get_logger().info('Nav2 stopped, mode: idle')

    # ===================== Nav2 Goal Sending =====================

    def _send_nav_goal(self, x, y, yaw, name=''):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 action server not available')
            with self._lock:
                self._nav_status = 'failed'
            return
        #BONUS
        with self._lock:
            if self._goal_handle is not None:
                self.get_logger().info('Canceling previous goal before sending a new one...')
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        q = self._yaw_to_quat(float(yaw))
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        with self._lock:
            self._nav_status = 'navigating'
            self._current_goal_name = name

        self.get_logger().info(f'Navigating to "{name}" ({x:.2f}, {y:.2f})')

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback)
        future.add_done_callback(self._nav_goal_response)

    def _nav_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            with self._lock:
                self._nav_status = 'failed'
            return
        with self._lock:
            self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_feedback(self, feedback_msg):
        pass

    def _nav_result(self, future):
        result = future.result()
        with self._lock:
            self._goal_handle = None
            if result.status == 4:  # SUCCEEDED
                self._nav_status = 'succeeded'
                self.get_logger().info(
                    f'Arrived at "{self._current_goal_name}"')
            elif result.status == 5:  # CANCELED
                self._nav_status = 'canceled'
            else:
                self._nav_status = 'failed'

    def _cancel_nav(self):
        with self._lock:
            gh = self._goal_handle
        if gh is not None:
            gh.cancel_goal_async()
        with self._lock:
            self._nav_status = 'canceled'

    # ===================== Manual Control =====================

    def _send_cmd_vel(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self._cmd_vel_pub.publish(msg)

    def _set_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        q = self._yaw_to_quat(float(yaw))
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self._initial_pose_pub.publish(msg)

    # ===================== Status =====================

    def _publish_status(self):
        maps = self._list_maps()
        with self._lock:
            status = {
                'pose': self._robot_pose,
                'nav_status': self._nav_status,
                'goal_name': self._current_goal_name,
                'mode': self._mode,
                'available_maps': maps,
                'path': self._current_path,  # <--- ADD ROW
            }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    def get_status(self):
        with self._lock:
            return {
                'pose': dict(self._robot_pose),
                'nav_status': self._nav_status,
                'goal_name': self._current_goal_name,
                'mode': self._mode,
                'available_maps': self._list_maps(),
            }

    def get_map_data(self):
        with self._lock:
            return self._map_data

    def get_map_version(self):
        with self._lock:
            return self._map_version

    def _list_maps(self):
        files = globmod.glob(os.path.join(MAPS_DIR, '*.yaml'))
        return sorted([os.path.basename(f) for f in files])

    # ===================== Helpers =====================

    @staticmethod
    def _quat_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def _yaw_to_quat(yaw):
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


def main(args=None):
    rclpy.init(args=args)
    node = NavManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cleanup()
        node.destroy_node()
        #rclpy.shutdown()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
