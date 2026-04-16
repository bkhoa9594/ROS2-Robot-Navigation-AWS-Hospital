import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
import time


class SquareController(Node):
    def __init__(self):
        super().__init__('square_controller')

        # Publisher
        self.pub = self.create_publisher(
            TwistStamped,
            '/mobile_base_controller/reference',
            10
        )

        # Odometry
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.create_subscription(
            Odometry,
            '/mobile_base_controller/odometry',
            self.odom_callback,
            10
        )

    # ========================
    # ODOM PROCESSING
    # ========================
    # def odom_callback(self, msg):
    #     self.current_x = msg.pose.pose.position.x
    #     self.current_y = msg.pose.pose.position.y
    #     self.current_yaw = self.get_yaw(msg)

    def odom_callback(self, msg):
        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Orientation → yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        yaw_deg = math.degrees(yaw)

        # Print nicely
        self.get_logger().info(
            f"x={x:.3f} m, y={y:.3f} m, yaw={yaw_deg:.2f} deg"
        )

        # Save for controller if needed
        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw

    def get_yaw(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def angle_diff(self, a, b):
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))

    # ========================
    # LOW-LEVEL COMMAND
    # ========================
    def publish_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz

        self.pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0, 0.0)

    # ========================
    # CLOSED-LOOP FORWARD
    # ========================
    def move_forward(self, distance=2.0, speed=0.2):
        self.get_logger().info("Moving forward (closed-loop)")

        start_x = self.current_x
        start_y = self.current_y

        while rclpy.ok():
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            dist = math.sqrt(dx*dx + dy*dy)

            error = distance - dist

            if error <= 0.01:
                break

            # simple proportional control
            vx = min(0.3, max(0.05, 0.8 * error))

            self.publish_cmd(vx=vx)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.stop()
        time.sleep(0.5)

    # ========================
    # CLOSED-LOOP ROTATION
    # ========================
    def rotate_90(self, angle=math.pi/2):
        self.get_logger().info("Rotating 90 deg (closed-loop)")

        start_yaw = self.current_yaw

        while rclpy.ok():
            diff = self.angle_diff(self.current_yaw, start_yaw)

            error = angle - abs(diff)

            if error <= 0.01:
                break

            # proportional control
            wz = min(0.8, max(0.1, 1.5 * error))

            # keep rotation direction consistent
            wz = wz if angle > 0 else -wz

            self.publish_cmd(wz=wz)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.stop()
        time.sleep(0.5)

    # ========================
    # MAIN LOGIC
    # ========================
    def run_square(self):
        time.sleep(2.0)  # wait for odom

        for i in range(4):
            self.get_logger().info(f"=== SIDE {i+1} ===")

            self.move_forward(2.0)
            self.rotate_90()

        self.get_logger().info("Square completed!")


def main():
    rclpy.init()
    node = SquareController()
    node.run_square()
    rclpy.shutdown()