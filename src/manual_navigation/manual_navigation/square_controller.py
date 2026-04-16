import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

class SquareController(Node):
    def __init__(self):
        super().__init__('square_controller')

        self.pub = self.create_publisher(
            TwistStamped,
            '/mobile_base_controller/reference',
            10
        )

    def move(self, vx=0.0, vy=0.0, wz=0.0, duration=1.0):
        msg = TwistStamped()
        msg.header.frame_id = "base_footprint"

        start = time.time()
        while time.time() - start < duration:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = vx
            msg.twist.linear.y = vy
            msg.twist.angular.z = wz
            self.pub.publish(msg)
            time.sleep(0.05)

        self.stop()

    def stop(self):
        self.pub.publish(TwistStamped())

    def run_square(self):
        linear_speed = 0.2
        angular_speed = 0.5

        forward_time = 2.0 / linear_speed      # 2 meters
        rotate_time = 1.5708 / angular_speed   # 90 degrees

        for i in range(4):
            self.get_logger().info(f"Side {i+1}: Forward")

            self.move(
                vx=linear_speed,
                duration=forward_time
            )

            self.get_logger().info(f"Side {i+1}: Rotate 90 deg")  #thuc hien che do xoay

            self.move(
                wz=angular_speed,
                duration=rotate_time
            )


def main():
    rclpy.init()
    node = SquareController()

    time.sleep(2.0)
    node.run_square()

    rclpy.shutdown()