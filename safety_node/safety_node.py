import rclpy
import numpy as np
import math
from rclpy.node import Node 
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class Safety(Node):
    def __init__(self):
        super().__init__('safety')

        self.speed = 0.0

        self.scan_subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback,
            10
        )

        self.odom_subscriber = self.create_subscription(
            Odometry, 
            '/ego_racecar/odom', 
            self.odom_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            AckermannDriveStamped, 
            '/drive', 
            10
        )

    def odom_callback(self, msg):
        self.speed = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        num_beams = len(ranges)
        angles = msg.angle_min + np.arange(num_beams) * msg.angle_increment
        range_rate = self.speed * np.cos(angles)
        range_rate = np.maximum(range_rate, 0.001)

        ttc = ranges / range_rate


        ttc = np.nan_to_num(ttc, posinf=100.0, nan=100.0)

        front_indices = np.abs(angles) < 0.35
        front_ttc = ttc[front_indices]

        threshold = 0.6
        if np.sum(front_ttc < threshold) > 10:
            self.publish_brake()
            self.get_logger().info('EMERGENCY BRAKE!')

    def publish_brake(self):
        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = 0.0
        self.publisher_.publish(brake_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Safety()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()