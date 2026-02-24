#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        self.declare_parameter('ttc_threshold', 0.5)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.get_logger().info('Safety Node Initialized')

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        
        ranges[np.isinf(ranges)] = 200
        ranges[np.isnan(ranges)] = 200

        angles= np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        range_rate = self.speed * np.cos(angles)

        denominators = np.maximum(0.0001, range_rate)

        time_to_collision = ranges / denominators
        min_time = np.min(time_to_collision)

        threshold = self.get_parameter('ttc_threshold').get_parameter_value().double_value

        if min_time < threshold:
            self.get_logger().info(f'Emergency brake! TTC: {min_time:.3f} < threshold: {threshold:.2f}')
            self.drive_publisher.publish(AckermannDriveStamped(drive=AckermannDrive(speed=0.0)))

 

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()