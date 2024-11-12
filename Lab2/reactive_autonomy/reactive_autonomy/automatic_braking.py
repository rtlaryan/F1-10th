#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
import math

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class AutomaticBraking(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('automatic_braking')
        """
        One publisher should publish to the /drive topic with a AckermannDrive drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        odom_topic = '/odom'
        drive_topic = '/drive'

        self.speed = 0.0

        # define threshold for min iTTC before braking
        self.threshold = 0.88

        # Create ROS subscribers and publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            qos_profile = qos_profile
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)
        self.odom_subscription

        self.drive_publisher = self.create_publisher(
            AckermannDrive,
            drive_topic,
            1)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x #update current speed

    def scan_callback(self, scan_msg):
        # create array of angles scanned
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)

        # calculate range rate by mapping velocity (speed at 0 heading) onto angles
        range_rate = self.speed * np.cos(angles)

        # calculate iTTC
        iTTC = np.full((np.shape(range_rate)), np.nan)
        for i in range(np.shape(range_rate)[0]):
            if range_rate[i] > 0 and scan_msg.ranges[i] != 0:
                iTTC[i] = scan_msg.ranges[i] / range_rate[i]
            else:
                iTTC[i] = np.inf
        
        # report and publish
        if np.min(iTTC) < self.threshold:
            print("Brake!\n", np.min(iTTC))
            ackermann = AckermannDrive()
            ackermann.speed = 0.0
            for i in range(2000):
                self.drive_publisher.publish(ackermann)



def main(args=None):
    rclpy.init(args=args)
    automatic_braking = AutomaticBraking()
    rclpy.spin(automatic_braking)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    automatic_braking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
