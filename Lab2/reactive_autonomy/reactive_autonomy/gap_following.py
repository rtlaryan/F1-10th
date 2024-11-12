import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class GapFollowing(Node):
    """ 
    Implement Follow the Gap on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('gap_following')

        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # create ROS subscribers and publishers
        self.scan_subscription = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            qos_profile = qos_profile)
        self.scan_subscription

        self.drive_publisher = self.create_publisher(
            AckermannDrive,
            drive_topic,
            1)
        
        self.counter = 0

        # threshold to count as part of a gap
        self.threshold = 1.5

        self.last_gap_angle = np.inf


    def preprocess_lidar(self, ranges_, angles_):
        """ Preprocess the LiDAR scan array. Recommended implementation includes:
            1. Setting each value to the mean over some window
            2. Rejecting high values (eg. > 3m)
        """

        # remove nans and infs
        ranges_ = np.nan_to_num(ranges_)

        # restrain angles
        angle_to_restrain_to = 90
        angle_filter = np.abs(angles_) < (angle_to_restrain_to*np.pi/180)
        angles_ = angles_[angle_filter]
        ranges_ = ranges_[angle_filter]

        # restrain ranges
        range_to_restrain_to = 15
        range_filter = ranges_ > range_to_restrain_to
        ranges_[range_filter] = range_to_restrain_to

        # apply moving average filter
        ma_length = 5
        window = []
        for i in range(len(ranges_)):
            window.append(ranges_[i])
            if len(window) > ma_length:
                window.pop(0)
            if len(window) == ma_length:
                ranges_[i] = sum(window) / len(window)

        proc_ranges = ranges_
        proc_angles = angles_
        return proc_ranges, proc_angles


    def find_max_gap(self, ranges_, angles_):
        """ Return the start index & end index of the largest gap (longest consecutive string of 0s) in free_space_ranges
        """

        gap = []
        angle = []

        max_gap = []
        max_angle = []

        # find the sequences of values (gaps) above a threshold
        for i in range(np.shape(ranges_)[0]):
            if ranges_[i] >= self.threshold:
                gap.append(ranges_[i])
                angle.append(angles_[i])
            else:
                if len(gap) > len(max_gap):
                    max_gap = gap
                    max_angle = angle
                    gap = []
                    angle = []

        np_max_angle = np.array(max_angle)
        max_angle_gap = np.mean(np_max_angle)

        return max_angle_gap
    

    def scan_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDrive Message
        """
        ranges = np.array(data.ranges)
        angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)

        # preprocess ranges and angles
        proc_ranges, proc_angles = self.preprocess_lidar(ranges, angles)

        # find closest point to LiDAR
        closest_point = np.argmin(proc_ranges)

        # eliminate all points inside 'bubble' (set them to zero) 
        bubble = 8
        for i in range(-bubble, bubble+1):
            if (closest_point + i) >= 0 and (closest_point + i) < len(proc_ranges):
                proc_ranges[closest_point + i] = 0

        # find best gap 
        max_gap_angle = self.find_max_gap(proc_ranges, proc_angles)

        # publish Drive message
        if np.abs(max_gap_angle) < np.deg2rad(8):
            gap_direction = "Straight"
        elif np.abs(max_gap_angle) > np.deg2rad(8) and np.abs(max_gap_angle) < np.deg2rad(15):
            if max_gap_angle < 0:
                gap_direction = "Right"
            else:
                gap_direction = "Left"
        elif np.abs(max_gap_angle) > np.deg2rad(15) and np.abs(max_gap_angle) < np.deg2rad(90):
            if max_gap_angle < 0:
                gap_direction = "Hard Right"
            else:
                gap_direction = "Hard Left"
        elif np.abs(max_gap_angle) > np.deg2rad(90):
            if max_gap_angle < 0:
                gap_direction = "Back Right"
            else:
                gap_direction = "Back Left"

        print("Direction to gap:\n", gap_direction)

        if np.abs(self.last_gap_angle - max_gap_angle) > 0.03:
            ackermann = AckermannDrive()
            ackermann.steering_angle = max_gap_angle
            ackermann.steering_angle_velocity = 0.5
            ackermann.speed = 0.5
            self.drive_publisher.publish(ackermann)
            self.last_gap_angle = max_gap_angle


def main(args=None):
    rclpy.init(args=args)
    gap_following = GapFollowing()
    rclpy.spin(gap_following)

    gap_following.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()