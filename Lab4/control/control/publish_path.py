#!/usr/bin/env python3
import sys
import csv
import time
import rclpy
from rclpy.node import Node
from rosidl_runtime_py import message_to_csv
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import tf_transformations


class PublishPathNode(Node):
    def __init__(self, filename):
        super().__init__("record_path_node")

        self.declare_parameter("rate", 50)
        self.rate =  self.get_parameter("rate").get_parameter_value().integer_value
        self.path = []
        self.current_pose = None
        self.filename = filename

        transient_local = QoSProfile(
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )


        self.path_pub = self.create_publisher(PoseArray, "/cmd_path", qos_profile = transient_local)


    def sendPath(self):
        path_msg = PoseArray()
        with open(self.filename, 'r') as csv_file:
            reader = csv.reader(csv_file, delimiter = ',')

            values = next(reader)

            if len(values) < 10:
                print("invalid or empty path file")
                return

            path_msg.header.frame_id = values[2]

            index = 3

            try:
                while index < len(values):
                    pose = Pose()
                    pose.position.x = float(values[index + 0])
                    pose.position.y = float(values[index + 1])
                    pose.position.z = float(values[index + 2])
                    pose.orientation.x =float(values[index + 3])
                    pose.orientation.y =float(values[index + 4])
                    pose.orientation.z =float(values[index + 5])
                    pose.orientation.w =float(values[index + 6])

                    path_msg.poses.append(pose)
                    index += 7
            except IndexError:
                print("index error at: ", index)
                pass

            self.path_pub.publish(path_msg)
            time.sleep(1.0)
            print("Path of length ", (index - 3 // 7)," published from: ", self.filename)
            return

def main():
    rclpy.init()

    filename = "path.csv"
    if len(sys.argv) < 2:
        print("default filename: 'path.csv'")
    else:
        filename = sys.argv[1]

    publish_path_node = PublishPathNode(filename)
    while(not rclpy.ok()):
        # Wait for publisher to be ready
        pass
    publish_path_node.sendPath()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
