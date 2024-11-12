#!/usr/bin/env python3
import sys
import csv
import rclpy
from rclpy.node import Node
from rosidl_runtime_py import message_to_csv
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import Odometry
import tf_transformations


class RecordPathNode(Node):
    def __init__(self, filename):
        super().__init__("record_path_node")

        self.declare_parameter("rate", 10)
        self.rate =  self.get_parameter("rate").get_parameter_value().integer_value
        self.path = []
        self.current_pose = None
        self.filename = filename

        transient_local = QoSProfile(
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.real_pose_publisher = self.create_publisher(PoseArray, "/viz/real_path", qos_profile=transient_local)

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 1)
        self.control_timer = self.create_timer(1.0 / self.rate, self.path_callback)

    def odom_callback(self, msg):
        x, y, z, w = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        _, _, yaw = tf_transformations.euler_from_quaternion((x, y, z, w))

        self.current_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ]

    def path_callback(self):
        # Append valid new poses to the path
        # This might have issues with drift/floating point innaccuracy, could use something other than !=
        if self.current_pose != None and (len(self.path) == 0 or self.current_pose != self.path[-1]):
            self.path.append(self.current_pose)
            self.real_pose_publisher.publish(configs_to_pose_array(self.path))
           

    def save_path_to_csv(self):
        if len(self.path) == 0:
            print("\nPath is empty, no file saved")
            return

        msg = PoseArray()
        msg.header.frame_id = "map"

        for pose in self.path:
            p_msg = Pose()
            p_msg.position.x = pose[0]
            p_msg.position.y = pose[1]

            quat = tf_transformations.quaternion_from_euler(0, 0, pose[2])
            p_msg.orientation.x = quat[0]
            p_msg.orientation.y = quat[1]
            p_msg.orientation.z = quat[2]
            p_msg.orientation.w = quat[3]

            msg.poses.append(p_msg)

        # Convert the message to csv format
        csv_string = message_to_csv(msg)

        # Save the message to a CSV file
        with open(self.filename, 'w') as csvfile:
            csvfile.write(csv_string)

        print("\nPath of length ", len(msg.poses), " saved to: ", self.filename)

def array_to_pose(config):
    p = Pose()
    p.position.x = config[0]
    p.position.y = config[1]

    quat = tf_transformations.quaternion_from_euler(0, 0, config[2])
    p.orientation.x = quat[0]
    p.orientation.y = quat[1]
    p.orientation.z = quat[2]
    p.orientation.w = quat[3]

    return p

def configs_to_pose_array(path_xyt):
    """Publish path visualization messages."""

    path_as_poses = list(
        map(array_to_pose, path_xyt)
    )

    pa = PoseArray()
    pa.header.frame_id = "map"
    pa.poses = path_as_poses
    return pa


def main():
    rclpy.init()

    filename = "path.csv"
    if len(sys.argv) < 2:
        print("default filename: 'path.csv'")
    else:
        filename = sys.argv[1]

    record_path_node = RecordPathNode(filename)

    try:
        rclpy.spin(record_path_node)
    except KeyboardInterrupt:
        record_path_node.save_path_to_csv()
    else:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        record_path_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
