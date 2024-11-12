#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import matplotlib.cm as cm
import matplotlib.colors as colors
import numpy as np
import rclpy
import tf2_ros
import tf_transformations
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from .controller import time_parameterize_ramp_up_ramp_down
from .pid import PIDController
from .pure_pursuit import PurePursuitController
from .stanley import StanleyController

from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray
from std_msgs.msg import Float64
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

controllers = {
    "pid": PIDController,
    "pp": PurePursuitController,
    "s": StanleyController,
}

class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        controller_type, params = self.get_ros_params()
        self.controller = controllers[controller_type](**params)
        self.prev_controller_output = None

        self.setup_pub_sub()

        self.control_timer = self.create_timer(1.0 / self.rate, self.control_callback)


    def odom_callback(self, msg):
        x, y, z, w = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        _, _, yaw = tf_transformations.euler_from_quaternion((x, y, z, w))

        self.controller.current_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        ])

    def path_callback(self, msg):
        """Handle a new geometric path tracking request."""

        path_xyt = np.array(
            list(map(pose_to_array, msg.poses))
        )

        if len(path_xyt) != 0:
            self.set_path_with_speed(path_xyt, self.speed)
        else:
            print("invalid path")

    def control_callback(self):
        controller_output = self.controller.control_step()

        msg = AckermannDrive()
        if controller_output == None:
            # Send a stop command once when the controller stops running
            if self.prev_controller_output != None:
                self.cmd_publisher.publish(msg)
                self.prev_controller_output = None
        else:
            msg.speed = controller_output[0]
            msg.steering_angle = controller_output[1]
            self.cmd_publisher.publish(msg)

            error_msg = Float64()
            error_msg.data = self.controller.distance_error
            self.error_publisher.publish(error_msg)

            if len(self.controller.real_path) > 0:
                self.real_pose_publisher.publish(configs_to_pose_array(self.controller.real_path))

        self.prev_controller_output = controller_output

    def setup_pub_sub(self):
        """Initialize ROS service and publishers."""

        self.odom_subscriber = self.create_subscription(
             Odometry,
             "odom",
             self.odom_callback,
             1)

        transient_local = QoSProfile(
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # In ROS2 this should technically be set up as an action instead of a topic
        self.path_subscriber = self.create_subscription(
            PoseArray,
            "/cmd_path",
            self.path_callback,
            qos_profile = transient_local)

        self.cmd_publisher = self.create_publisher(AckermannDrive, "/drive", 10)

        self.error_publisher = self.create_publisher(Float64, "/distance_error", 1)
        self.real_pose_publisher = self.create_publisher(PoseArray, "/viz/real_path", qos_profile=transient_local)


    def reset_state(self):
        """Reset the controller's internal state (e.g., accumulators in PID control)."""
        self.controller.reset_state()
        print("reset state")
        return []

    def set_path_with_speed(self, path_xyt, speed):
        """Follow a geometric path of states with a desired speed.

        Args:
            path_xyt: np.array of states with shape L x 3
            speed (double): desired speed
        """

        path_pose_array = configs_to_pose_array(path_xyt)
        self.controller.set_path_with_speed(path_xyt, speed)
        print("path set")

    def get_ros_params(self):
        """Pull controller parameters from the ROS parameter server.

        Returns:
            controller_type (str): one of "pid", "pp", "mpc"
            controller_params (dict): controller-specific parameters
        """

        # ROS node parameters
        self.declare_parameter("rate", 50)
        self.declare_parameter("speed", 0.5)

        self.rate = self.get_parameter("rate").get_parameter_value().integer_value
        self.speed = self.get_parameter("speed").get_parameter_value().double_value

        # Base Controller Parameters
        self.declare_parameter("min_speed", 0.2)
        self.declare_parameter("distance_lookahead", 0.2)
        self.declare_parameter("finish_threshold", 0.1)
        self.declare_parameter("exceed_threshold", 0.5)

        base_params = {
            "min_speed": self.get_parameter("min_speed").get_parameter_value().double_value,
            "distance_lookahead": self.get_parameter("distance_lookahead").get_parameter_value().double_value,
            "finish_threshold": self.get_parameter("finish_threshold").get_parameter_value().double_value,
            "exceed_threshold": self.get_parameter("exceed_threshold").get_parameter_value().double_value
        }

        # Controller Type
        self.declare_parameter("type", "pid")
        controller_type = self.get_parameter("type").get_parameter_value().string_value.lower()

        # Specific Controller Params
        if controller_type == "pid":
            self.declare_parameter("pid.kp", 1.0)
            self.declare_parameter("pid.kd", 1.0)
            params = {
                "kp": self.get_parameter("pid.kp").get_parameter_value().double_value,
                "kd": self.get_parameter("pid.kd").get_parameter_value().double_value
            }
        elif controller_type == "pp":
            params = {}
        elif controller_type == "s":
            self.declare_parameter("stanley.ke", 1.0),
            self.declare_parameter("stanley.kv", 0.01)
            params = {
                "ke": self.get_parameter("stanley.ke").get_parameter_value().double_value,
                "kv": self.get_parameter("stanley.kv").get_parameter_value().double_value
            }
        else:
            print("Controller Type: ", controller_type, "is not a valid controller type, please use 'pid', 'pp', or 's'")


        merged_params = base_params.copy()
        merged_params.update(params)
        return controller_type, merged_params


def pose_to_array(pose):
    x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    _, _, yaw = tf_transformations.euler_from_quaternion((x, y, z, w))
    
    return [pose.position.x, pose.position.y, yaw]

def array_to_pose( config):
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


def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
