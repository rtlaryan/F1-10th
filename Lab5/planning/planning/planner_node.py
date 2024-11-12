'''
NOTE: This is not a very good way to design this lab but doing it properly
would require modifications to Lab 4 to support local planning
and constant path updates. Hopefully this can be fixed in future semesters
'''

import numpy as np
import math

import rclpy
from rclpy.node import Node
import tf_transformations
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, OccupancyGrid
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker

from planning.rrt import RRT

from control import controller, pid, pure_pursuit, stanley
from control.pid import PIDController
from control.pure_pursuit import PurePursuitController
from control.stanley import StanleyController
from control.control_node import pose_to_array, array_to_pose, configs_to_pose_array


controllers = {
    "pid": PIDController,
    "pp": PurePursuitController,
    "s": StanleyController,
}

class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")

        best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
           "/scan",
            self.scan_callback,
            qos_profile=best_effort)

        # Publishers
        self.cmd_publisher = self.create_publisher(AckermannDrive, "/drive", 10)

        self.rrt_publisher = self.create_publisher(Marker, "/viz/rrt_tree", qos_profile=best_effort)
        self.grid_publisher = self.create_publisher(OccupancyGrid, "/viz/occupancy_grid", qos_profile=best_effort)
        self.planned_path_publisher = self.create_publisher(PoseArray, "/viz/planned_path", qos_profile=best_effort)


        # Controller Params
        self.declare_parameter("controller.enabled", False)
        self.declare_parameter("controller.rate", 20)
        self.declare_parameter("controller.speed", 0.5)

        self.controller_enabled = self.get_parameter("controller.enabled").get_parameter_value().bool_value
        self.rate = self.get_parameter("controller.rate").get_parameter_value().integer_value
        self.speed = self.get_parameter("controller.speed").get_parameter_value().double_value


        # RRT Occupancy Grid Params
        self.declare_parameter("rrt.max_grid_distance", 2.0)
        self.declare_parameter("rrt.grid_resolution", 0.05)
        self.declare_parameter("rrt.inflation_radius", 3)

        self.max_grid_distance = self.get_parameter("rrt.max_grid_distance").get_parameter_value().double_value
        self.grid_resolution = self.get_parameter("rrt.grid_resolution").get_parameter_value().double_value
        self.inflation_radius = self.get_parameter("rrt.inflation_radius").get_parameter_value().integer_value


        # RRT planning params
        self.declare_parameter("rrt.max_rrt_iters", 300)
        self.declare_parameter("rrt.lookahead_distance", 1.6)
        self.declare_parameter("rrt.max_expansion_distance", 0.7)
        self.declare_parameter("rrt.search_radius", 0.5)
        self.declare_parameter("rrt.goal_tolerance", 0.1)

        self.max_rrt_iters = self.get_parameter("rrt.max_rrt_iters").get_parameter_value().integer_value
        self.rrt_lookahead_distance = self.get_parameter("rrt.lookahead_distance").get_parameter_value().double_value
        self.max_rrt_expansion_distance = self.get_parameter("rrt.max_expansion_distance").get_parameter_value().double_value
        self.search_radius = self.get_parameter("rrt.search_radius").get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter("rrt.goal_tolerance").get_parameter_value().double_value


        # Class variables

        # Controller
        if self.controller_enabled:
            self.control_timer = self.create_timer(1.0 / self.rate, self.control_callback)

        controller_type, params = self.get_controller_params()
        self.controller = controllers[controller_type](**params)
        self.controller.current_pose = np.array([0.0, 0.0, 0.0])


        # RRT
        self.rrt = RRT(self.max_grid_distance, self.grid_resolution, self.inflation_radius, 
                    self.max_rrt_iters, self.max_rrt_expansion_distance, self.search_radius, self.goal_tolerance)


    def control_callback(self):
        controller_output = self.controller.control_step()

        msg = AckermannDrive()

        if controller_output != None:
            msg.speed = controller_output[0]
            msg.steering_angle = controller_output[1]

        self.cmd_publisher.publish(msg)


    def scan_callback(self, msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """

        self.rrt.update_occupancy_grid(msg)
        self.visualize_grid()

        target_point =  []
        target_angles = [0, np.pi/8.0, -np.pi/8.0, np.pi/4.0, -np.pi/4.0, np.pi/3.0, -np.pi/3.0, 3.0*np.pi/8.0, -3.0*np.pi/8.0]
        node_path = []

        # Find a valid target and path
        for i in range(len(target_angles)):
            target_point =  [self.rrt_lookahead_distance * np.cos(target_angles[i]), np.sin(target_angles[i])]
            
            # Check if target is in obstacle
            if not self.rrt.check_point_collision(target_point):
                node_path = self.rrt.plan_path(target_point)

                if len(node_path) != 0:
                    # Found a valid path
                    break

        self.visualize_tree(target_point)

        if len(node_path) != 0:
            path = np.zeros((len(node_path) + 1, 4))
            prev = np.zeros(2)
            heading = 0

            # Convert path to the controller format
            for i in reversed(range(0, len(node_path))):
                x = node_path[i].x
                y = node_path[i].y
                vec = np.array((x, y)) - prev
                heading = np.arctan2(vec[1], vec[0])

                # leave room for the final point on the path
                path[len(node_path)-1 - i, :] = np.array((prev[0], prev[1], heading, self.speed))
                prev[0] = x
                prev[1] = y

            # Append the final element and reuse the previous heading
            path[-1, :] = np.array((prev[0], prev[1], heading, self.speed))

        else:
            # Slowly try to get unstuck if we don't have a path
            path = np.array([[0.1, 0.0, 0.0, 0.1]])

        self.visualize_path(path)
        self.controller.path = path


    def visualize_tree(self, target_point):

        if self.count_subscribers("/viz/rrt_tree") > 0:

            # Tree
            tree = Marker()
            tree.header.stamp = self.get_clock().now().to_msg()
            tree.header.frame_id = "laser"
            tree.ns = "rrt_tree"
            tree.action = Marker.ADD
            tree.pose.orientation.w = 1.0
            tree.type = Marker.LINE_LIST

            tree.scale.x = 0.01

            tree.color.r = 1.0
            tree.color.g = 0.0
            tree.color.b = 0.0
            tree.color.a = 1.0

            for node in self.rrt.tree:
                if not node.is_root:
                    point = Point()
                    point.x = float(node.x)
                    point.y = float(node.y)

                    parent = Point()
                    parent.x = float(node.parent.x)
                    parent.y = float(node.parent.y)

                    tree.points.append(point)
                    tree.points.append(parent)

            # Target Point
            target = Marker()
            target.header.stamp = self.get_clock().now().to_msg()
            target.header.frame_id = "laser"
            target.ns = "rrt_target"
            target.action = Marker.ADD
            target.pose.orientation.w = 1.0
            target.type = Marker.POINTS

            target.scale.x = 0.1
            target.scale.y = 0.1

            target.color.r = 0.0
            target.color.g = 1.0
            target.color.b = 0.0
            target.color.a = 1.0

            point = Point()
            point.x = float(target_point[0])
            point.y = float(target_point[1])
            target.points = [point]

            self.rrt_publisher.publish(tree)
            self.rrt_publisher.publish(target)


    def visualize_grid(self):
        if self.count_subscribers("/viz/occupancy_grid") > 0:
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "laser"

            msg.info.width = self.rrt.grid_length
            msg.info.height = self.rrt.grid_length
            msg.info.resolution = self.rrt.grid_resolution
            msg.info.origin.position.y = -self.rrt.max_grid_distance / 2.0
            msg.info.origin.orientation.w = 1.0

            msg.data = (100 * self.rrt.grid).flatten(order='F').tolist()

            self.grid_publisher.publish(msg)

    def visualize_path(self, path):
        if self.count_subscribers("/viz/planned_path") > 0:
            msg = configs_to_pose_array(path)
            msg.header.frame_id = "laser"
            self.planned_path_publisher.publish(msg)

    def get_controller_params(self):

        # Base Controller Parameters
        self.declare_parameter("controller.min_speed", 0.0)
        self.declare_parameter("controller.distance_lookahead", 0.2)
        self.declare_parameter("controller.finish_threshold", 0.1)
        self.declare_parameter("controller.exceed_threshold", 0.5)

        base_params = {
            "min_speed": self.get_parameter("controller.min_speed").get_parameter_value().double_value,
            "distance_lookahead": self.get_parameter("controller.distance_lookahead").get_parameter_value().double_value,
            "finish_threshold": self.get_parameter("controller.finish_threshold").get_parameter_value().double_value,
            "exceed_threshold": self.get_parameter("controller.exceed_threshold").get_parameter_value().double_value
        }

        # Controller Type
        self.declare_parameter("controller.type", "pid")
        controller_type = self.get_parameter("controller.type").get_parameter_value().string_value.lower()

        # Specific Controller Params
        if controller_type == "pid":
            self.declare_parameter("controller.pid.kp", 1.0)
            self.declare_parameter("controller.pid.kd", 1.0)
            params = {
                "kp": self.get_parameter("controller.pid.kp").get_parameter_value().double_value,
                "kd": self.get_parameter("controller.pid.kd").get_parameter_value().double_value
            }
            params = {}
        elif controller_type == "s":
            self.declare_parameter("controller.stanley.ke", 1.0),
            self.declare_parameter("controller.stanley.kv", 0.01)
            params = {
                "ke": self.get_parameter("controller.stanley.ke").get_parameter_value().double_value,
                "kv": self.get_parameter("controller.stanley.kv").get_parameter_value().double_value
            }
        else:
            print("Controller Type: ", controller_type, "is not a valid controller type, please use 'pid', 'pp', or 's'")


        merged_params = base_params.copy()
        merged_params.update(params)

        return controller_type, merged_params


def main(args=None):
    rclpy.init(args=args)
    planner_node = PlannerNode()
    rclpy.spin(planner_node)

    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
