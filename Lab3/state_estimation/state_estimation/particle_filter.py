#!/usr/bin/env python

# packages
import rclpy
from rclpy.node import Node
import numpy as np
import range_libc
import time
from threading import Lock
import tf2_ros
from .utils import *
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


# messages
from std_msgs.msg import String, Header, Float32MultiArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap

# visualization packages
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

'''
These flags indicate several variants of the sensor model. Only one of them is used at a time.
'''
VAR_NO_EVAL_SENSOR_MODEL = 0
VAR_CALC_RANGE_MANY_EVAL_SENSOR = 1
VAR_REPEAT_ANGLES_EVAL_SENSOR = 2
VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT = 3
VAR_RADIAL_CDDT_OPTIMIZATIONS = 4

class ParticleFilter(Node):
    '''
    This class implements Monte Carlo Localization based on odometry and a laser scanner.
    '''

    def __init__(self, name):
        super().__init__(name)
        # parameters
        self.declare_parameter("angle_step", 18)
        self.declare_parameter("max_particles", 4000)
        self.declare_parameter("max_viz_particles", 4000)
        self.declare_parameter("squash_factor", 2.2)
        self.declare_parameter("max_range", 10.0)
        self.declare_parameter("theta_discretization", 112)
        self.declare_parameter("range_method", "cddt")
        self.declare_parameter("rangelib_variant", 2)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("viz", True)

        self.ANGLE_STEP        = self.get_parameter("angle_step").get_parameter_value().integer_value
        self.MAX_PARTICLES     = self.get_parameter("max_particles").get_parameter_value().integer_value
        self.MAX_VIZ_PARTICLES = self.get_parameter("max_viz_particles").get_parameter_value().integer_value
        self.INV_SQUASH_FACTOR = 1.0 / self.get_parameter("squash_factor").get_parameter_value().double_value
        self.MAX_RANGE_METERS  = self.get_parameter("max_range").get_parameter_value().double_value
        self.THETA_DISCRETIZATION = self.get_parameter("theta_discretization").get_parameter_value().integer_value
        self.WHICH_RM          = self.get_parameter("range_method").get_parameter_value().string_value.lower()
        self.RANGELIB_VAR      = self.get_parameter("rangelib_variant").get_parameter_value().integer_value
        self.PUBLISH_ODOM      = self.get_parameter("publish_odom").get_parameter_value().bool_value
        self.DO_VIZ            = self.get_parameter("viz").get_parameter_value().bool_value

        # sensor model constants
        self.declare_parameter("z_short", 0.01)
        self.declare_parameter("z_max", 0.07)
        self.declare_parameter("z_rand", 0.12)
        self.declare_parameter("z_hit", 0.75)
        self.declare_parameter("sigma_hit", 8.0)

        self.Z_SHORT   = self.get_parameter("z_short").get_parameter_value().double_value
        self.Z_MAX     = self.get_parameter("z_max").get_parameter_value().double_value
        self.Z_RAND    = self.get_parameter("z_rand").get_parameter_value().double_value
        self.Z_HIT     = self.get_parameter("z_hit").get_parameter_value().double_value
        self.SIGMA_HIT = self.get_parameter("sigma_hit").get_parameter_value().double_value

        # motion model constants
        self.declare_parameter("motion_dispersion_x", 0.05)
        self.declare_parameter("motion_dispersion_y", 0.025)
        self.declare_parameter("motion_dispersion_theta", 0.025)

        self.MOTION_DISPERSION_X = self.get_parameter("motion_dispersion_x").get_parameter_value().double_value
        self.MOTION_DISPERSION_Y = self.get_parameter("motion_dispersion_y").get_parameter_value().double_value
        self.MOTION_DISPERSION_THETA = self.get_parameter("motion_dispersion_theta").get_parameter_value().double_value
        
        # Map parameters
        self.declare_parameter("static_map", "/map_server/map")

        # various data containers used in the MCL algorithm
        self.MAX_RANGE_PX = None
        self.odometry_data = np.array([0.0,0.0,0.0])
        self.laser = None
        self.iters = 0
        self.map_info = None
        self.map_initialized = False
        self.lidar_initialized = False
        self.odom_initialized = False
        self.last_pose = None
        self.laser_angles = None
        self.downsampled_angles = None
        self.range_method = None
        self.last_time = None
        self.last_stamp = None
        self.first_sensor_update = True
        self.state_lock = Lock()

        # cache this to avoid memory allocation in motion model
        self.local_deltas = np.zeros((self.MAX_PARTICLES, 3))

        # cache this for the sensor model computation
        self.queries = None
        self.ranges = None
        self.tiled_angles = None
        self.sensor_model_table = None

        # particle poses and weights
        self.inferred_pose = None
        self.particle_indices = np.arange(self.MAX_PARTICLES)
        self.particles = np.zeros((self.MAX_PARTICLES, 3))
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)

        # initialize the state
        self.timer = Timer(10)
        self.get_omap()
        self.precompute_sensor_model()
        self.initialize_global()

        # these topics are for visualization
        self.pose_pub      = self.create_publisher(PoseStamped, "/pf/viz/inferred_pose", 1)
        self.particle_pub  = self.create_publisher(PoseArray, "/pf/viz/particles", 1)
        self.pub_fake_scan = self.create_publisher(LaserScan, "/pf/viz/fake_scan", 1)
        self.rect_pub      = self.create_publisher(PolygonStamped, "/pf/viz/poly1", 1)

        if self.PUBLISH_ODOM:
            self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )


        # these topics are for coordinate space things
        self.pub_tf = tf2_ros.TransformBroadcaster(self)

        # these topics are to receive data from the car
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.lidarCB, qos_profile=qos_profile)
        self.odom_sub  = self.create_subscription(Odometry, "/odom", self.odomCB, 1)
        self.pose_sub  = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.clicked_pose, 1)
        self.click_sub = self.create_subscription(PointStamped, "/clicked_point", self.clicked_pose, 1)

    def get_omap(self):
        '''
        Fetch the occupancy grid map from the map_server instance, and initialize the correct
        RangeLibc method. Also stores a matrix which indicates the permissible region of the map
        '''
        map_service_name = self.get_parameter("static_map").get_parameter_value().string_value
        map_client = self.create_client(GetMap, map_service_name)
        map_request = GetMap.Request()
        print("waiting for map service: ", map_service_name)

        while not map_client.wait_for_service():
            continue

        print("getting map")

        map_response = map_client.call_async(map_request)
        rclpy.spin_until_future_complete(self, map_response)
        map_msg = map_response.result().map  

        self.map_info = map_msg.info
        oMap = range_libc.PyOMap(map_msg)
        self.MAX_RANGE_PX = int(self.MAX_RANGE_METERS / self.map_info.resolution)

        # initialize range method
        print("Initializing range method:", self.WHICH_RM)
        if self.WHICH_RM == "bl":
            self.range_method = range_libc.PyBresenhamsLine(oMap, self.MAX_RANGE_PX)
        elif "cddt" in self.WHICH_RM:
            self.range_method = range_libc.PyCDDTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
            if self.WHICH_RM == "pcddt":
                print("Pruning...")
                self.range_method.prune()
        elif self.WHICH_RM == "rm":
            self.range_method = range_libc.PyRayMarching(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RM == "rmgpu":
            self.range_method = range_libc.PyRayMarchingGPU(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RM == "glt":
            self.range_method = range_libc.PyGiantLUTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
        print("Done loading map")

         # 0: permissible, -1: unmapped, 100: blocked
        array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

        # 0: not permissible, 1: permissible
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255==0] = 1
        self.map_initialized = True

    def publish_tf(self, pose, stamp=None):
        """ Publish a tf for the car. This tells ROS where the car is with respect to the map. """
        if stamp == None:
            stamp = self.get_clock().now()

        map_laser_rotation = np.array( tf_transformations.quaternion_from_euler(0, 0, pose[2]) )

        # Get map -> laser transform.
        map_laser_pos = np.array( (pose[0],pose[1],0) )
        # Apply laser -> base_link transform to map -> laser transform
        # This gives a map -> base_link transform
        laser_base_link_offset = (0.0125, 0, 0)
        map_laser_pos -= np.dot(tf_transformations.quaternion_matrix(tf_transformations.unit_vector(map_laser_rotation))[:3,:3], laser_base_link_offset).T

        # Publish transform
        base_laser_t = TransformStamped()
        base_laser_t.header.stamp = self.get_clock().now().to_msg()
        base_laser_t.header.frame_id = "map"
        base_laser_t.child_frame_id = "base_link"

        base_laser_t.transform.translation.x = map_laser_pos[0]
        base_laser_t.transform.translation.y = map_laser_pos[1]
        base_laser_t.transform.translation.z = map_laser_pos[2]

        base_laser_t.transform.rotation.x = map_laser_rotation[0]
        base_laser_t.transform.rotation.y = map_laser_rotation[1]
        base_laser_t.transform.rotation.z = map_laser_rotation[2]
        base_laser_t.transform.rotation.w = map_laser_rotation[3]
        
        self.pub_tf.sendTransform(base_laser_t)

        # also publish odometry to facilitate getting the localization pose
        if self.PUBLISH_ODOM:
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "/map"
            odom.pose.pose.position.x = pose[0]
            odom.pose.pose.position.y = pose[1]
            odom.pose.pose.orientation = angle_to_quaternion(pose[2])
            self.odom_pub.publish(odom)

    def visualize(self):
        '''
        Publish various visualization messages.
        '''
        if not self.DO_VIZ:
            return

        if self.count_subscribers("/pf/viz/inferred_pose") > 0 and isinstance(self.inferred_pose, np.ndarray):
            # Publish the inferred pose for visualization
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = "map"
            ps.pose.position.x = self.inferred_pose[0]
            ps.pose.position.y = self.inferred_pose[1]
            ps.pose.orientation = angle_to_quaternion(self.inferred_pose[2])
            self.pose_pub.publish(ps)

        # publish a downsampled version of the particle distribution to avoid a lot of latency
        if self.count_subscribers("/pf/viz/particles") > 0:
            if self.MAX_PARTICLES > self.MAX_VIZ_PARTICLES:
                # randomly downsample particles
                proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES, p=self.weights)
                # proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES)
                self.publish_particles(self.particles[proposal_indices,:])
            else:
                self.publish_particles(self.particles)

        if self.count_subscribers("/pf/viz/fake_scan") > 0 and isinstance(self.ranges, np.ndarray):
            # generate the scan from the point of view of the inferred position for visualization
            self.viz_queries[:,0] = self.inferred_pose[0]
            self.viz_queries[:,1] = self.inferred_pose[1]
            self.viz_queries[:,2] = self.downsampled_angles + self.inferred_pose[2]
            self.range_method.calc_range_many(self.viz_queries, self.viz_ranges)
            self.publish_scan(self.downsampled_angles, self.viz_ranges)

    def publish_particles(self, particles):
        # publish the given particles as a PoseArray object
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = "map"
        pa.poses = list(particles_to_poses(particles))
        self.particle_pub.publish(pa)

    def publish_scan(self, angles, ranges):
        # publish the given angels and ranges as a laser scan message
        ls = LaserScan()
        ls.header.stamp = self.get_clock().now().to_msg()
        ls.header.frame_id = "laser"
        ls.angle_min = float(np.min(angles))
        ls.angle_max = float(np.max(angles))
        ls.angle_increment = float(np.abs(angles[0] - angles[1]))
        ls.range_min = 0.0
        ls.range_max = float(np.max(ranges))
        ls.ranges = list(map(float, ranges))
        self.pub_fake_scan.publish(ls)

    def lidarCB(self, msg):
        '''
        Initializes reused buffers, and stores the relevant laser scanner data for later use.
        '''
        if not isinstance(self.laser_angles, np.ndarray):
            print("...Received first LiDAR message")
            self.laser_angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            self.downsampled_angles = np.copy(self.laser_angles[0::self.ANGLE_STEP]).astype(np.float32)
            self.viz_queries = np.zeros((self.downsampled_angles.shape[0],3), dtype=np.float32)
            self.viz_ranges = np.zeros(self.downsampled_angles.shape[0], dtype=np.float32)

        # store the necessary scanner information for later processing
        self.downsampled_ranges = np.array(msg.ranges[::self.ANGLE_STEP])
        self.lidar_initialized = True

        # This is the slower topic so update every time we recieve a message
        self.update()

    def odomCB(self, msg):
        '''
        Store deltas between consecutive odometry messages in the coordinate space of the car.

        Odometry data is accumulated via dead reckoning, so it is very inaccurate on its own.
        '''
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y])

        orientation = quaternion_to_angle(msg.pose.pose.orientation)
        pose = np.array([position[0], position[1], orientation])

        if isinstance(self.last_pose, np.ndarray):
            # changes in x,y,theta in local coordinate system of the car
            rot = rotation_matrix(-self.last_pose[2])
            delta = np.array([position - self.last_pose[0:2]]).transpose()
            local_delta = (rot*delta).transpose()
            
            self.odometry_data = np.array([local_delta[0,0], local_delta[0,1], orientation - self.last_pose[2]])
            self.last_pose = pose
            self.last_stamp = msg.header.stamp
            self.odom_initialized = True
        else:
            print("...Received first Odometry message")
            self.last_pose = pose

    def clicked_pose(self, msg):
        '''
        Receive pose messages from RViz and initialize the particle distribution in response.
        '''
        if isinstance(msg, PointStamped):
            self.initialize_global()
        elif isinstance(msg, PoseWithCovarianceStamped):
            self.initialize_particles_pose(msg.pose.pose)

    def initialize_particles_pose(self, pose):
        '''
        Initialize particles in the general region of the provided pose.
        '''
        print("SETTING POSE")
        print(pose)
        self.state_lock.acquire()
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)
        self.particles[:,0] = pose.position.x + np.random.normal(loc=0.0,scale=0.5,size=self.MAX_PARTICLES)
        self.particles[:,1] = pose.position.y + np.random.normal(loc=0.0,scale=0.5,size=self.MAX_PARTICLES)
        self.particles[:,2] = quaternion_to_angle(pose.orientation) + np.random.normal(loc=0.0,scale=0.4,size=self.MAX_PARTICLES)

        self.state_lock.release()

    def initialize_global(self):
        '''
        Spread the particle distribution over the permissible region of the state space.
        '''
        print("GLOBAL INITIALIZATION")
        # randomize over grid coordinate space
        self.state_lock.acquire()
        permissible_x, permissible_y = np.where(self.permissible_region == 1)
        indices = np.random.randint(0, len(permissible_x), size=self.MAX_PARTICLES)

        permissible_states = np.zeros((self.MAX_PARTICLES,3))
        permissible_states[:,0] = permissible_y[indices]
        permissible_states[:,1] = permissible_x[indices]
        permissible_states[:,2] = np.random.random(self.MAX_PARTICLES) * np.pi * 2.0

        map_to_world(permissible_states, self.map_info)
        self.particles = permissible_states
        self.weights[:] = 1.0 / self.MAX_PARTICLES
        self.state_lock.release()

    def precompute_sensor_model(self):
        '''
        Generate and store a table which represents the sensor model. For each discrete computed
        range value, this provides the probability of measuring any (discrete) range.

        This table is indexed by the sensor model at runtime by discretizing the measurements
        and computed ranges from RangeLibc.
        '''
        print("Precomputing sensor model")
        # sensor model constants
        z_short = self.Z_SHORT
        z_max   = self.Z_MAX
        z_rand  = self.Z_RAND
        z_hit   = self.Z_HIT
        sigma_hit = self.SIGMA_HIT
        
        table_width = int(self.MAX_RANGE_PX) + 1
        self.sensor_model_table = np.zeros((table_width,table_width))

        t = time.time()
        # d is the computed range from RangeLibc
        for d in range(table_width):
            norm = 0.0
            sum_unkown = 0.0
            # r is the observed range from the lidar unit
            for r in range(table_width):
                prob = 0.0
                z = float(r-d)
                # reflects from the intended object
                prob += z_hit * np.exp(-(z*z)/(2.0*sigma_hit*sigma_hit)) / (sigma_hit * np.sqrt(2.0*np.pi))

                # observed range is less than the predicted range - short reading
                if r < d:
                    prob += 2.0 * z_short * float(d - r) / float(d)

                # erroneous max range measurement
                if int(r) == int(self.MAX_RANGE_PX):
                    prob += z_max

                # random measurement
                if r < int(self.MAX_RANGE_PX):
                    prob += z_rand * 1.0/float(self.MAX_RANGE_PX)

                norm += prob
                self.sensor_model_table[int(r),int(d)] = prob

            # normalize
            self.sensor_model_table[:,int(d)] /= norm

        # upload the sensor model to RangeLib for ultra fast resolution
        if self.RANGELIB_VAR > 0:
            self.range_method.set_sensor_model(self.sensor_model_table)

        # code to generate various visualizations of the sensor model
        if False:
            # visualize the sensor model
            fig, ax = plt.subplots(subplot_kw={"projection" : "3d"})

            # Make data.
            X = np.arange(0, table_width, 1.0)
            Y = np.arange(0, table_width, 1.0)
            X, Y = np.meshgrid(X, Y)

            # Plot the surface.
            surf = ax.plot_surface(X, Y, self.sensor_model_table, cmap="bone", rstride=2, cstride=2,
                                   linewidth=0, antialiased=True)

            ax.text2D(0.05, 0.95, "Precomputed Sensor Model", transform=ax.transAxes)
            ax.set_xlabel('Ground truth distance (in px)')
            ax.set_ylabel('Measured Distance (in px)')
            ax.set_zlabel('P(Measured Distance | Ground Truth)')

            plt.show()
        elif False:
            plt.imshow(self.sensor_model_table * 255, cmap="gray")
            plt.show()
        elif False:
            plt.plot(self.sensor_model_table[:,140])
            plt.plot([139,139],[0.0,0.08], label="test")
            plt.ylim(0.0, 0.08)
            plt.xlabel("Measured Distance (in px)")
            plt.ylabel("P(Measured Distance | Ground Truth Distance = 140px)")
            plt.show()

    def motion_model(self, proposal_dist, action):
        '''
        The motion model applies the odometry to the particle distribution. Since there the odometry
        data is inaccurate, the motion model mixes in gaussian noise to spread out the distribution.

        Vectorized motion model. Computing the motion model over all particles is thousands of times
        faster than doing it for each particle individually due to vectorization and reduction in
        function call overhead
        
        TODO this could be better, but it works for now
            - fixed random noise is not very realistic
            - ackermann model provides bad estimates at high speed
        '''
        # rotate the action into the coordinate space of each particle
        # t1 = time.time()
        cosines = np.cos(proposal_dist[:,2])
        sines = np.sin(proposal_dist[:,2])

        self.local_deltas[:,0] = cosines*action[0] - sines*action[1]
        self.local_deltas[:,1] = sines*action[0] + cosines*action[1]
        self.local_deltas[:,2] = action[2]

        proposal_dist[:,:] += self.local_deltas
        proposal_dist[:,0] += np.random.normal(loc=0.0,scale=self.MOTION_DISPERSION_X,size=self.MAX_PARTICLES)
        proposal_dist[:,1] += np.random.normal(loc=0.0,scale=self.MOTION_DISPERSION_Y,size=self.MAX_PARTICLES)
        proposal_dist[:,2] += np.random.normal(loc=0.0,scale=self.MOTION_DISPERSION_THETA,size=self.MAX_PARTICLES)

    def sensor_model(self, proposal_dist, obs, weights):
        '''
        This function computes a probablistic weight for each particle in the proposal distribution.
        These weights represent how probable each proposed (x,y,theta) pose is given the measured
        ranges from the lidar scanner.

        There are 4 different variants using various features of RangeLibc for demonstration purposes.
        - VAR_REPEAT_ANGLES_EVAL_SENSOR is the most stable, and is very fast.
        - VAR_NO_EVAL_SENSOR_MODEL directly indexes the precomputed sensor model. This is slow
                                   but it demonstrates what self.range_method.eval_sensor_model does
        - VAR_RADIAL_CDDT_OPTIMIZATIONS is only compatible with CDDT or PCDDT, it implments the radial
                                        optimizations to CDDT which simultaneously performs ray casting
                                        in two directions, reducing the amount of work by roughly a third
        '''
        
        num_rays = self.downsampled_angles.shape[0]
        # only allocate buffers once to avoid slowness
        if self.first_sensor_update:
            if self.RANGELIB_VAR <= 1:
                self.queries = np.zeros((num_rays*self.MAX_PARTICLES,3), dtype=np.float32)
            else:
                self.queries = np.zeros((self.MAX_PARTICLES,3), dtype=np.float32)

            self.ranges = np.zeros(num_rays*self.MAX_PARTICLES, dtype=np.float32)
            self.tiled_angles = np.tile(self.downsampled_angles, self.MAX_PARTICLES)
            self.first_sensor_update = False

        if self.RANGELIB_VAR == VAR_RADIAL_CDDT_OPTIMIZATIONS:
            if "cddt" in self.WHICH_RM:
                self.queries[:,:] = proposal_dist[:,:]
                self.range_method.calc_range_many_radial_optimized(num_rays, self.downsampled_angles[0], self.downsampled_angles[-1], self.queries, self.ranges)

                # evaluate the sensor model
                self.range_method.eval_sensor_model(obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
                # apply the squash factor
                self.weights = np.power(self.weights, self.INV_SQUASH_FACTOR)
            else:
                print("Cannot use radial optimizations with non-CDDT based methods, use rangelib_variant 2")
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT:
            self.queries[:,:] = proposal_dist[:,:]
            self.range_method.calc_range_repeat_angles_eval_sensor_model(self.queries, self.downsampled_angles, obs, self.weights)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
        elif self.RANGELIB_VAR == VAR_REPEAT_ANGLES_EVAL_SENSOR:
            # this version demonstrates what this would look like with coordinate space conversion pushed to rangelib
            self.queries[:,:] = proposal_dist[:,:]
            self.range_method.calc_range_repeat_angles(self.queries, self.downsampled_angles, self.ranges)
            # evaluate the sensor model on the GPU
            self.range_method.eval_sensor_model(obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
        elif self.RANGELIB_VAR == VAR_CALC_RANGE_MANY_EVAL_SENSOR:
            # this version demonstrates what this would look like with coordinate space conversion pushed to rangelib
            # this part is inefficient since it requires a lot of effort to construct this redundant array
            self.queries[:,0] = np.repeat(proposal_dist[:,0], num_rays)
            self.queries[:,1] = np.repeat(proposal_dist[:,1], num_rays)
            self.queries[:,2] = np.repeat(proposal_dist[:,2], num_rays)
            self.queries[:,2] += self.tiled_angles

            self.range_method.calc_range_many(self.queries, self.ranges)

            # evaluate the sensor model on the GPU
            self.range_method.eval_sensor_model(obs, self.ranges, self.weights, num_rays, self.MAX_PARTICLES)
            np.power(self.weights, self.INV_SQUASH_FACTOR, self.weights)
        elif self.RANGELIB_VAR == VAR_NO_EVAL_SENSOR_MODEL:
            # this version directly uses the sensor model in Python, at a significant computational cost
            self.queries[:,0] = np.repeat(proposal_dist[:,0], num_rays)
            self.queries[:,1] = np.repeat(proposal_dist[:,1], num_rays)
            self.queries[:,2] = np.repeat(proposal_dist[:,2], num_rays)
            self.queries[:,2] += self.tiled_angles

            # compute the ranges for all the particles in a single functon call
            self.range_method.calc_range_many(self.queries, self.ranges)

            # resolve the sensor model by discretizing and indexing into the precomputed table
            obs /= float(self.map_info.resolution)
            ranges = self.ranges / float(self.map_info.resolution)
            obs[obs > self.MAX_RANGE_PX] = self.MAX_RANGE_PX
            ranges[ranges > self.MAX_RANGE_PX] = self.MAX_RANGE_PX

            intobs = np.rint(obs).astype(np.uint16)
            intrng = np.rint(ranges).astype(np.uint16)

            # compute the weight for each particle
            for i in range(self.MAX_PARTICLES):
                weight = np.product(self.sensor_model_table[intobs,intrng[i*num_rays:(i+1)*num_rays]])
                weight = np.power(weight, self.INV_SQUASH_FACTOR)
                weights[i] = weight
        else:
            print("PLEASE SET rangelib_variant PARAM to 0-4")

    def MCL(self, a, o):
        '''
        Performs one step of Monte Carlo Localization.
            1. resample particle distribution to form the proposal distribution
            2. apply the motion model
            3. apply the sensor model
            4. normalize particle weights

        This is in the critical path of code execution, so it is optimized for speed.
        '''

        # draw the proposal distribution from the old particles
        proposal_indices = np.random.choice(self.particle_indices, self.MAX_PARTICLES, p=self.weights)
        proposal_distribution = self.particles[proposal_indices,:]

        # compute the motion model to update the proposal distribution
        self.motion_model(proposal_distribution, a)

        # compute the sensor model
        self.sensor_model(proposal_distribution, o, self.weights)

        # normalize importance weights
        self.weights /= np.sum(self.weights)

        # save the particles
        self.particles = proposal_distribution
    
    def expected_pose(self):
        # returns the expected value of the pose given the particle distribution
        return np.dot(self.particles.transpose(), self.weights)

    def update(self):
        '''
        Apply the MCL function to update particle filter state. 

        Ensures the state is correctly initialized, and acquires the state lock before proceeding.
        '''
        if self.lidar_initialized and self.odom_initialized and self.map_initialized:
            if self.state_lock.locked():
                print("Concurrency error avoided")
            else:
                self.state_lock.acquire()
                self.timer.tick()
                self.iters += 1

                t1 = time.time()
                observation = np.copy(self.downsampled_ranges).astype(np.float32)
                action = np.copy(self.odometry_data)
                self.odometry_data = np.zeros(3)

                # run the MCL update algorithm
                self.MCL(action, observation)

                # compute the expected value of the robot pose
                self.inferred_pose = self.expected_pose()
                self.state_lock.release()
                t2 = time.time()

                # publish transformation frame based on inferred pose
                self.publish_tf(self.inferred_pose, self.last_stamp)

                # this is for tracking particle filter speed
                ips = 1.0 / (t2 - t1)
                if self.iters % 10 == 0:
                    print("iters per sec:", int(self.timer.fps()))

                self.visualize()

def main(args=None):
    rclpy.init(args=args)
    particle_filter = ParticleFilter("particle_filter")

    rclpy.spin(particle_filter)

    rclpy.shutdown()
    particle_filter.destroy_node()


if __name__=="__main__":
    main()
