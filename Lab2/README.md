# Lab 2: Reactive Autonomy

## I. Objectives

- Install the new AMRDocker container on the car
- Automatic Braking
- Follow the Gap

## II. Overview
It is recommended to write and test as much of the code as possible using the simulator before testing it on the real car both to save time and to catch any bugs before potentially damaging real hardware.

### Useful Commands
#### ROS
* Run a single ROS executable: `ros2 run <name_of_package> <name_of_executable>`
* Launch multiple ROS nodes as specified in a launch file: `ros2 launch <name_of_package> <name_of_launch_file>`
* See a list of running nodes: `ros2 node list`
* See a list of active topics: `ros2 topic list`
* Print all messages published to a topic: `ros2 topic echo <name_of_topic>`
* See the definition of a message type: `ros2 interface show <msg_name>`

#### Simulator
Docker based f1tenth simulator (NVIDIA rocker):
* Start the docker container: `rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros --volume ../labs:/sim_ws/src/labs -- f1tenth_gym_ros`
* Get a terminal session in the container: `docker exec -it f1tenth_gym_ros bash`

Docker based f1tenth simulator (Without gpu):
* Start the docker container: `docker-compose up`
* Get a terminal session in the container: `docker exec -it f1tenth_gym_ros-sim-1 bash`

Running the simulator:
1. Make a folder in next to the f1tenth_gym_ros folder called `labs` and put your ROS packages in it
2. Start the docker container as explained above
3. Cd to the `/sim_ws` folder: `cd /sim_ws`
4. Build the ROS packages: `colcon build`
5. Source ROS: `source /opt/ros/humble/setup.bash`
6. Source the workspace: `source install/setup.bash`
7. Run `ros2 launch f1tenth_gym_ros gym_bridge_launch.py`

Keyboard control in the simulator:
1. Follow the instructions above to start the simulator
2. Open a new terminal in the docker container
3. Run `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

#### On the car
Running code on the car:
1. Put your ROS packages in the `~/amr_ws/src` folder
2. Start a terminal session in the AMRDocker container: `./amr_session.sh`
3. Cd to the `/amr_ws` folder: `cd ~/amr_ws`
4. Build the ROS packages: `colcon build`
5. Source the workspace: `source install/setup.bash`
6. Run `ros2 launch amr_bringup amr_car_launch.py`
7. Open and prepare a new terminal, then launch/run whatever other nodes you choose

### Install the AMRDocker container
1. Go to the [AMRDocker Github](https://github.gatech.edu/ECE4804-AMR/AMRDocker) and follow the instructions there to install the AMRDocker container on the car.
2. Make a folder in the home directory of the jetson (not in a docker container) called `amr_ws`: `mkdir ~/amr_ws`. This workspace should be used for all of the code/labs for the class. The folder will be automatically mounted as an external volume in the docker container when you run `./amr_session.sh` so that you won't lose any of your files if the docker container gets reset.
3. Make a `src` folder in the `amr_ws`: `mkdir ~/amr_ws/src`

### Automatic Braking
The skeleton code for the automatic braking node can be found in the `reactive_autonomy/automatic_braking.py` file. A useful example of how to set up ROS2 publishers and subscribers in python can be found here: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html.

#### Useful Message Types
* [Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html): This message type contains odometry information from the IMU and wheel encoders. The `twist` field will contain the estimated linear and angular velocities of the vehicle. NOTE: the forward velocity of the vehicle can be found in `twist.twist.linear.x`

* [LaserScan](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html): This message type contains information from the lidar. The `ranges` array contains the distances recorded by each lidar beam in order of the angle. The `angle_min` and `angle_max` fields specify the angles of the first and last beam that the lidar outputs while the `angle_increment` field specifies the difference in angle between consecutive beams.

* [AckermannDrive](https://docs.ros.org/en/noetic/api/ackermann_msgs/html/msg/AckermannDrive.html): This message is used to command an Ackermann style vehicle. The `speed` field sets the forward veocity while the `steering_angle` field sets the steering angle of the wheels (positive is to the left).

#### Instantaneous Time to Collision
Time to Collision (TTC) is the time it would take for the car to collide with an obstacle if it maintained its current heading and velocity. We approximate the time to collision using Instantaneous Time to Collision (iTTC), which is the ratio of instantaneous range to range rate calculated from current range measurements and velocity measurements of the vehicle.

As discussed in the lecture, we can calculate the iTTC as:

$$ iTTC=\frac{r}{\lbrace- \dot{r}\rbrace_{+}} $$

where $r$ is the instantaneous range measurements, $\dot{r}$ is the current range rate for that measurement, and the operator $\lbrace \rbrace_{+}$ is defined as $\lbrace x\rbrace_{+} = \text{max}( x, 0 )$.



The instantaneous range $r$ to an obstacle is easily obtained by using the current measurements from the `LaserScan` message. Since the LiDAR effectively measures the distance from the sensor to some obstacle.
The range rate $\dot{r}$ is the expected rate of change along each scan beam. A positive range rate means the range measurement is expanding, and a negative one means the range measurement is shrinking.
Thus, it can be calculated in two different ways:

First, it can be calculated by mapping the vehicle's current longitudinal velocity from the `Odometry` message onto each scan beam's angle by using $v_x \cos{\theta_{i}}$. Be careful with assigning the range rate a positive or a negative value. The range rate could then be interpreted as how much the range measurement will change if the vehicle keeps the current velocity and the obstacle remains stationary.

Alternatively, you can take the difference between the previous range measurement and the current one then divide it by how much time has passed in between (timestamps are available in message headers) to calculate the range rate that way.

Note the negation in the calculation. This is to correctly interpret whether the range measurement should be decreasing or increasing. For a vehicle travelling forward towards an obstacle, the corresponding range rate for the beam directly in front of the vehicle should be negative since the range measurement should be shrinking. Vice versa, the range rate corresponding to the vehicle travelling away from an obstacle should be positive since the range measurement should be increasing. The operator is in place so the iTTC calculation will be meaningful. When the range rate is positive, the operator will make sure iTTC for that angle goes to infinity.



After your calculations, you should end up with an array of iTTCs that correspond to each angle. When a time to collision drops below a certain threshold, it means a collision is imminent.

#### Automatically braking using iTTC
For the first part of this lab, you will make an automatic braking node that should halt the car before it collides with obstacles. To do this, you will make a ROS 2 node that subscribes to the `LaserScan` and `Odometry` messages. It should analyze the `LaserScan` data and, if necessary, publish an `AckermannDrive` with the `speed` field set to 0.0 m/s to brake. After you've calculated the array of iTTCs, you should decide how to proceed with this information. You'll have to decide how to threshold, and how to best remove false positives (braking when collision isn't imminent). Don't forget to deal with `inf`s or `nan`s in your arrays.

NOTE: on the rosmaster R2 car the lidar has 360 degree vision so if you have the screen attached or some other obstruction in the lidar vision you might want to exclude that range of angles from being considered or the vehicle will refuse to drive backwards.

To run your code copy the entire outer reactive_autonomy folder (the one with the package.xml file in it) onto the car or into the simulator as explained in the [Useful Commands](#useful-commands) section then run the automatic_braking node by running `ros2 run reactive_autonomy automatic_braking`.

### Follow the Gap
The skeleton code for the gap following node can be found in the `reactive_autonomy/gap_following.py` file.


For the second part of this lab you will make a node that drives forward while "following the gap" to avoid nearby obstacles and drive at the largest detected open area. One such algorithm is outlined below:

1. Preprocess LIDAR data
    1. Replace each point with the average value over a window (average the point and a few of its neighbors together) to reduce noise in the measurements
    2. Reject points that are over a threshold distance (> 3m for example)
2. Find the closest LIDAR point
3. Draw a safety bubble around this point by setting nearby points (maybe 8 points in either direction) to 0. All remaining non-zero points are now considered gaps.
4. Find the largest gap, in other words the largest consecutive string of non-zero elements in the array.
5. Find the best goal point in the chosen gap. This could be the furthest away point in the gap but other methods of determining the best point are also acceptable.
6. Drive the car towards the best point by publishing an `AckermannDrive` message to the `/drive` topic.

You can test your code in simulation using the default map or by modifying the map_path field in the `f1tenth_gym_ros/config/sim.yaml` to point to one of the maps in the `extra_maps` folder which have extra obstacles.

To run your code copy the entire outer reactive_autonomy folder (the one with the package.xml file in it) onto the car or into the simulator as explained in the [Useful Commands](#useful-commands) section then run the gap_following node by running `ros2 run reactive_autonomy gap_following`.

## III: Checkoffs
1. Demonstrate automatic braking
2. Demonstrate follow the gap
