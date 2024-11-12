# Lab 5: Planning

## I. Objectives

- Develop a working RRT path planner
- Local Motion Control to drive around a track

## II. Overview

**IMPORTANT NOTE 1:** The `planning` package depends on the `control` package but not the `state_estimation` package. Make sure you have the Lab4 folder in the `src` folder of your workspace. 

### Useful Tools
#### RViz
RViz launches automatically with the simulator. To run it on the car make sure to have started the docker container from the car instead of an ssh connection (this sets the DISPLAY environment variable) then run `rviz2`.  The rrt planner defaults to publishing a couple useful visualization topics.

A configuration file called `planning.rviz` is provided and can be run using `rviz2 -d <path_to_planning.rviz>`. This sets the fixed frame to the cars lidar frame since there will be no global position data and will display these topics:
1. Map topic `/viz/occupancy_grid`: Displays the occupancy grid in the laser frame.
2. Marker topic `/viz/rrt_tree`: Displays the tree structure of the RRT as red lines and a green dot representing the target point.
3. PoseArray topic `/viz/planned_path`: blue arrows representing the points of the path.

![Rviz](https://github.gatech.edu/ECE4804-AMR/Lab5/wiki/images/lab5_rviz.png)


### RRT Planner
The RRT portion of the lab is a standard RRT path planner based on this [paper](https://arxiv.org/pdf/1105.1186.pdf) (specifically sections 3.1, 3.2, and Algorithm 3). The planner_node that provides the RRT with target points is not based on any real algorithm. It just generates 9 different target points based on the provided lookahead distance and a set of angles (starting from the center and moving out). These target points are each evaluated by the RRT. If the point is inside an obstacle or the RRT fails to generate a path the planner tries the next point in the sequence. If none of the points provide a valid path it simply drives forward slowly hoping to get to a position where the planner starts working again.

The skeleton code for the RRT path planner can be found in the `rrt.py` file while all of the rrt, occupancy grid, and controller parameters are in the `config/rrt.yaml` file. All of the functions you need to complete have been marked in the code with `# TODO: `. You shouldn't need to modify the `planner_node.py` file unless you want to change the target point generation.

#### Occupancy Grid
The RRT planner uses an occupancy grid to determine where obstacles are. The first function to complete in the `rrt.py` file is the `update_occupancy_grid` function which takes a Laserscan message and produces the occupancy grid. The grid is a square centered on the car horizontally and extending out from the front of the lidar in the forward direction. The format must be a 2d numpy array using `grid[x][y]` with 0 representing an opening and 1 representing an obstacle. Be careful with the conversion from polar coordinates to cartesian coordinates and the conversion to the array indices. Also, make sure to properly bounds check the indices after inflating the obstacle.

#### RRT
The RRT pseudocode shown below describes the outline of the `plan_path` function which will run the RRT and produce a path. A class called RRTNode is provided which must be used for the nodes when building the tree. Instead of `sampleFree` you should write the `sample` function to uniformly sample from the planning space then check if the point is valid using the `check_point_collision function`. Similarly, instead of using `ObstacleFree` you should use the `check_point_collision` function to check for collisions. It is recommended to test the collision functions thoroughly since it can be very difficult to tell if they are working properly just by watching the RRT (one effective method is to manually generate nodes inside/across obstacles and call the collision functions).

![RRT](https://github.gatech.edu/ECE4804-AMR/Lab5/wiki/images/lab5_algorithm.png)

#### Controller
The controller is set to be disabled by default so the car won't drive itself while you are testing the occupancy grid or RRT. Once you are ready you can set the `enabled` flag in the config file to True and the controller should automatically use all of the settings provided in the config file. The car has been tested with Stanley control using the provided `ke` and `kv` values but you are welcome to use whatever controller and tuning you prefer.

When testing the controller in simulation the AMR map provided with the simulator is a bit too small for the car to comfortably navigate. To fix this you can either switch back to the `levine.yaml` map by modifying the `f1tenth_gym_ros/config/sim.yaml` file or you can scale up the AMR map by modifying the `AMR.yaml` map file to:
```
image: AMR.png
mode: trinary
resolution: 0.08
origin: [-3.93, -1.52, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

#### Running the planner

As always it is recommended to test everything in simulation to start then move to the real car once everything seems to be working properly.

1. Start up whichever platform you intend to use
    * Simulation: `ros2 launch f1tenth_gym_ros gym_bridge_launch.py`
    * Real Car: `ros2 launch amr_bringup amr_car_launch.py` (don't forget to unplug the controller usb dongle)
2. In a new terminal Build and source in the workspace folder: `colcon build` then `source install/setup.bash`
3. Run the RRT planner: `ros2 launch planning rrt_launch.py`

## III: Checkoffs
1. Demonstrate the RRT planner generating paths
2. Demonstrate the car driving around the track

