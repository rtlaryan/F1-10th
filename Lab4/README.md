# Lab 4: Controls

## I. Objectives

- Use Slam Toolbox to create a map
- Create a PID controller
- Create a Pure Pursuit controller
- Create a Stanley controller

## II. Overview

**IMPORTANT NOTE 1:** The `control` package depends on the `state_estimation` particle filter when running on the real car so make sure to have the Lab3 folder in the `src` folder of your workspace. It might be helpful to make sure to turn off the sensor model plot popup in the particle filter so you don't have to keep closing it. For simulation you can now reenable the `publish_tf` parameter in the f1tenth_gym_ros `config/sim.yaml` file

**IMPORTANT NOTE 2:** Due to a combination of issues with odometry and not having full hardware acceleration on the real car the particle filter can sometimes cause the position of the car to drift as it is driving around. This can cause issues with both recording the path and following it using the controller. A good way to spot the issue is if you see some of the arrows of the trajectory move sideways relative to the direction they are facing. We don't have a great solution for this yet so just be aware that if the controller isn't performing as expected on the real car it may not be an issue with your code.

### Useful Tools
#### RViz
RViz launches automatically with the simulator. To run it on the car make sure to have started the docker container from the car instead of an ssh connection (this sets the DISPLAY environment variable) then run `rviz2`. At the bottom of the pane on the left is the Add button which can be used to add topics to visualize. It is usually easiest to select the By Topic tab in the popup so that it will display the available topics to pick from. The particle filter defaults to publishing a couple useful visualization topics. Specifically, the red PoseArray is the commanded path while the blue PoseArray is the actual trajectory of the car.

A configuration file with these topics already added called `control.rviz` is provided and can be run using `rviz2 -d pf.rviz`. The functionality to manually reinitialize the particle filter by clicking the Publish Point button at the top and clicking anywhere in the map is also still available. If you want to clear the visualized data from a topic you can deselect then reselect the checkbox next to its type in the list on the left. This configuration file also adds the SlamToolboxPlugin that will allow you to save your map file below the topics list on the left (In order for this to work you need to have updated the AMR docker container or installed slam toolbox manually).

![Rviz](https://github.gatech.edu/ECE4804-AMR/Lab4/wiki/images/lab4_rviz_follow_path.png)


#### Plotjuggler
Plotjuggler can be used to generate plots of data both by opening a ROS bag file or by subscribing directly to topics and recording in real time.

![Plotjuggler](https://github.gatech.edu/ECE4804-AMR/Lab4/wiki/images/lab4_plotjuggler.png)

Installation:
* Docker: You can either update the docker container by pulling the newest changes from Github: `git pull` then building the new version by running `docker-compose build` (it should be a fairly quick update) or by following the native installation instructions below from inside the container.
* Native: Install plotjuggler directly by running `sudo apt update` then `apt-get install ros-humble-plotjuggler-ros`

Usage:
* Click the icon in the top left next to "Data:" and browse to the yaml file inside a ROS bag file or click the start button in the Streaming section on the left and select the specific topics you want to view. You can now drag topics from the list on the left into the open center area to plot them. Right clicking the graph allows you to split it as well as clear the topics being displayed. If you are monitoring topics in real time it can be useful to increase the buffer under the Streaming section to record a longer period of time.


### Create a map with Slam Toolbox
You can wait to do this step until you are ready to test on the real car. The `slam_launch.py` file does not work in simulation currently.
1. Install slam toolbox by updating the AMR Docker container (cd to the folder, `git pull`, then run `./build.sh`. The update should be fairly quick) or install it manually by running `apt update` then `apt install ros-humble-slam-toolbox`.
2. Run rviz: `rviz2 -d control.rviz`
3. In a new terminal build the control package and source it: cd to the `amr_ws` folder, run `colcon build` then `source install/setup.bash`
4. Place the car in course. It is probably easiest to place it somewhere on the starting line.
5. Run `ros2 launch control slam_launch.py` (This will run all of the necessary hardware nodes as well, you SHOULD NOT run the `amr_car_launch.py` file at the same time as this).
4. Drive the car VERY slowly around the course. The docker container does not have full hardware acceleration so the SLAM update process will lag behind if you go too quickly. It is also recommended to stop and make sure the visualization of the map has caught up every fourth to eighth of the track.
5. Once you have the course mapped out in rviz save the map using the save map button in the panel in the bottom left (shown below). You can name the map whatever you want but a later portion will need the map to be named map and stored in the `amr_ws` folder. If you choose to go this route make sure to write out the full path with no file extension: `/root/amr_ws/map`

![rviz save map button](https://github.gatech.edu/ECE4804-AMR/Lab4/wiki/images/lab4_slam_save_map.png)


### Controllers in simulation
The skeleton code for the controllers can be found in the `control` file. Specifically. All sections of code that you are intended to complete have been marked with `TODO: ` in the code.

First, there are 2 functions to be completed in the `control/controller.py` file.
1.  `compute_position_in_frame`: This function is used to calculate the position of a pose in a different coordinate frame. (This is typically done as a matrix multiplication)
2. `get_reference_index`: This function is used to find the index of a point on the trajectory for the controller to use as its target. This is typically accomplished by finding the closest point to the vehicle on the trajectory then finding the first point further along the trajectory that is farther away than the lookahead distance.

Next, each of the controllers implementation files have 2 functions to fill in:
1. `get_error`: This function uses the current pose of the vehicle and the reference pose found earlier to compute the error used for the controller. Note that the provided pose argument is always in the lidars reference frame but the controllers should calculate their error relative to either the front or rear axle depending on the specific controller.
2. `get_control`: This function uses the pose, reference pose + velocity, and error to output the commanded forward speed and steering angle. Our trajectory follower precalculates the velocity for each point so you can just return the velocity from the reference.

#### PID
Skeleton code is in `control/pid.py`. The PID controller has two parameters (`kp` and `kd`) and should calculate its error in reference to the back axle

#### Pure Pursuit
Skeleton code is in `control/pure_pursuit.py`. The Pure Pursuit controller has no parameters of its own but depends on the `distance_lookahead` parameter and should calculate its error in reference to the back axle

#### Stanley
Skeleton code is in `control/stanley.py`. The Stanley controller has two parameters (`kv` and `ke`) and should calculate its error in reference to the front axle

#### Parameters
In the `config/controller.yaml` file there are a variety of parameters for the controllers. It will likely be necessary to tune the parameters to get the controllers to efffectively follow the trajectory. Also, the best parameters for simulation and the real car may be different.

#### Running the controllers
1. Select which controller you want to test in the `config/controller.yaml` file then `colcon build` from your workspace folder (`/sim_ws` for the f1tenth docker).
2. Run the simulator: `ros2 launch f1tenth_gym_ros gym_bridge_launch.py`.
    If you want to display the trajectories in rviz use the file/open dialog to open the `control.rviz` file provided. You will need to use the add button then the by topic section to add the `/odom` topic then set the `keep` parameter in the topic on the left to 1 if you want to see an arrow where the car is. See the rviz section above for more information about the topics.
3. Start the controller: `ros2 launch control control_sim_launch.py`
4. Record a path for the car to follow (or skip this step and use the `test_path.csv` file): In a new terminal window run `ros2 run control record_path <file path>` (If you do not provide a filename it will default to `path.csv`) then drive the path you want to store (you may need to open another terminal to run `ros2 run teleop_twist_keyboard teleop_twist_keyboard)`. Finally, `Ctrl+C` the record_path process to cause it to save the file.
5. Publish a path to the controller: You can reuse the terminal from step 4 for this. Run `ros2 run control publish_path`.
6. You should be able to watch the car follow the commanded path in rviz now.

### Running controllers on the real car
As always please ensure that your code is functioning in simulation before running it on the real car to minimize the chance of damaging any hardware. Additionally, start with a low velocity (no higher than 0.4 m/s) and keep a close eye on the car so you can use `Ctrl+C` to stop the car if it is going to hit something.

1. Rename the map files you generated using SLAM to map.png/pgm and map.yaml and placed them in `~/amr_ws`. Also, if you renamed the files you will need to edit the first line of the `map.yaml` file to match the new name.
2. If you wish to save a bag file to use to generate plots later run `ros2 bag record -o <filename> /cmd_path /viz/real_path /pf/pose/odom /distance_error /drive`
3. In a new terminal run the controller and hardware nodes: `ros2 launch control control_real_launch.py`. You SHOULD NOT run the `amr_car_launch.py` file at the same time as this.
4. Record a path: in a new terminal run `ros2 launch control record_real_launch.py` then drive the path you wish to save. Finally, `Ctrl+C` the process to save the path. The file will be named `path.csv` and will be saved wherever you ran the command from.
5. Publish a path to the controller: `ros2 run control publish_path <file path>` (It will default to looking for `path.csv` in the current folder). Be prepared to switch back over to the terminal from step 3 to use `Ctrl+C` to stop the car if it is going to hit something.
6. Once you are done recording the bag file use `Ctrl+C` to stop it and save the file.

## III: Checkoffs
1. Show the map generated by Slam Toolbox
2. Demonstrate the PID controller following a path
2. Demonstrate the Pure Pursuit controller following a path
2. Demonstrate the Stanley controller following a path

# IV: Lab Report
I highly recommend taking screenshots of the cars trajectories in rviz on the car or saving bag files and replaying them in rviz (`ros2 bag play <file name>`) in the f1tenth simulator to get figures to put in your lab report. Plotjuggler plots are also a great addition but since you can only plot variables with respect to time (no 2d trajectories) it isn't always clear what the path looked like.

