# Lab 3: Localization

## I. Objectives

- Particle Filter (Part A)
- Scan Matching (Part B)

## II. Overview
The particle filter requires new dependencies so make sure to udpate both the F1Tenth Simulator and the AMRDocker on the car before trying to run any code. It is recommended to write and test as much of the code as possible using the simulator before testing it on the real car both to save time and to catch any bugs before potentially damaging real hardware.

### Useful Commands
### RViz
RViz launches automatically with the simulator. To run it on the car make sure to have started the docker container from the car instead of an ssh connection (this sets the DISPLAY environment variable) then run `rviz2`. At the bottom of the pane on the left is the Add button which can be used to add topics to visualize. It is usually easiest to select the By Topic tab in the popup so that it will display the available topics to pick from. The particle filter defaults to publishing a couple useful visualization topics. `pf/viz/particles` shows all of the particles while `pf/viz/inferred_pose` shows the estimated pose.

A configuration file with these topics already added called `pf.rviz` is provided and can be run using `rviz2 -d pf.rviz`. By default the PoseArray is left disabled for performance reasons but if you enable the checkbox you can visualize all of the particles from the particle filter. Additionally, rviz can be used to manually reinitialize all of the particles to a random global distribution by clicking the Publish Point button at the top and then click anywhere in the map. Using the 2D Pose Estimate button instead allows you to initialize all of the particle filters particles close to the specified pose.

### Update the F1Tenth Simulator
1. cd to the `f1tenth_gym_ros` folder
2. Pull the newest changes from GitHub: `git pull`

If you are using the docker container build the new version by running `docker-compose build`

If you are not using the docker container then you will need to manually install some dependencies:

1. Install apt dependencies:
    1. `sudo apt-get update`
    2. `sudo apt-get install python3-pip ros-humble-tf-transformations python3-tk`
2. Install pip dependencies:
    1. `pip3 install transforms3d cython matplotlib`
3. Install range_libc:
    1. Clone range_libc (It doesn't matter where you do this, you can delete the repo once it is installed): `git clone https://github.com/chachmu/range_libc.git`
    2. cd to the `range_libc/pywrapper` folder: `cd range_libc/pywrapper`
    3. install range_libc: `python3 setup.py install`

### Update the AMRDocker container
1. cd to the `AMRDocker` folder on the car: `cd ~/AMRDocker`
2. Pull the newest changes from GitHub: `git pull`
3. Build the new version of the docker container: `./build.sh`

### Particle Filter
The skeleton code for the particle filter can be found in the `state_estimation/particle_filter.py` file. All sections of code that you are intended to complete have been marked with `TODO: ` in the code.

#### Parameters
In the `config/particle_filter.yaml` file there are a variety of parameters that effect the performance of the particle filter. The `angle_step`, `max_particles`, and `max_range` have been confirmed to work well in sim but you are welcome to tune them further to improve performance. The `squash_factor`, sensor model constants, and motion model constants have been set to default values so you will need to determine their proper values.

#### Initial Particle Distribution
In the `initialize_global` function (line 354) you will need to fill in the `self.particles` and `self.weights`. `self.particles` is a MAX_PARTICLES by 3 numpy array of x, y, and theta values for each particle. `self.weights` is a MAX_PARTICLES long numpy array of the probability for each particle. The function should randomly distribute the particles across the map and initialize the weights array with starting values.

#### Sensor Model
For performance reasons instead of calculating the sensor model on the fly a 2d array of probabilities is pregenerated in the `precompute_sensor_model` function (line 382). There is already skeleton code to iterate through each point in the 2d array so you will need to complete the sensor probability calculation in the inner loop and store it in the `prob` variable.

At the bottom of the `precompute_sensor_model` function there are several `if` statements that are currently set to False. You can manually set the values to True to generate plots of your sensor model. I would recommend at least working to get the bottom plot (the 1d PDF of measured distance assuming a ground truth value of 140 px) to look similar to the combined probabalistic model from lecture 12 before you try to test the overall performance of the particle filter.

#### Motion Model
The motion model is applied to the particles in the `motion_model` function (line 454). You will need to modify the proposal_dist array directly to motion noise to it.

#### Resampling
The first step in the `MCL` function (line 556) is to resample the particle distribution to form the new proposal distribution. You will need to add code to randomly sample the `self.particle_indices` array then use the new indices to generate the new proposal distribution and store it in the `proposal_distribution` variable.

#### Running the Particle Filter
When running the simulator with the particle filter make sure to set the `publish_tf` flag in the `f1tenth_gym_ros/config/sim.yaml` file to `False` so that the particle filter can publish its own estimates of the cars location. Since the simulator already publishes a map topic when you launch the particle filter for simulation use `ros2 launch state_estimation pf_sim_launch.py`. When running on the car use `ros2 launch state_estimation pf_real_launch.py` to also launch a map server.


### Scan Matching
The scan matching portion of the lab is not ready yet.

## III: Checkoffs
1. Demonstrate the particle filter localizing in the real course
2. Demonstrate the scan matching node localizing in the real course

# IV: Lab Report
I highly recommend putting plots of your sensor model in the lab report.
