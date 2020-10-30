# csl_uav_simulator
UAV simulator featuring ArduCopter and MAVROS integration


## Table of contents
* [Introduction](#introduction)
* [Packages explanation](#packages_explanation)
* [Install](#install)
* [Usage](#usage)



## Introduction
This Github repo features a UAV simulator integrating ArduCopter and MAVROS communication.
It is developed on Ubuntu 18.04 LTS using ROS Melodic (https://wiki.ros.org/melodic) and Gazebo 9.

Before continuing with the packages explanation you should have install the ArduPilot/Copter SITL on Linux based on the thread below.
Setting up SITL on Linux: https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html

## Packages explanation
### ardupilot_gazebo
The following plugin is a pure Gazebo plugin, so ROS is not needed to use it. You can still use ROS with Gazebo with normal gazebo-ros packages. It is based on the ArduPilot documentation (https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html).
The current package is a combination of the basic ardupilot_gazebo packages:

1. https://github.com/khancyr/ardupilot_gazebo
2. https://github.com/SwiftGust/ardupilot_gazebo

with some new and modified models according to the simulator's use.
We recommend you to study the Using Gazebo Simulator with SITL tutorial and then installing the present ardupilot_gazebo package in this repository.

### vrx
This package is the home to the modified source code and software documentation for the VRX Simulation and the VRX Challenge (https://github.com/osrf/vrx).
It is used from iris_coastline package to launch a sandisland world featuring an iris quadcopter.

### mavros
MAVLink extendable communication node for ROS. Modification made based on the original package (https://github.com/mavlink/mavros) in order to adapt it to our needs.

### usb_cam
A ROS driver for V4L USB cameras. In the simulator's case it can be used to extract image information from the ZED stereo camera mounted on our iris quadcopter.
Source code: https://github.com/ros-drivers/usb_cam (modifications were made).

### iris_coastline
Package launching the modified vrx world with an iris quadcopter featuring a ZED stereo camera, ArduCopter and MAVROS communications.

### iris_gazebo and iris_navigation
Packages created for a 2D navigation demo of the quadcopter featuring obstacle avoidance while using the move_base (https://wiki.ros.org/move_base) ROS package and Hokuyo Lidar.


## Install
Create a catkin workspace with the following commands: 

```
$ cd ~
$ mkdir -p csl_uav_simulator_ws/src
$ cd csl_uav_simulator_ws
$ catkin_make
```
After the the workspace is ready, clone the repository:
```
$ cd ~/csl_uav_simulator_ws/src
$ git clone https://github.com/sotomotocross/csl_uav_simulator.git
```
Move the ardupilot_gazebo package to the home directory and build it there (cross-check with the Using Gazebo Simulator with SITL documentation given above):
```
$ mv ardupilot_gazebo/ /home/$USER/
$ cd ~/adupilot_gazebo/
$ mkdir build
$ cd build
$ cmake ..
$ make -j4
$ sudo make install
```
Set path of Gazebo Models/Worlds... Open up .bashrc
```
$ sudo gedit ~/.bashrc
```
Copy & Paste Following at the end of .bashrc file
```
$ source /usr/share/gazebo/setup.sh

$ export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
$ export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
$ export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
$ export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
```
After installing ardupilot_gazebo and testing it continue with the workspace of the simulator:
```
$ cd ~/csl_uav_simulator_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make
$ source devel/setup.bash
```
Install is complete

Now launch a world file with a copter/rover/plane and ardupilot plugin, and it should work!

## Usage

### Initial launch of the world
```
$ cd ~/csl_uav_simulator_ws
$ source devel/setup.bash
$ roslaunch iris_coastline iris_coastline.launch
```
Open a second terminal and launch SITL through the scripts file in the repo:
```
$ cd ~/csl_uav_simulator_ws
$ source devel/setup.bash
$ cd src/scripts
$ ./startsitl.sh
```
Open a third terminal and launch mavros:
```
$ cd ~/csl_uav_simulator_ws
$ source devel/setup.bash
$ roslaunch mavros apm.launch
```
These three terminals launch the sandislad world with the iris quadcopter, the SITL (both communication, telemetry, console and map) and the mavros communcations.

### Iris quadcopter teleoperation
You will need a joystick to launch this package (ideally a Logitech Wireless F710).
If you have a joystick (if it's not a logitech you have to configure a yaml file for the axes and button mapping), having launched the three terminals above you open a fourth terminal and run the commands below:
```
$ cd ~/csl_uav_simulator_ws
$ source devel/setup.bash
$ roslaunch mavros_extras teleop.launch teleop_args:=-vel
```
when the package is launched you go the the MAVLink console and type the commands below:
```
$ mode GUIDED
$ arm throttle
$ takeoff 3
```
when the quadcopter has been taken off to 3 meters and the teleoperation package is on you can operate the vehicle through your joysticks.

### Obstacle avoidance (2D Navigation)
```
$ cd ~/csl_uav_simulator_ws
$ source devel/setup.bash
$ roslaunch iris_gazebo 
```

### Waves, wind, fog and ambient configuration
The RobotX-specific behavior of the models and the environment is generated by a set of Gazebo Model Plugins.
You can experiment with the basic plugins for us:
1. Wave characteristics
2. Wind velocity and windage coefficients
3. Fog and Ambient visual conditions
if you want to change the environment conditions of your simulation.

### World scenes configuration (Pending)
A pipeline is being developed through which the user can replace the original world in Gazebo with a real location chosen from Google Maps.
When it is completed all the details will be updated in this section!
