# csl_uav_simulator
UAV simulator featuring ArduCopter and MAVROS integration


## Table of contents
* [Introduction](#introduction)
* [Packages explanation](#packages_explanation)
* [Install](#install)
* [Demanding script changes](#demanding_script_changes)
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
$ cd ../
$ git clone https://github.com/HBPNeurorobotics/gazebo_dvs_plugin.git
$ git clone https://github.com/uzh-rpg/rpg_dvs_ros.git
$ git clone https://github.com/catkin/catkin_simple.git
```
Move the ardupilot_gazebo package to the home directory and build it there (cross-check with the Using Gazebo Simulator with SITL documentation given above):
```
$ cd csl_uav_simulator/
$ mv ardupilot_gazebo/ /home/$USER/
$ cd ~/ardupilot_gazebo/
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
After installing ardupilot_gazebo and testing it continue with the setup of the ecatkin_ws:
```
$ mkdir -p ~/ecatkins_ws/src
$ cd ~/ecatkin_ws
$ pip3 install numpy
$ pip3 install scipy matplotlib pillow
$ pip3 install imutils h5py==2.10.0 requests progressbar2
$ pip3 install cython
$ pip3 install scikit-learn scikit-build scikit-image
$ pip3 install opencv-contrib-python==4.4.0.46
$ pip3 install opencv-python==4.4.0.42
$ pip3 install rospkg empy
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
```
Now you can move all the ecatkin_ws content from the repo to the src directory of the ecatkin_ws you just created and build.
Then you execute the commands below:
```
$ cd ~/ecatkin_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
$ source devel/setup.bash
```
Go to the rpg_dvs_ros github repo (https://github.com/uzh-rpg/rpg_dvs_ros) and install all the dependencies according to the documentation of the package.
Since the catkin_make of the ecatkin_ws is succesful you can build the simulator:
```
$ cd ~/csl_uav_simulator_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make
$ source devel/setup.bash
$ cd src/csl_uav_simulator_ws/src/csl_uav_simulator/mavros/mavros/scripts
$ ./install_geographiclib_datasets.sh
$ cd ~/cs_uav_simulator_ws
```
Install is complete

Now launch a world file with a copter/rover/plane and ardupilot plugin, and it should work!

## Demanding script changes

### First
File: iris_coastline.launch
Inside the iris_coastline package.
In the line 86 you change the path of the model.sdf launched inside the script
```
<arg name="sdf_robot_file" default="/home/$USER/ardupilot_gazebo/models/iris_with_ardupilot_and_zed_stereocamera/model.sdf" />
```

### Second
File spawn_drone.launch
Inside the iris_gazebo package.
In the line 13 change the path for the model.sdf file launched inside the script.
```
<arg name="sdf_robot_file" default="/home/sotiris/ardupilot_gazebo/models/iris_with_lidar/model.sdf" /> 
```

## Usage

### Initial launch of the world
```
$ cd ~/csl_uav_simulator_ws/src/csl_uav_simulator/scripts
$ ./startgz.sh
```
Open a second terminal and launch SITL through the scripts file in the repo:
```
$ cd ~/csl_uav_simulator_ws/src/csl_uav_simulator/scripts
$ ./startsitl.sh
```
Open a third terminal and launch mavros:
```
$ cd ~/csl_uav_simulator_ws/src/csl_uav_simulator/scripts
$ ./startmavros
```
These three terminals launch the sandislad world with the iris quadcopter, the SITL (both communication, telemetry, console and map) and the mavros communcations.
If everyhting are launched succesfuly then you will have topics both from the ZED stereo camera and from the DVS (only one topic that gives events). The DVS has not a body. You will be watching only its field of view.
While the simulator is running you open a terminal and run the commands below that can start the trained NN for coastline detection running:
```
$ cd ~/csl_uav_simulator_ws/src/csl_uav_simulator/scripts
$ ./start_coast_det.sh
```

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
Open a terminal and launch an empty world with iris_gazebo
```
$ cd ~/csl_uav_simulator_ws
$ source devel/setup.bash
$ roslaunch iris_gazebo iris_empty_world.launch
```
open a second terminal and spawn the quadcopter equipped with a Hokuyo Lidar:
```
$ cd ~/csl_uav_simulator_ws
$ source devel/setup.bash
$ roslaunch iris_gazebo spawn_drone.launch
```
open a third terminal and run sitl
```
$ cd ~/ardupilot/ArduCopter
$ ../Tools/autotest/sim_vehicle.py --map --console
```
open a fourth terminal and launch mavros:
```
$ cd ~/csl_uav_simulator_ws
$ source devel/setup.bash
$ roslaunch mavros apm.launch
```
open a fifth terminal and launch the move_base package:
```
$ cd ~/csl_uav_simulator_ws
$ source devel/setup.bash
$ roslaunch iris_navigation move_base.launch
```
open a sixth terminal and begin navigation:
```
$ cd ~/csl_uav_simulator_ws
$ source devel/setup.bash
$ rosrun iris_navigations cmd_vel_to_mavros.py
```
open a seventh terminal to run rviz and visualize the navigation of the quadcopter:
```
$ rosrun rviz rviz
```
In the rviz environment open csl_uav_simualtor/iris_navigation/navigation.rviz.
Then go to the SITL console and give the commands bellow:
```
$ mode GUIDED
$ arm throttle
$ takeoff 3
```
Then select 2D nav goal, inside the rviz environment.

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
