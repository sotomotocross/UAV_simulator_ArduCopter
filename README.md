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




## Install

## Usage
