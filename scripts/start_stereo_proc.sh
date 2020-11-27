#!/bin/bash
cd ~/csl_uav_simulator_ws/
source devel/setup.bash
ROS_NAMESPACE=iris_demo/ZED_stereocamera/camera rosrun stereo_image_proc stereo_image_proc _approximate_sync:=True
