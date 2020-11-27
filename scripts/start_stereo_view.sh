#!/bin/bash
cd ~/csl_uav_simulator_ws/
source devel/setup.bash
rosrun image_view stereo_view stereo:=/iris_demo/ZED_stereocamera/camera image:=image_raw _approximate_sync:=True _queue_size:=10
