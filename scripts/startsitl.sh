#!/bin/bash
cd ~/ardupilot/ArduCopter/
sim_vehicle.py --mavproxy-args="--streamrate=140" -v ArduCopter -f gazebo-iris --console --map
