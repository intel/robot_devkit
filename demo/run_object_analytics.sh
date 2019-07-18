#!/bin/bash

source /opt/robot_devkit/robot_devkit_setup.bash

ros2 launch realsense_ros2_camera rs.launch.py &

ros2 launch dynamic_vino_sample ros2_openvino_oa.launch.py &

ros2 launch object_analytics_node object_analytics_sample.launch.py
