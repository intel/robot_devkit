#!/bin/bash

ROS_VERSION=release-dashing-20190531
SRC=src_$ROS_VERSION
ROS2_DISTRO=dashing

rm -rf temp
mkdir temp
cd temp
git clone https://github.com/ros2/ros2.git -b $ROS_VERSION

rm -rf $SRC
mkdir $SRC
vcs import $SRC < ros2/ros2.repos

rosdep install -s --reinstall --from-paths "$SRC" --ignore-src --rosdistro "$ROS2_DISTRO" -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69  rti-connext-dds-5.3.1 urdfdom_headers" > ros_deps.txt
