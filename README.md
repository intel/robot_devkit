DISCONTINUATION OF PROJECT

This project will no longer be maintained by Intel.

Intel has ceased development and contributions including, but not limited to, maintenance, bug fixes, new releases, or updates, to this project.  

Intel no longer accepts patches to this project.

If you have an ongoing need to use this project, are interested in independently developing it, or would like to maintain patches for the open source software community, please create your own fork of this project.  

Contact: webadmin@linux.intel.com
# Intel Robot DevKit

## 1. Introduction
Intel Robot DevKit is the tool to generate Robotics Software Development Kit (RDK) designed for autonomous devices, including the ROS2 core and capacibilities packages like perception, planning, control driver etc. It provides flexible build/runtime configurations to meet different autonomous requirement on top of diversity hardware choices, for example use different hareware engine CPU/GPU/VPU to accelerate AI related features.

After build, the RDK is installed on Ubuntu18.04 with below items on development machine for further development.
* ROS2 core
* ROS2 Object Analytics(OA) with RealSense(RS) camera
* ROS2 OpenVINO for people detection
* Gazebo 9 with Gazebo-ros2 simulator
* ROS2 navigation
* rviz2
* ROS2 tutorial for Intel components

### Hardware Requirements
* [Intel NUC](https://www.intel.com/content/www/us/en/products/boards-kits/nuc/kits.html) (CPU: Intel i7-6700HQ @2.60GHz, Mem:16G) or [ADLink Neuron Board](https://neuron.adlinktech.com/en/)
* Intel Movidius Neural Compute Stick 2
* Intel RealSense D400 Series, Intel RealSense T265

### Robot support
* Turtlebot 3 Waffle

### System Requirements
* We support Ubuntu Linux Bionic Beaver 18.04 on 64-bit. We not support Mac OS X and Windows.

## 2. How to use
Please refer to [Intel ROS2 Project Tutorial](https://intel.github.io/robot_devkit_doc/) for more details.

## 3. Report issue
If run into any issue of RDK, feel free to report issue in this project.

###### *Any security issue should be reported using process at https://01.org/security*
