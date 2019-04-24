# Intel Robot SDK

## 1. Introduction
Intel Robot SDK is the tool to generate Robotics Software Development Kit (SDK) designed for autonomous devices, including the ROS2 core and capacibilities packages like perception, planning, control driver etc. It provides flexible build/runtime configurations to meet different autonomous requirement on top of diversity hardware choices, for example use different hareware engine CPU/GPU/VPU to accelerate AI related features. Further more, it is integrated and validated for real-time, FuSa, security with platform capacibilities.

After build, the SDK is installed on Ubuntu18.04 with below items on development machine for further development.
* ROS2 core
* ROS2 Object Analytics(OA) with RealSense(RS) camera and Movidius NCS2
* ROS2 OpenVINO for people detection
* Gazebo 9 with Gazebo-ros2 simulator
* ROS2 navigation
* rviz2
* ROS2 tutorial for Intel components

### Hardware Requirements
* Intel NUC (CPU: Intel i7-6700HQ @2.60GHz, Mem:16G) or [ADLink Neuron Board](https://neuron.adlinktech.com/en/)
* Intel Movidius Neural Compute Stick 2
* Intel RealSense D400 Series

### Robot support
* Turtlebot 3 Waffle

### System Requirements
* We support Ubuntu Linux Bionic Beaver 18.04 on 64-bit. We not support Mac OS X and Windows.

## 2. How to use
Please refer to [Intel ROS2 Project Tutorial](https://intel.github.io/robot_sdk/) for more details.

## 3. Report issue
If run into any issue of SDK, feel free to report issue in this project.

###### *Any security issue should be reported using process at https://01.org/security*
