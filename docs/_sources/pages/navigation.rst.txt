Navigation2
===========

1. Overview
------------

The ROS 2 Navigation System is the control system that enables a robot
to autonomously reach a goal state, such as a specific position and
orientation relative to a specific map. Given a current pose, a map, and
a goal, such as a destination pose, the navigation system generates a
plan to reach the goal, and outputs commands to autonomously drive the
robot, respecting any safety constraints and avoiding obstacles
encountered along the way.

2. Running the demo
-------------------

The ``nav2_bringup`` package is an example bringup system for
navigation2 applications.

.. note::
    * We recommend doing this on a Ubuntu 18.04 installation. We have build issues on 16.04. (see https://github.com/ros-planning/navigation2/issues/353)
    * This stack and ROS2 are still in heavy development and there are some bugs and stability issues being worked on, so please do not try this on a robot without taking heavy safety precautions. THE ROBOT MAY CRASH!
    * It is recommended to start with simulation using Gazebo before proceeding to run on a physical robot

2.1 Launch Navigation2 in simulation with Gazebo
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Pre-requisites:
:::::::::::::::::

* Gazebo installed on the system
* gazebo_ros_pkgs for ROS2 installed on the system
* A Gazebo world for simulating the robot (see Gazebo tutorials)
* A map of that world saved to a map.pgm and map.yaml (see ROS Navigation tutorials)


**Terminal 1: Config turtlebot3 and Gazebo simulation environment**

Example: See `turtlebot3_gazebo models`_ for details.

.. code:: bash

   source /opt/robot_devkit/robot_devkit_setup.bash
   echo export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_simulations`/turtlebot3_gazebo/models/ >> ~/.bashrc
   echo export TURTLEBOT3_MODEL=waffle >> ~/.bashrc
   echo ROS_DOMAIN_ID=30 >> ~/.bashrc

**view result at rviz2 and Gazebo**

.. image:: ../imgs/nav2.png

**Terminal 2: Launch your robot in Gazebo**

.. code:: bash

    source /opt/robot_devkit/robot_devkit_setup.bash
    source /opt/ros/dashing/local_setup.bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

**Terminal 3: Launch navigation2**

.. code:: bash

    source /opt/robot_devkit/robot_devkit_setup.bash

    # Launch the nav2 system
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True autostart:=True \
    map:=`ros2 pkg prefix nav2_system_tests`/maps/map_circular.yaml

.. note::

    In RViz:

    * You should see the map.
    * Hit the "Startup" button.
    * Localize the robot using “2D Pose Estimate” button.
    * Make sure all transforms from odom are present. (odom->base_link->base_scan)
    * Send the robot a goal using “Navigation2 Goal” button. Note: this uses a ROS2 Action to send the goal, and a pop-up window will appear on your screen with a 'cancel' button if you wish to cancel.

    To view the robot model in RViz:

    * Add "RobotModel", set "Description Source" with "File", set "Description File" with the name of the urdf file for your robot (example: turtlebot3_burger.urdf)"


2.2 Launch Navigation2 on a Robot
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Pre-requisites:
:::::::::::::::::

* Run SLAM or Cartographer with tele-op to drive the robot and generate a map of an area for testing first. The directions below assume this has already been done. If not, it can be done in ROS1 before beginning to install our code.
* Publish all the transforms from your robot from base_link to base_scan

**Terminal 1: Launch the robot**

.. code:: bash

   echo export TURTLEBOT3_MODEL=waffle >> ~/.bashrc
   echo ROS_DOMAIN_ID=30 >> ~/.bashrc
   source /opt/robot_devkit/robot_devkit_setup.bash
   ros2 launch turtlebot3_bringup robot.launch.py

**Terminal 2: Launch the code using this launch file and your map.yaml**

.. code:: bash

    source /opt/robot_devkit/robot_devkit_setup.bash
    source /opt/ros/dashing/local_setup.bash
    ros2 launch nav2_bringup nav2_bringup_launch.py map:=<full/path/to/map.yaml>

**Terminal 3: Run RVIZ**

.. code:: bash

   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz

**In RVIZ2:**

    * Make sure all transforms from odom are present. (odom->base_link->base_scan)
    * Click 2D Pose Estimate button in the menu bar and then point exact pose of TurtleBot3 on the map.
    * If TurtleBot3 is close to a costmap or nearby the costmap map, click Navigation2 Goal button in the menu bar and then point goal pose on the map.

3. Known issues
---------------

* This stack and ROS2 are still in heavy development and there are some bugs and stability issues being worked on, so please do not try this on a robot without taking *heavy* safety precautions. THE ROBOT MAY CRASH! 

* For a current list of known issues, see *https://github.com/ros-planning/navigation2/issues*.

4. ToDo
-------

* Add additional maps and examples.
* Add instructions for running navigation2 with SLAM.

.. _turtlebot3_gazebo models: https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2/turtlebot3_gazebo/models
.. _turtlebot3_gazebo: https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2/turtlebot3_gazebo
