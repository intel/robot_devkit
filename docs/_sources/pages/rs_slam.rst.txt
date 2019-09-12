RealSense™ for SLAM and Navigation
==================================

1. Overview
-----------

SLAM with cartographer requires laser scan data for robot pose estimation. Intel® RealSense™ depth cameras (D400 series) can generate depth image, which can be converted to laser scan with depthimage_to_laserscan package and t265 camera can provide pose information as a odometer. Therefore, we provide a way to use RealSense™ for SLAM and navigation.

2. SLAM with RealSense™
------------------------

\ **Install dependencies**\

.. code:: bash

    source /opt/robot_devkit/robot_devkit_setup.bash
    mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
    git clone https://github.com/ros-perception/depthimage_to_laserscan -b dashing-devel
    cd .. && colcon build
    source ~/ros2_ws/install/local_setup.bash

\ **Start to SLAM**\ 

.. code:: bash

   # In terminal 1, launch cartographer node
   source ~/ros2_ws/install/local_setup.bash
   ros2 launch realsense_examples rs_cartographer.launch.py

   # In terminal 2, launch Intel® RealSense™ D400 camera and T265 camera 
   # You should config the serial number and tf in the launch file ros2_intel_realsense/realsense_examples/launch/rs_t265_and_d400.launch.py before launch the camera 
   source /opt/robot_devkit/robot_devkit_setup.bash
   ros2 launch realsense_examples rs_t265_and_d400.launch.py

   # In terminal 3, launch the turtlebot3 for RealSense™ SLAM
   export TURTLEBOT3_MODEL=waffle
   source /opt/robot_devkit/robot_devkit_setup.bash
   ros2 launch realsense_examples tb3_robot.launch.py

   # In terminal 4, launch the teleoperation node for robot
   source /opt/robot_devkit/robot_devkit_setup.bash
   export TURTLEBOT3_MODEL=waffle
   ros2 run turtlebot3_teleop teleop_keyboard

Control and move the turtlebot3 with keyboard to build map, and when the map building process is done, please save the map with the following command:

.. code:: bash

   # In terminal 5
   source /opt/robot_devkit/robot_devkit_setup.bash
   source /opt/ros/dashing/local_setup.bash
   ros2 run nav2_map_server map_saver -f ~/map

Next, try to open and preview the map.pgm to confirm it. The following is a map built with RealSense™ and cartographer:

.. image:: ../imgs/slam_with_rs.png

3. Navigation with RealSense™
-----------------------------

Generally, In order to navigation with the map from SLAM with RealSense™, the ros2 navigation stack should be built and ready to use.

\ **Bringup the turtlebot3**\ 

.. code:: bash

   # In terminal 1
   export TURTLEBOT3_MODEL=waffle
   source /opt/robot_devkit/robot_devkit_setup.bash
   ros2 launch realsense_examples tb3_robot.launch.py

\ **Start ROS2 realsense and depth image to laser scan**\ 

.. code:: bash

   # In terminal 2
   source /opt/robot_devkit/robot_devkit_setup.bash
   source ~/ros2_ws/install/local_setup.bash
   ros2 launch realsense_examples rs_nav.launch.py

\ **Start the navigation2 stack with the map**\ 

.. code:: bash

   # In terminal 3
   export TURTLEBOT3_MODEL=waffle
   source /opt/ros/dashing/local_setup.bash
   ros2 launch nav2_bringup nav2_bringup_launch.py map:=$HOME/map.yaml

   # In terminal 4
   source /opt/ros/dashing/local_setup.bash
   ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/nav2_default_view.rviz


Finally, please give an initial pose and goal within RVIZ2 to direct and navigate the turtlebot3 with the running map.

4. Known issues
---------------

* Keep the RealSense™ parallel to the ground, or the tilt of the RealSense™ may influence the SLAM.