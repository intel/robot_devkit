RealSense™ for SLAM and Navigation
==================================

.. warning::

   To be updated base on turtlebot3 readiness for dashing release.

1. Overview
-----------

SLAM with cartographer requires laser scan data for robot pose estimation. Intel® RealSense™ depth cameras (D400 series) can generate depth image, which can be converted to laser scan with depthimage_to_laserscan package and t265 camera can provide pose information as a odometer. Therefore, we provide a way to use RealSense™ for SLAM and navigation.

2. SLAM with RealSense™
------------------------

\ **Start to SLAM**\ 

.. code:: bash

   # In terminal 1, launch cartographer node
   ros2 launch realsense_ros rs_cartographer.launch.py
   # In terminal 2, launch Intel® RealSense™ D400 camera and T265 camera 
   # You should config the serial number before launch the camera folow https://intel.github.io/robot_devkit/pages/rs.html
   source /opt/robot_devkit/robot_devkit_setup.bash
   ros2 launch realsense_ros multi_cam.launch.py
   # In terminal 3, launch the turtlebot3
   export TURTLEBOT3_MODEL=waffle
   source /opt/robot_devkit/robot_devkit_setup.bash
   ros2 launch turtlebot3_bringup robot.launch.py

   # In terminal 4, launch the teleoperation node for robot
   source /opt/robot_devkit/robot_devkit_setup.bash
   export TURTLEBOT3_MODEL=waffle
   ros2 run turtlebot3_teleop teleop_keyboard

Control and move the turtlebot3 with keyboard to build map, and when the map building process is done, please save the map with the following command:

.. code:: bash

   # In terminal 5
   source /opt/robot_devkit/robot_devkit_setup.bash
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
   ros2 launch turtlebot3_bringup robot.launch.py

\ **Start ROS2 realsense and depth image to laser scan**\ 

.. code:: bash

   # In terminal 2
   source /opt/robot_devkit/robot_devkit_setup.bash
   ros2 launch realsense_ros multi_cam.launch.py

\ **Read and distribute map with map server**\ 

.. code:: bash

   # In terminal 4
   source /opt/robot_devkit/robot_devkit_setup.bash
   ros2 run nav2_map_server map_server -f ~/map

\ **Start the navigation2 stack with the map**\ 

.. code:: bash

   # In terminal 5
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=${HOME}/map.yaml

Finally, please give an initial pose and goal within RVIZ2 to direct and navigate the turtlebot3 with the running map.

4. Known issues
---------------

* The accuracy of RealSense™ depth depends on the detection distance and the quality may not good enough to build a big map, it drifts the map building.

* Keep the RealSense™ parallel to the ground, or the tilt of the RealSense™ may influence the SLAM.
