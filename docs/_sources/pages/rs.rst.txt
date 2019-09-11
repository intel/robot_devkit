Intel® RealSense™
=================

1. Overview
-----------

These are packages for using Intel® RealSense™ cameras (D400 series and T265)
with ROS2.

2. Running the demo
-------------------

\ **Start the camera node**\ 



To start the camera node in ROS2, plug in the camera, then type the
following command:

.. code:: bash

   source /opt/robot_devkit/robot_devkit_setup.bash
   # To launch with "ros2 run"
   ros2 run realsense_node realsense_node
   # Or use "ros2 launch"
   ros2 launch realsense_examples rs_camera.launch.py

This will stream all camera sensors and publish on the appropriate ROS2
topics. PointCloud2 is enabled by default.

\ **View camera data**\ 

.. code:: bash

   # in Terminal #1 launch realsense_ros2_camera
   source /opt/robot_devkit/robot_devkit_setup.bash
   ros2 launch realsense_examples rs_camera.launch.py
   # in terminal #2 launch rviz2
   source /opt/robot_devkit/robot_devkit_setup.bash
   rviz2

This will launch `RVIZ2`_ and display the five streams: color, depth,
infra1, infra2, pointcloud.

**Realsense can also support SLAM and navigation, see** \ `here`_\ **.**

.. image:: ../imgs/ros2_rviz.png

3. Key Interfaces
-----------------

- `/camera/depth/image_rect_raw`_

- `/camera/aligned_depth_to_color/image_raw`_

- `/camera/color/image_raw`_

- `/camera/infra1/image_rect_raw`_

- `/camera/infra2/image_rect_raw`_

- `/camera/pointcloud`_

4. Known Issues
---------------

- This ROS2 node does not currently provide any dynamic reconfigure support for camera properties/presets.

- We support Ubuntu Linux 18.04 (Bionic Beaver) on 64-bit, but not support Mac OS X and Windows yet.

5. ToDo
-------


.. _RVIZ2: http://wiki.ros.org/rviz
.. _here: https://yechun1.github.io/robot_devkit/rs_for_slam_nav
.. _/camera/depth/image_rect_raw: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg
.. _/camera/aligned_depth_to_color/image_raw: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg
.. _/camera/color/image_raw: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg
.. _/camera/infra1/image_rect_raw: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg
.. _/camera/infra2/image_rect_raw: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg
.. _/camera/pointcloud: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg