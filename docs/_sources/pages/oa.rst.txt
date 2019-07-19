Object Analytics
================

1. Overview
-----------

Object Analytics (OA) is ROS2 wrapper for realtime object detection,
localization and tracking. These packages aim to provide real-time
object analyses over RGB-D camera inputs, enabling ROS developers to
easily create amazing robotics advanced features, like intelligent
collision avoidance and semantic SLAM. It consumes
`sensor_msgs::PointClould2`_ data delivered by RGB-D camera, publishing
topics on `object detection`_, `object tracking`_, and `object
localization`_ in 3D camera coordination system.

OA keeps integrating with various “state-of-the-art” algorithms. By
default, backend of object detection is Intel® movidius ncs2.

2. Running the demo
-------------------

**Object Analytics with OpenVINO toolkit**

.. code:: bash

   # Start OA demo with OpenVINO
   source /opt/robot_devkit/robot_devkit_setup.bash
   ros2 launch object_analytics_node object_analytics_with_openvino_sdk.launch.py

**OA demo video:**

.. image:: ../imgs/oa_demo.gif

**Customize launch**


By default, object analytics will launch both tracking and localization
features, but either tracking or localization or both can be dropped.
Detailed please refer comments embedded in launch file.

3. Interfaces
-------------

Subscribed topics
>>>>>>>>>>>>>>>>>

* Object Detection:

    /object_analytics/detected_objects (`object_msgs::msg::ObjectsInBoxes`_)

Published topics
>>>>>>>>>>>>>>>>

* Image:

    /object_analytics/rgb (`sensor_msgs::msg::Image`_)

* Pointcloud:

    /object_analytics/pointcloud (`sensor_msgs::msg::PointCloud2`_)

* Localization:

    /object_analytics/localization (`object_analytics_msgs::msg::ObjectsInBoxes3D`_)

* Tracking:

    /object_analytics/tracking (`object_analytics_msgs::msg::TrackedObjects`_)

* Movement:

    /object_analytics/movement (`object_analytics_msgs::msg::MovingObjectsInFrame`_)

4. Known issues
---------------

–

5. ToDo
-------

–

.. _`sensor_msgs::PointClould2`: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
.. _object detection: https://github.com/intel/ros2_object_msgs
.. _object tracking: https://github.com/intel/ros2_object_analytics/tree/master/object_analytics_msgs
.. _object localization: https://github.com/intel/ros2_object_analytics/tree/devel/object_analytics_msgs
.. _`object_msgs::msg::ObjectsInBoxes`: https://github.com/intel/ros2_object_msgs/blob/master/msg/ObjectsInBoxes.msg
.. _`sensor_msgs::msg::Image`: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Image.msg
.. _`sensor_msgs::msg::PointCloud2`: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg
.. _`object_analytics_msgs::msg::ObjectsInBoxes3D`: https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/ObjectsInBoxes3D.msg
.. _`object_analytics_msgs::msg::TrackedObjects`: https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/TrackedObjects.msg
.. _`object_analytics_msgs::msg::MovingObjectsInFrame`: https://github.com/intel/ros2_object_analytics/blob/master/object_analytics_msgs/msg/MovingObjectsInFrame.msg
