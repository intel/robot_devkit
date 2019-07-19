SLAM with Lidar
===============

.. warning::

   To be updated base on turtlebot3 readiness for dashing release.

1. Overview
-----------

Robot SDK has integrated Cartographer for SLAM. For details, please
refer to `here`_.

2. Running the demo
-------------------

**Terminal 1: Run Micro-XRCE-DDS Agent for OpenCR**

.. code:: bash

   cd ~/turtlebot3 && MicroXRCEAgent serial /dev/ttyACM0

**Terminal 2: Run Micro-XRCE-DDS Agent for Lidar**

.. code:: bash

   cd ~/turtlebot3 && MicroXRCEAgent udp 2018

**Terminal 3: Run Lidar application**

.. code:: bash

   ~/turtlebot3/turtlebot3_lidar

**Terminal 4: Launch robot turtlebot3_node**

.. code:: bash

   source /opt/robot_devkit/robot_devkit_setup.bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_bringup robot.launch.py

**Terminal 5: Run teleoperation node**

.. code:: bash

   source /opt/robot_devkit/robot_devkit_setup.bash
   export TURTLEBOT3_MODEL=waffle
   ros2 run turtlebot3_teleop teleop_keyboard

**Terminal 6: Run cartographer**

.. code:: bash

   source /opt/robot_devkit/robot_devkit_setup.bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_cartographer cartographer.launch.py

**Terminal 7: Save the map**

.. code:: bash

   source /opt/robot_devkit/robot_devkit_setup.bash
   export TURTLEBOT3_MODEL=waffle
   ros2 run nav2_map_server map_saver -f ~/map

.. _here: https://github.com/ROBOTIS-GIT/cartographer_ros/tree/dashing