SLAM with Lidar
===============

1. Overview
-----------

Robot SDK has integrated Cartographer for SLAM. For details, please
refer to `here`_.

2. Running the demo
-------------------

**Terminal 1: Launch the robot**

.. code:: bash

   echo export TURTLEBOT3_MODEL=waffle >> ~/.bashrc
   echo ROS_DOMAIN_ID=30 >> ~/.bashrc
   source /opt/robot_devkit/robot_devkit_setup.bash
   ros2 launch turtlebot3_bringup robot.launch.py

.. note ::
    waffle are real TB3, if you use another robot, you may change this

**Terminal 2: Launch teleoperation node**

.. code:: bash

   source /opt/robot_devkit/robot_devkit_setup.bash
   export TURTLEBOT3_MODEL=waffle
   ros2 run turtlebot3_teleop teleop_keyboard

**Terminal 3: Run cartographer**

.. code:: bash

   source /opt/robot_devkit/robot_devkit_setup.bash
   source /opt/ros/dashing/local_setup.bash
   ros2 launch turtlebot3_cartographer cartographer.launch.py

**Terminal 4: Save the map**

.. code:: bash

   source /opt/robot_devkit/robot_devkit_setup.bash
   source /opt/ros/dashing/local_setup.bash
   ros2 run nav2_map_server map_saver -f ~/map

.. _here: https://github.com/ROBOTIS-GIT/cartographer_ros/tree/dashing