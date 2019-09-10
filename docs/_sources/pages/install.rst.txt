Installation
=====================

Robot support
-------------

-  `Turtlebot3 Waffle`_

Hardware Requirements
---------------------
-  An x86_64 computer with the 6th CPU or above running Ubuntu 18.04, an sample is `ADLink Neuron Board`_\ 

-  Intel® `Movidius™ Neural Compute Stick 2`_\

-  Intel® `RealSense™ D400 Series`_\

-  Intel® `RealSense™ T265`_\

Software Requirements
---------------------
-  We support Ubuntu Linux 18.04 (Bionic Beaver) on 64-bit. We do not
   support Mac OS X and Windows.

-  \ `Robot DevKit`_

Installation Steps
------------------

Automated installation
>>>>>>>>>>>>>>>>>>>>>>

.. code:: bash

   git clone https://github.com/intel/robot_devkit.git
   cd robot_devkit
   ./demo/rdk_install.sh

Manual installation
>>>>>>>>>>>>>>>>>>>

.. code:: bash

   # Build ROS2 packages
   ./rdk.sh config --default                    # select a target product, default option is turtlebot3
   ./rdk.sh install-deps                        # install system dependences
   ./rdk.sh sync-src  --force                   # sync source code for selected packages
   ./rdk.sh build                               # build all ROS2 packges
   ./rdk.sh install                             # install generated ros2 to /opt/robot_devkit folder.

   # Remove ROS2 packages
   ./rdk.sh clean                               # remove build folders.
   ./rdk.sh uninstall                           # delete rdk_ws folder and uninstall generated ros2 from /opt/robot_devkit folder.
                                                
.. note:: The build workspace overlay will be like this:

    .. code:: bash

        └── rdk_ws
            ├── turtlebot3_ws # Turtlebot3 packages from ./packages/turtlebot3/repos.
            │   ├── build
            │   ├── install
            │   └── src
            └── perception_ws # Intel® packages from ./packages/perception/repos
                ├── build
                ├── install
                └── src

Develop ROS2 packages
---------------------

Edit installed component (ros2_intel_realsense for example)
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

.. code:: bash

    source /opt/robot_devkit/robot_devkit_setup.bash
    cd ~/robot_devkit/rdk_ws/perception_ws
    colcon build --symlink-install --base-paths src/ros2_intel_realsense
    source install/local_setup.bash

Create a new project
>>>>>>>>>>>>>>>>>>>>

After Robot Devkit build environment installed, you could source setup.bash and develop your own ROS2 packages.

.. code:: bash

    source /opt/robot_devkit/robot_devkit_setup.bash
    mkdir ~/ros2_ws && cd ~/ros2_ws

Then you can follow the `ROS2 official document`_ to create and build your own packages.


.. _Turtlebot3 Waffle: http://www.robotis.us/turtlebot-3-waffle-pi/
.. _Movidius™ Neural Compute Stick 2: https://software.intel.com/en-us/neural-compute-stick
.. _RealSense™ D400 Series: https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html
.. _RealSense™ T265: https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html
.. _ADLink Neuron Board: https://neuron.adlinktech.com/en/
.. _Robot DevKit: https://github.com/intel/robot_devkit
.. _ROS2 official document: https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/
