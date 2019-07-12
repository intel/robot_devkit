Installation
=====================

Robot support
-------------

-  `Turtlebot3 Waffle`_

Hardware Requirements
---------------------
-  An x86_64 computer running Ubuntu 18.04 or `ADLink Neuron Board`_\ 

-  Intel® `Movidius™ Neural Compute Stick 2`_\

-  Intel® `RealSense™ D400 Series`_\

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
   ./rdk.sh build --cmake-args -DCMAKE_BUILD_TYPE=Release
                                                # build all ROS2 packges
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


.. _Turtlebot3 Waffle: http://www.robotis.us/turtlebot-3-waffle-pi/
.. _Movidius™ Neural Compute Stick 2: https://software.intel.com/en-us/neural-compute-stick
.. _RealSense™ D400 Series: https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html
.. _ADLink Neuron Board: https://neuron.adlinktech.com/en/
.. _Robot DevKit: https://github.com/intel/robot_devkit
