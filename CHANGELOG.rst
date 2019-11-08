changelog for robot_devkit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2019-11-08)
------------------
* add ur_modern_driver(ros2_ur_description), handeye, criutils, baldor for ros2_grasp_library
* integrate robot-devkit-src which include rtmonitor and rdk_perf two performance measurement tools
* fix popup caused by libssl
* using apt-get to install ros-dashing-desktop instead of installing release-dashing- tarball.
* add a config pip tool to improve network connection.
* fix other installation issues.


0.2.0 (2019-09-12)
------------------
* rename robot_sdk to robot_devkit
* enable ros2 core binaries install from tarball (update to release-dashing-20190614)
* install librealsense by apt-get
* support package configuration for customization
* support logging for warn/error/info messages
* support tarball binaries release for remote target deployment
* integrate ros2_grasp_library
* update ros2_openvino_toolkit and integrate openvino_toolkit_p_2019.1.144
* update ros2_intel_realsense and add Intel RealSense T265 support
* support Realsense camera based slam and navigation with D435 and T265 cameras
* update ros2_object_analytics
* improve installation scripts and fix issues.


0.1.0 (2019-04-24)
------------------
* Initial version for robot_sdk release
