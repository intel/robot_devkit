#!/bin/bash

# source environment
source /opt/robot_devkit/robot_devkit_setup.bash

# launch realsense backend
ros2 launch realsense_examples rs_camera.launch.py &

# launch openvino backend
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib
# export CPU_EXTENSION_LIB=/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libcpu_extension.so
# export GFLAGS_LIB=/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libgflags_nothreads.a
. /opt/intel/openvino/bin/setupvars.sh
ros2 launch dynamic_vino_sample ros2_openvino_oa.launch.py &

# launch object analytics
ros2 launch object_analytics_node object_analytics_sample.launch.py
