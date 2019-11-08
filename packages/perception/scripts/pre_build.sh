#!/bin/bash

echo "Prepare for [perception] build"

set -e

obj_dir=$1
echo "Prebuild for [$obj_dir]"


# openvino deps

# Copy label files (excute once) OpenVINO
sudo mkdir -p /opt/openvino_toolkit
sudo ln -sf "${obj_dir}"/src/intel/ros2_openvino_toolkit /opt/openvino_toolkit

sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP32
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/openvino_toolkit/models/object_detection/mobilenet-ssd/caffe/output/FP16
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/openvino_toolkit/models/emotions-recognition/output/FP32/Retail/object_attributes/emotions_recognition/0003/dldt/
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/openvino_toolkit/models/face_detection/output/FP32/Transportation/object_detection/face/pruned_mobilenet_reduced_ssd_shared_weights/dldt/
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001-fp16.labels /opt/openvino_toolkit/models/face_detection/output/FP16/Transportation/object_detection/face/pruned_mobilenet_reduced_ssd_shared_weights/dldt/
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels /opt/openvino_toolkit/models/segmentation/output/FP32
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/openvino_toolkit/models/vehicle-license-plate-detection/output/FP32/Security/object_detection/barrier/0106/dldt/

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib
export CPU_EXTENSION_LIB=/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libcpu_extension.so
export GFLAGS_LIB=/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libgflags_nothreads.a

# export openvino env
# shellcheck disable=SC1091
. /opt/intel/openvino/bin/setupvars.sh
# Reset OpenCR to old version. Ref to [https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/BINARY_VERSION_README.md]
export OpenCV_DIR=${obj_dir}/../third_party/opencv/build/

sudo apt-get install -y \
  liblz4-1 \
  liblz4-dev \
  python3-scipy \
  ros-dashing-eigen3-cmake-module
