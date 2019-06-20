#!/bin/bash

echo "Prepare for [perception] build"

set -e

obj_dir=$1
echo "Prebuild for [$obj_dir]"


# openvino deps

# Copy label files (excute once) OpenVINO
sudo mkdir -p /opt/openvino_toolkit
sudo ln -sf "${obj_dir}"/src/intel/ros2_openvino_toolkit /opt/openvino_toolkit

sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/intel/openvino/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/intel/openvino/deployment_tools/intel_models/face-detection-adas-0001/FP32
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/intel/openvino/deployment_tools/intel_models/face-detection-adas-0001/FP16
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels ~/Downloads/models/mask_rcnn_inception_v2_coco_2018_01_28/output
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/intel/openvino/deployment_tools/tools/model_downloader/object_detection/common/mobilenet-ssd
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/intel/openvino/deployment_tools/tools/model_downloader/object_detection/common/mobilenet-ssd
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/intel/openvino/deployment_tools/tools/model_downloader/Security/object_detection/barrier/0106/dldt
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
  liblz4-dev
