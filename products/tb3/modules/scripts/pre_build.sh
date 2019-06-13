#!/bin/bash

echo "Prepare for ROS2 build"

set -e

sdk_ws=$1
echo "$sdk_ws"

# Copy label files (excute once) OpenVINO
sudo mkdir -p /opt/openvino_toolkit
sudo ln -sf "$sdk_ws"/modules_ws/src/intel/ros2_openvino_toolkit /opt/openvino_toolkit

sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/emotions-recognition/FP32/emotions-recognition-retail-0003.labels /opt/intel/openvino/deployment_tools/intel_models/emotions-recognition-retail-0003/FP32
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/intel/openvino/deployment_tools/intel_models/face-detection-adas-0001/FP32
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/face_detection/face-detection-adas-0001.labels /opt/intel/openvino/deployment_tools/intel_models/face-detection-adas-0001/FP16
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_segmentation/frozen_inference_graph.labels ~/Downloads/models/mask_rcnn_inception_v2_coco_2018_01_28/output
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/intel/openvino/deployment_tools/tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP32
 sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/mobilenet-ssd.labels /opt/intel/openvino/deployment_tools/tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP16
sudo cp /opt/openvino_toolkit/ros2_openvino_toolkit/data/labels/object_detection/vehicle-license-plate-detection-barrier-0106.labels /opt/intel/openvino/deployment_tools/tools/model_downloader/Security/object_detection/barrier/0106/dldt

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib
export CPU_EXTENSION_LIB=/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libcpu_extension.so
export GFLAGS_LIB=/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libgflags_nothreads.a

echo "export openvino env"
# shellcheck disable=SC1091
. /opt/intel/openvino/bin/setupvars.sh
# Reset OpenCR to old version. Ref to [https://github.com/intel/ros2_openvino_toolkit/blob/devel/doc/BINARY_VERSION_README.md]
export OpenCV_DIR=${sdk_ws}/third_party/opencv/opencv/build/

sudo apt-get install -y \
  liblz4-1 \
  liblz4-dev

# Configure the Neural Compute Stick USB Driver
cd ~/Downloads
cat <<EOF > 97-usbboot.rules
SUBSYSTEM=="usb", ATTRS{idProduct}=="2150", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="usb", ATTRS{idProduct}=="2485", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
SUBSYSTEM=="usb", ATTRS{idProduct}=="f63b", ATTRS{idVendor}=="03e7", GROUP="users", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
sudo cp 97-usbboot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo ldconfig
rm 97-usbboot.rules

# Delete sudo time invalid operation
echo "Delete sudo time invalid operation"
sudo rm -rf /etc/sudoers.d/timeout
