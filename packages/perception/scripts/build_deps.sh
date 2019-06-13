#!/bin/bash

set -e

#
# Build opencv
#
build_opencv()
{
  local src_path=$1

  mkdir -p "${src_path}"/opencv
  cd "${src_path}"/opencv
  if [[ ! -d opencv ]]
  then
    git clone https://github.com/opencv/opencv.git -b 3.4.2
  fi
  if [[ ! -d opencv_contrib ]]
  then
    git clone https://github.com/opencv/opencv_contrib.git -b 3.4.2
  fi
  cd opencv
  mkdir -p build
  cd build
  cmake -DOPENCV_EXTRA_MODULES_PATH="${src_path}"/opencv/opencv_contrib/modules -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_opencv_cnn_3dobj=OFF ..
  make -j8
  sudo make install
  sudo ldconfig
}

#
# build openvino binary 
#
build_openvino_binary()
{
  echo "Build openvino binary..."
  local src_path=$1
  mkdir -p "${src_path}"/Openvino/openvino_binart
  cd "${src_path}"/Openvino/openvino_binart
  wget -c http://registrationcenter-download.intel.com/akdlm/irc_nas/15078/l_openvino_toolkit_p_2018.5.455.tgz
  tar -xvf l_openvino_toolkit_p_2018.5.455.tgz
  cd l_openvino_toolkit_p_2018.5.455
  sudo  ./install_cv_sdk_dependencies.sh
  sed -i 's/ACCEPT_EULA=decline/ACCEPT_EULA=accept/g' silent.cfg
  if [ ! -d /opt/intel/computer_vision_sdk ];then
    sudo ./install.sh --silent silent.cfg
  else
    echo "WARNING: Destination directory already exists."
  fi
  cd /opt/intel/computer_vision_sdk/install_dependencies
  sudo ./install_NEO_OCL_driver.sh

  # shellcheck disable=SC1091
  . /opt/intel/computer_vision_sdk/bin/setupvars.sh
  mkdir -p "${src_path}"/Openvino/openvino_binart_example
  cd "${src_path}"/Openvino/openvino_binart_example
  mkdir -p build && cd build
  cmake /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/ && make && cd ..
  sudo /bin/cp -rf build /opt/intel/computer_vision_sdk/deployment_tools/inference_engine/samples/

  cd /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/install_prerequisites
  sudo ./install_prerequisites.sh
  mkdir -p ~/Downloads/models
  cd ~/Downloads/models
  wget http://download.tensorflow.org/models/object_detection/mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
  tar -zxvf mask_rcnn_inception_v2_coco_2018_01_28.tar.gz
  cd mask_rcnn_inception_v2_coco_2018_01_28
  python3 /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/mo_tf.py --input_model frozen_inference_graph.pb --tensorflow_use_custom_operations_config /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/extensions/front/tf/mask_rcnn_support.json --tensorflow_object_detection_api_pipeline_config pipeline.config --reverse_input_channels --output_dir ./output/
  sudo mkdir -p /opt/models
  sudo ln -sf ~/Downloads/models/mask_rcnn_inception_v2_coco_2018_01_28 /opt/models/
  cd /opt/intel/computer_vision_sdk/deployment_tools/model_downloader
  sudo python3 ./downloader.py --name mobilenet-ssd
  #FP32 precision model
  sudo python3 /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/mo.py --input_model /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/mobilenet-ssd.caffemodel --output_dir /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP32 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
  #FP16 precision model
  sudo python3 /opt/intel/computer_vision_sdk/deployment_tools/model_optimizer/mo.py --input_model /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/mobilenet-ssd.caffemodel --output_dir /opt/intel/computer_vision_sdk/deployment_tools/model_downloader/object_detection/common/mobilenet-ssd/caffe/output/FP16 --data_type=FP16 --mean_values [127.5,127.5,127.5] --scale_values [127.5]
  echo "Install OPENVINO SUCCESS!"

}

# Build librealsense
#
build_librealsense()
{
  echo "Build librealsense..."
  local src_path=$1
  cd "${src_path}"
  if [[ ! -d librealsense ]]
  then
    git clone https://github.com/IntelRealSense/librealsense.git -b v2.17.1
  fi
  sudo apt-get install -y \
    libudev-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev
  cd librealsense
  mkdir -p build
  cd build
  cmake ../
  make -j8
  sudo make uninstall
  sudo make install
  cd ../
  sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && udevadm trigger
}

#
# opencl
#
build_opencl()
{
  echo "Build opencl..."
  local src_path=$1
  cd "${src_path}"
  mkdir -p OpenCL
  cd OpenCL
  mkdir -p intel-opencl
  wget -c http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB5.0_linux64.zip
  unzip -o SRB5.0_linux64.zip
  sudo usermod -a -G video "$LOGNAME"
  # shellcheck disable=SC2011
  ls ./*.tar.xz | xargs -n1 tar -C intel-opencl/ -xvf
  sudo cp -rf intel-opencl/* /
  sudo ldconfig
}

#
# Main entry to build deps
#
main()
{
  deps_dir=${1}

  if [ ! -d "${deps_dir}" ]; then
    mkdir "${deps_dir}" -p
    echo "No such directory, create \"${deps_dir}\""
  fi

  build_opencl "${deps_dir}"
  build_openvino_binary "${deps_dir}"
  build_opencv "${deps_dir}"
  build_librealsense "${deps_dir}"
}

main "$@"
