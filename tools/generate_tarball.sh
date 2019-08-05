#!/bin/bash

set -e

CURRENT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

RDK_TOP=${CURRENT_DIR}
DEPS_DIR=${CURRENT_DIR}/rdk_ws/third_party
RELEASE_DIR=${CURRENT_DIR}/rdk_ws/release/rdk_release
ROOTFS=${RELEASE_DIR}/rootfs

add_apt_key()
{
  mkdir ${RELEASE_DIR}/keys -p
  cp $RDK_TOP/packages/perception/deps/20-C8B3A55A6F3EFCDE ${RELEASE_DIR}/keys
  cp $RDK_TOP/packages/common/deps/10-F42ED6FBAB17C654 ${RELEASE_DIR}/keys
}


install_opencv()
{
  cd $DEPS_DIR/opencv/build
  sudo make install DESTDIR=$ROOTFS
}

install_opencl()
{
  DEPS=/data/proj/rdk/robot_devkit/rdk_ws/release/deps
  mkdir ${RELEASE_DIR}/deps -p
  cp -rf $DEPS_DIR/opencl ${RELEASE_DIR}/deps
}

install_openvino()
{
  sudo rm -rf $ROOTFS/opt/intel
  sudo mkdir $ROOTFS/opt/intel -p
  sudo cp -rf /opt/intel $ROOTFS/opt/

  sudo mkdir $ROOTFS/opt/models/mask_rcnn_inception_v2_coco_2018_01_28 -p
  sudo cp -rf /opt/models/mask_rcnn_inception_v2_coco_2018_01_28/* $ROOTFS/opt/models/mask_rcnn_inception_v2_coco_2018_01_28

  sudo mkdir $ROOTFS/etc/udev/rules.d -p
  sudo cp -rf /etc/udev/rules.d/97-usbboot.rules $ROOTFS/etc/udev/rules.d/97-usbboot.rules
}

install_turtlebot3()
{
  sudo mkdir $ROOTFS/etc/udev/rules.d -p
  sudo cp -rf /etc/udev/rules.d/99-turtlebot3-cdc.rules $ROOTFS/etc/udev/rules.d/99-turtlebot3-cdc.rules
  sudo cp -rf /etc/udev/rules.d/99-opencr-cdc.rules $ROOTFS/etc/udev/rules.d/99-opencr-cdc.rules

  cd $DEPS_DIR/opencr/Micro-XRCE-DDS-Agent/build
  sudo make install DESTDIR=$ROOTFS

  cd $DEPS_DIR/opencr/Micro-XRCE-DDS-Client/build
  sudo make install DESTDIR=$ROOTFS
}


install_ros2_pkg()
{
  echo $ROOTFS
  sudo mkdir -p $ROOTFS/opt/robot_devkit/
  sudo cp -rf /opt/robot_devkit/* $ROOTFS/opt/robot_devkit
}

generate_tarball()
{
  cp $RDK_TOP/demo/rdk_target_install.sh $RELEASE_DIR/rdk_target_install.sh
  cp $RDK_TOP/demo/run_listener.sh $RELEASE_DIR/run_listener.sh
  cp $RDK_TOP/demo/run_talker.sh $RELEASE_DIR/run_talker.sh

  cd $RELEASE_DIR/..
  # TODO: fix tar: Exiting with failure status due to Permission denied"
  sudo chmod 644 $ROOTFS/opt/intel/computer_vision_sdk_2018.5.455/openvino_toolkit_uninstaller/uninstall/images/splash.png
  sudo chmod 644 $ROOTFS/opt/intel/.scripts/*

  version=$(date +'%Y%m%d%H%M')
  tar czvf rdk_release_${version}.tar.gz rdk_release
  md5sum rdk_release_${version}.tar.gz > rdk_release_${version}.tar.gz.md5
  echo "Succeed to generate tarball to $RELEASE_DIR/../rdk_release_${version}.tar.gz"
}

function main()
{
  if [ ! -f /opt/robot_devkit/robot_devkit_setup.bash ]; then
    echo "Error: Not found installed RDK packages, please build and install RDK first"
    exit
  fi

  echo "Delete $RELEASE_DIR and re-create packages"
  sudo rm -rf $RELEASE_DIR
  add_apt_key
  install_opencv
  install_opencl
  install_openvino
  install_turtlebot3
  install_ros2_pkg
  generate_tarball
}


main "$@"
