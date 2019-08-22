#!/bin/bash
################################################################################
#
# Copyright (c) 2017 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
################################################################################

CURRENT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

set -e

. "${CURRENT_DIR}"/product.sh

#######################################
# Print package version
#######################################
print_pkg_version()
{

  local rdk_ws
  local packages=("perception_ws/src/intel/ros2_intel_realsense"
    "perception_ws/src/intel/ros2_object_analytics"
    "perception_ws/src/intel/ros2_openvino_toolkit"
    "turtlebot3_ws/src/cartographer/cartographer"
    "turtlebot3_ws/src/navigation2/navigation2"
    "turtlebot3_ws/src/turtlebot3/turtlebot3")

  rdk_ws=$(get_rdk_ws_dir)

  if [[ ! -d "$rdk_ws/perception_ws" && ! -d "$rdk_ws/turtlebot3_ws" ]]; then
    return 1
  fi

  echo "package:"
  for pkg in "${packages[@]}"
  do
    if [[ ! -d "$rdk_ws/$pkg" ]]; then
      continue
    fi
    cd "$rdk_ws"/"$pkg"
    commit=$(git rev-parse --short HEAD)
    branch=$(git rev-parse HEAD |xargs git branch -r --contains|sed 's/origin\///g'|sed -n '1p')
    pkg=${pkg##*/} 
    echo "    $pkg:${branch}-${commit}" 
  done

}

#######################################
# Print thirdparty version
#######################################
print_thirdparty_version()
{
  local rdk_ws
  local ros2_version
  local librealsense_version
  local opencv_version
  local openvino_version

  rdk_ws=$(get_rdk_ws_dir)
  if [[ ! -d "$rdk_ws/third_party" ]]; then
    return 1
  fi

  echo "third_party:"
  # ros2-linux
  ros2_version=$(ls "$rdk_ws"/third_party/ros2-linux_download/)
  echo "    ros2-linux: $ros2_version"

  # librealsense
  librealsense_version=$(dpkg -l |grep librealsense2-dev|awk '{print $3}')
  echo "    librealsense: $librealsense_version"

  #opencv
  cd "$rdk_ws"/third_party/opencv
  opencv_version=$(git tag)
  echo "    opencv: ${opencv_version}"

  #openvino
  openvino_version=$(ls "$rdk_ws"/third_party/openvino_binart|grep openvino|sed -n 1p)
  echo "    openvino: ${openvino_version}"
}

print_sdk_version()
{
  commit=$(git rev-parse --short HEAD)
  branch=$(git rev-parse --abbrev-ref HEAD)
  echo "robot_devkit(rdk) version ${branch}-${commit}"
}

######################################
# Print version of this repository
######################################
rdk_version() {
  print_sdk_version
  print_pkg_version
  print_thirdparty_version
}

unset CURRENT_DIR
