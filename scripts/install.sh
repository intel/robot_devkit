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
# Install the generated RDK to /opt/robot_devkit folder.
#######################################
install_rdk()
{
  info "Install rdk ...\n"
  local rdk_ws_dir
  local target_dir

  local core_build
  local core_install
  local device_install
  local modules_build
  local modules_install

  rdk_ws_dir=$(get_rdk_ws_dir)
  target_dir=$(get_install_dir)
  sudo mkdir -p "${target_dir}"

  # Install ROS2 core to /opt/robot_devkit
  ros2_core=${rdk_ws_dir}/third_party/ros2-linux
  target_ros2_dir=${target_dir}/ros2-linux

  info "Link ${ros2_core} to ${target_ros2_dir}"
  if [[ -d ${ros2_core} ]]; then
    sudo rm -rf "${target_ros2_dir}"
    sudo ln -sf "${ros2_core}" "${target_ros2_dir}"

    # Generate setup bash file
    sudo bash -c "cat << EOF > ${target_dir}/robot_devkit_setup.bash
#!/bin/bash
. ${target_ros2_dir}/local_setup.bash
EOF"
  else
    warn "Not found: ${ros2_core}"
    exit
  fi


  # Install packages to /opt/robot_devkit
  local build_pkg_dir

  read -r -a array <<< "$(get_packages_list)"
  for pkg in "${array[@]}"
  do

    build_pkg_dir=${rdk_ws_dir}/${pkg}_ws/install
    target_pkg_dir=${target_dir}/${pkg}

    info "Link ${build_pkg_dir} to ${target_pkg_dir}"

    if [[ -d ${build_pkg_dir} ]]; then
      sudo rm -rf "$target_pkg_dir"
      sudo ln -sf "${build_pkg_dir}" "$target_pkg_dir"
      # Append setup bash file
      echo ". ${target_pkg_dir}/local_setup.bash" | sudo tee -a ${target_dir}/robot_devkit_setup.bash > /dev/null
    else
      warn "Not found: ${build_pkg_dir}"
    fi
  done


  info "Generated setup bash to $target_dir/robot_devkit_setup.bash"
  ok "Successful install ROS package on ${target_dir}.\n"
}

#######################################
# Clean the output folder
#######################################
uninstall_rdk()
{
  info "Uninstall all build files...\n"
  local install_dir
  local rdk_ws
  local log_dir

  install_dir=$(get_install_dir)
  rdk_ws=$(get_rdk_ws_dir)
  log_dir=$(get_root_dir)/log

  info "Delete: ${install_dir}"
  sudo rm -fr "${install_dir}"

  info "Delete: ${rdk_ws}"
  sudo rm -fr "${rdk_ws}"

  info "Delete: ${log_dir}"
  sudo rm -fr "${log_dir}"

  ok "Successfully uninstalled all build and installed files.\n"
}

unset CURRENT_DIR
