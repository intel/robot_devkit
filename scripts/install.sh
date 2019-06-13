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
# Generate robot_devkit_setup.bash file
#######################################
generate_setup_bash()
{
  echo "bash"
}

#######################################
# Install the generated SDK to /opt/robot_devkit folder.
#######################################
install_rdk()
{
  info "\nInstall rdk ...\n"
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

  info "Install [core] to ${target_dir}"
  if [[ -d ${core_build} ]]; then
    sudo rm -rf "$core_install"
    sudo ln -svf "${core_build}" "$core_install"
  else
    info "Not found: ${core_build}"
  fi


  # Install packages to /opt/robot_devkit
  array[1]="Turtlebot3"
  array[2]="perception"

  local build_pkg_dir

  for pkg in "${array[@]}"
  do

    build_pkg_dir=${rdk_ws_dir}/${pkg}_ws/install
    target_pkg_dir=${target_dir}/${pkg}

    info "Install [$build_pkg_dir] to [${target_pkg_dir}]"

    if [[ -d ${build_pkg_dir} ]]; then
      sudo rm -rf "$target_pkg_dir"
      sudo ln -sf "${build_pkg_dir}" "$target_pkg_dir"
      # Append setup bash file
      echo ". ${target_pkg_dir}/local_setup.bash" | sudo tee -a ${target_dir}/robot_devkit_setup.bash > /dev/null
    else
      info "Not found: ${build_pkg_dir}"
    fi
  done


  info "\nGenerated setup bash to $target_dir/robot_devkit_setup.bash"
  ok "\nSuccessful install ROS package on ${target_dir}"
}

#######################################
# Clean the output folder
#######################################
uninstall_rdk()
{
  local install_dir
  local rdk_ws
  local log_dir

  install_dir=$(get_install_dir)
  rdk_ws=$(get_rdk_ws_dir)
  log_dir=$(get_root_dir)/log

  info "DELETE: ${install_dir}"
  info "DELETE: ${rdk_ws}"
  info "DELETE: ${log_dir}"
  info "unlink: products/current"

  sudo rm -fr "${install_dir}"
  sudo rm -fr "${rdk_ws}"
  sudo rm -fr "${log_dir}"
  ok "\nSuccessful uninstall all build and installed files"
}

unset CURRENT_DIR
