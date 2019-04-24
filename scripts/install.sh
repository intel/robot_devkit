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
# Generate robot_sdk_setup.bash file
#######################################
generate_setup_bash()
{
  echo "bash"
}

#######################################
# Install the generated SDK to /opt/robot_sdk folder.
#######################################
install_sdk()
{
  local source_dir
  local target_dir

  local core_build
  local core_install
  local device_build
  local device_install
  local modules_build
  local modules_install

  source_dir=$(get_current_sdk_ws)
  target_dir=$(get_install_dir)

  core_build=$source_dir/core_ws/install
  device_build=$source_dir/device_ws/install
  modules_build=$source_dir/modules_ws/install

  core_install=$target_dir/core
  device_install=$target_dir/device
  modules_install=$target_dir/modules

  sudo mkdir -p "${target_dir}"

  info "Install [core] to ${target_dir}"
  if [[ -d ${core_build} ]]; then
    sudo rm -rf "$core_install"
    sudo ln -svf "${core_build}" "$core_install"
  else
    info "Not found: ${core_build}"
  fi

  info "Install [device] to ${target_dir}"
  if [[ -d ${device_build} ]]; then
    sudo rm -rf "$device_install"
    sudo ln -svf "${device_build}" "$device_install"
  else
    info "Not found: ${device_build}"
  fi

  info "Install [modules] to ${target_dir}"
  if [[ -d ${modules_build} ]]; then
    sudo rm -rf "$modules_install"
    sudo ln -svf "${modules_build}" "$modules_install"
  else
    info "Not found: ${modules_build}"
  fi

  # Generate setup bash file
  info "\nGenerate setup bash to $target_dir/robot_sdk_setup.bash\n"
  sudo bash -c "cat << EOF > $target_dir/robot_sdk_setup.bash
#!/bin/bash
. $core_install/local_setup.bash
. $device_install/local_setup.bash
. $modules_install/local_setup.bash
EOF"

  ok "\nSuccessful install ROS package on ${target_dir}"
}

#######################################
# Clean the output folder
#######################################
uninstall_sdk()
{
  local install_dir
  local sdk_ws
  local log_dir

  install_dir=$(get_install_dir)
  sdk_ws=$(get_current_sdk_ws)
  log_dir=$(get_root_dir)/log

  info "DELETE: ${install_dir}"
  info "DELETE: ${sdk_ws}"
  info "DELETE: ${log_dir}"
  info "unlink: products/current"

  sudo rm -fr "${install_dir}"
  sudo rm -fr "${sdk_ws}"
  sudo rm -fr "${log_dir}"
  unlink products/current
  ok "\nSuccessful uninstall all build and installed files"
}

unset CURRENT_DIR
