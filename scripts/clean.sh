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

. "$CURRENT_DIR"/product.sh

#######################################
# Clean build files
# Arguments:
#  pkg: selected packages
#######################################
do_clean()
{
  info "Clean [$1]"

  local pkg_ws=${1}_ws
  rdk_ws=$(get_rdk_ws_dir)/${pkg_ws}

  if [[ -d "${rdk_ws}" ]]; then
    local ros2_build_dir
    local ros2_install_dir
    ros2_build_dir="$rdk_ws"/build
    ros2_install_dir="$rdk_ws"/install

    info "Execute rm -rf $ros2_build_dir"
    info "Execute rm -rf $ros2_install_dir"
    rm -rf "$ros2_build_dir"
    rm -rf "$ros2_install_dir"
  else
    warn "${rdk_ws} not exist, nothing to remove, skip"
  fi
}

#######################################
# Common build entry
# Arguments:
#   group: core or device or modules
#######################################
clean()
{
  info "Clean build and install under $(get_rdk_ws_dir) ...\n"
  if [[ ! -d "$(get_rdk_ws_dir)" ]]; then
    error "$(get_rdk_ws_dir) does not exist."
    exit 1
  fi

  read -r -a array <<< "$(get_packages_list)"
  for pkg in "${array[@]}"
  do
    do_clean "${pkg}"
  done

}
unset CURRENT_DIR
