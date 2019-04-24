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
# Build group
# Arguments:
#  group: core or device or modules
#  build_option: --include-deps
#######################################
clean_group()
{
  info "\nClean [$1] \n"

  local group=$1
  local group_ws
  local ros2_build_dir
  local ros2_install_dir
  group_ws=$(get_current_sdk_ws)/${group}_ws
  ros2_build_dir="$group_ws"/build
  ros2_install_dir="$group_ws"/install

  echo "rm -rf $ros2_build_dir"
  echo "rm -rf $ros2_install_dir"
  rm -rf "$ros2_build_dir"
  rm -rf "$ros2_install_dir"

}


#######################################
# Build all packages
# Arguments:
#   None
#######################################
clean_all()
{
  info "\nclean [all]\n"
  clean_group core
  clean_group device
  clean_group modules
}


#######################################
# Common build entry
# Arguments:
#   group: core or device or modules
#######################################
clean()
{
  local group=$1

  if [[ -z "$(get_current_product)" ]]; then
    error "Please select product via command \"robot_sdk.sh product\"."
    exit 1
  fi

  if [[ "${group}" != "core" ]] && [[ "${group}" != "device" ]] && [[ "${group}" != "modules" ]] && [[ "${group}" != "all" ]]; then
    error "\nThe group should be \"core\" or \"device\" or \"modules\" or \"all\".\n"
    exit 1
  fi

  case $group in
    core | device | modules)
      clean_group "${group}"
      ;;
    all)
      clean_all
      ;;
    *)
      error "\nThe group should be \"core\" or \"device\" or \"modules\" or \"all\" .\n"
      exit 1
      ;;
  esac
}
unset CURRENT_DIR
