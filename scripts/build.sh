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
build_group()
{
  info "\nBuild [$1] $2\n"

  local group=$1
  local build_options=$2
  local group_ws
  local src_dir
  local ros2_build_dir
  local ros2_install_dir
  group_ws=$(get_current_sdk_ws)/${group}_ws
  src_dir="$group_ws"/src
  ros2_build_dir="$group_ws"/build
  ros2_install_dir="$group_ws"/install

  if [[ ! -d "${src_dir}" ]]; then
    error "Source does not exist, please sync source first via command \"robot_sdk.sh sync-src\""
    exit 1
  fi

  if [[ "$ROS_DISTRO" = "melodic" ]];then
    warning "\n        Detected system has already been sourced ROS environment, this will make SDK build fail. Suggest not to source any ROS setup.bash and open a new terminal for SDK build.\n"
    exit 1
  fi

  # Install dependences from sources
  if [[ "${build_options}" =~ "--include-deps" ]]; then
    local build_deps_exec
    build_deps_exec=$(get_current_product_dir)/${group}/scripts/build_deps.sh
    info "\nBuild deps\nexecute ${build_deps_exec}\n"
    if [[ -f "${build_deps_exec}" ]] ; then
      execute "${build_deps_exec}" "$(get_current_product_deps_dir)"
    else
      info "\n${build_deps_exec} does not found, skip\n"
    fi
  fi

  # Install dependences libraries for prebuild
  local prebuild_exec
  prebuild_exec=$(get_current_product_dir)/${group}/scripts/pre_build.sh
  if [[ -f "${prebuild_exec}" ]] ; then
    info "\nInstall libs\n. ${prebuild_exec}\n"
    . "${prebuild_exec}" "$(get_current_sdk_ws)"
  fi

  # source ros2 core environment when build other packages
  if [[ "$group" == "device" ]]; then
     info "\nSource $(get_current_sdk_ws)/core_ws/install/local_setup.bash\n"
     . "$(get_current_sdk_ws)"/core_ws/install/local_setup.bash
  fi

  # source ros2 device environment when build modules packages
  if [[ "$group" == "modules" ]]; then
     info "\nSource $(get_current_sdk_ws)/core_ws/install/local_setup.bash\n"
     . "$(get_current_sdk_ws)"/core_ws/install/local_setup.bash

     info "\nSource $(get_current_sdk_ws)/device_ws/install/local_setup.bash\n"
     . "$(get_current_sdk_ws)"/device_ws/install/local_setup.bash
  fi

  build_execute "${group}" "${ros2_build_dir}" "${ros2_install_dir}" "${src_dir}" "${build_options}"
}


#######################################
# Build all packages
# Arguments:
#   None
#######################################
build_all()
{
  info "\nBuild [all]\n"
  build_group core "${build_options}"
  build_group device "${build_options}"
  build_group modules "${build_options}"
}


#######################################
# Build execution
# Arguments:
#   group: core or device or modules
#   ros2_build_dir: output build/build folder
#   ros2_install_dir: output build/install folder
#   src_dir: source code folder
#   build_options: --include-deps --args ARGS (colcon arguments)
#######################################
build_execute()
{
  info "\nBuild packages\n"

  local group=$1
  local ros2_build_dir=$2
  local ros2_install_dir=$3
  local src_dir=$4
  local build_options=$5

  echo "execute colcon build \
    --symlink-install \
    --build-base ${ros2_build_dir} \
    --install-base ${ros2_install_dir} \
    --base-paths ${src_dir} \
    $build_options"
  execute colcon build \
    --symlink-install \
    --build-base "${ros2_build_dir}" \
    --install-base "${ros2_install_dir}" \
    --base-paths "${src_dir}" \
    "$build_options"

  info "\nCompleted [$group] build.\n"
}


#######################################
# Common build entry
# Arguments:
#   group: core or device or modules
#   build_options: --args ARGS
#######################################
build()
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

  shift 1
  local build_options="$*"

  case $group in
    core | device | modules)
      build_group "${group}" "$build_options"
      ;;
    all)
      build_all "$build_options"
      ;;
    *)
      error "\nThe group should be \"core\" or \"device\" or \"modules\" or \"all\" .\n"
      exit 1
      ;;
  esac
}
unset CURRENT_DIR
