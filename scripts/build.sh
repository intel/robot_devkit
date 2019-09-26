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
# Build packages
# Arguments:
#  pkg: selected pacakges such as turtlebot3, perception...
#  build_option: --include-deps
#######################################
build_pkg()
{
  info "Build [$1] $2\n"

  local pkg=$1
  local build_options=$2
  local pkg_ws
  local src_dir
  local ros2_build_dir
  local ros2_install_dir
  pkg_ws=$(get_rdk_ws_dir)/${pkg}_ws
  src_dir="$pkg_ws"/src
  ros2_build_dir="$pkg_ws"/build
  ros2_install_dir="$pkg_ws"/install

  if [[ ! -d "${src_dir}" ]]; then
    warn "Source does not exist, please sync source first via command \"rdk.sh sync-src\""
    return 0
  fi

  # source ros2 core environment when build other packages
  local ros2_core
  ros2_core=/opt/ros/dashing/setup.bash
  info "Source ${ros2_core}\n"
  if [[ -f "${ros2_core}" ]] ; then
     . "${ros2_core}"
  else
    warn "${ros2_core} not exist"
  fi

  # Install dependences libraries for prebuild
  local prebuild_exec
  prebuild_exec=$(get_packages_dir)/${pkg}/scripts/pre_build.sh
  if [[ -f "${prebuild_exec}" ]] ; then
    info "Execute ${prebuild_exec} ${pkg_ws}"
    . "${prebuild_exec}" "${pkg_ws}"
  fi

  build_execute "${pkg}" "${ros2_build_dir}" "${ros2_install_dir}" "${src_dir}" "${build_options}"
}

#######################################
# Build execution
# Arguments:
#   pkg: selected pacakges such as turtlebot3, perception...
#   ros2_build_dir: output build/build folder
#   ros2_install_dir: output build/install folder
#   src_dir: source code folder
#   build_options: --args ARGS (colcon arguments)
#######################################
build_execute()
{
  local pkg=$1
  local ros2_build_dir=$2
  local ros2_install_dir=$3
  local src_dir=$4
  local build_options=$5

  info "Execute colcon build \
--build-base ${ros2_build_dir} \
--install-base ${ros2_install_dir} \
--base-paths ${src_dir} \
$build_options"

  execute colcon build \
--build-base "${ros2_build_dir}" \
--install-base "${ros2_install_dir}" \
--base-paths "${src_dir}" \
"$build_options"

  ok "Completed [$pkg] build.\n"
}


#######################################
# Common build entry
# Arguments:
#   build_options: --args ARGS
#######################################
build()
{
  info "Build packages ...\n"
  if [[ "$ROS_DISTRO" = "melodic" ]];then
    error "Detected system has already been sourced ROS environment, this will make RDK build fail. Suggest not to source any ROS setup.bash and open a new terminal for RDK build.\n"R
    exit 1
  fi

  local build_options="$*"

  read -r -a array <<< "$(get_packages_list)"
  for pkg in "${array[@]}"
  do
    build_pkg "${pkg}" "$build_options"
  done
}
unset CURRENT_DIR
