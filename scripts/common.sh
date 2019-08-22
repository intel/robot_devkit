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
ROBOT_DEVKIT_ROOT_DIR=$(readlink -f "${CURRENT_DIR}/../")
ROBOT_DEVKIT_WS_DIR=${ROBOT_DEVKIT_ROOT_DIR}/rdk_ws

# Color definitions for foreground
FG_NONE='\033[0m'
FG_BOLD='\033[1m'
FG_BLACK='\033[0;30m'
FG_GRAY="\033[1;30m"
FG_RED='\033[0;31m'
FG_GREEN='\033[0;32m'
FG_YELLOW='\033[0;33m'
FG_BLUE='\033[0;34m'
FG_MAGENTA='\033[0;35m'
FG_CYAN='\033[0;36m'
FG_LIGHT_GRAY="\033[0;37m"
FG_DARK_GRAY="\033[0;90m"
FG_LIGHT_RED="\033[0;91m"
FG_LIGHT_GREEN="\033[0;92m"
FG_LIGHT_YELLOW="\033[0;93m"
FG_LIGHT_BLUE="\033[0;94m"
FG_LIGHT_MAGENTA='\033[0;95m'
FG_LIGHT_CYAN='\033[1;96m'
FG_WHITE='\033[1;97m'

log() {
  FG=${1}
  timestamp=$(date +'%Y-%m-%d %H:%M:%S')
  filename=$(basename "${BASH_SOURCE[2]}")
  lineno=${BASH_LINENO[1]}

  shift 1
  string=${*}
  echo -e "$FG[${timestamp}][${filename}:${lineno}]$string"
}

#######################################
# Show info message
#######################################
info()
{
  log "${FG_BLUE}" "[INFO] $*${FG_NONE}"
}

######################################
# Show info message
######################################
warn()
{
  log "${FG_YELLOW}" "[WARN] $*${FG_NONE}"
}

######################################
# Show error message
######################################
error()
{
  log "${FG_RED}" "[ERROR] $*${FG_NONE}"
}

######################################
# Show ok message
######################################
ok()
{
  log "${FG_LIGHT_CYAN}" "[OK] $*${FG_NONE}"
}

######################################
# Print version of this repository
######################################
version() {
  commit=$(git log -1 --pretty=%H)
  date=$(git log -1 --pretty=%cd)
  echo "Commit: ${commit}"
  echo "Date: ${date}"
}

######################################
# Execute a given command and exit if error code is found.
# the command string will be executed.
######################################
execute() {
  cmd="$*"
  eval "$cmd"
  retcode=$?
  if [[ $retcode != 0 ]]; then
    echo -e "\n${FG_RED}Fail to execute command \"$cmd\", exit($retcode)${FG_NONE}\n"
    exit $retcode
  fi
}

######################################
# Get path of ROBOT_DEVKIT scripts directory
######################################
get_scripts_dir()
{
  echo "${ROBOT_DEVKIT_ROOT_DIR}"/scripts
}

######################################
# Get path of ROBOT_DEVKIT config directory
######################################
get_config_dir()
{
  echo "${ROBOT_DEVKIT_ROOT_DIR}"/scripts/config
}

######################################
# Get path of the ROBOT_DEVKIT root directory
######################################
get_root_dir()
{
  echo "${ROBOT_DEVKIT_ROOT_DIR}"
}

######################################
# Get path of the DEVKIT workspace directory
######################################
get_rdk_ws_dir()
{
  echo "${ROBOT_DEVKIT_WS_DIR}"
}

######################################
# Get path of the ROBOT_DEVKIT products directory
######################################
get_products_dir()
{
  echo "${ROBOT_DEVKIT_ROOT_DIR}"/products
}

#######################################
# Get packages directory
#######################################
get_packages_dir()
{
  echo "${ROBOT_DEVKIT_ROOT_DIR}"/packages
}

######################################
# Get installation path for ROBOT_DEVKIT ROS/ROS2 RDK
######################################
get_install_dir()
{
  echo "/opt/robot_devkit"
}

unset CURRENT_DIR
