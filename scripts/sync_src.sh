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
# Execute code syncing
#######################################
sync_src_execute()
{

  local repo_dir=${1}
  local src_dir=${2}
  local sync_option=${3}

  # clean old folder
  if [[ "${sync_option}" == "--force" ]]; then
    info "Remove old source at ${src_dir}"
    rm -rf "${src_dir}"
  fi

  # sync ros packages
  if [[ -d "${repo_dir}" ]]; then
    info "Search all *.repo under ${repo_dir}"
    while IFS= read -r -d '' file
    do
      mkdir -p "${src_dir}"
      info "vcs-import ${src_dir} < $file"
      vcs-import "${src_dir}" < "$file"
      vcs-pull "${src_dir}" < "$file" 1>/dev/null || true
    done <  <(find "${repo_dir}" -name '*.repos' -print0)
  else
    error "\n$repo_dir does not exist\n"
    exit 1
  fi
}

#######################################
# Sync the source code for groups
# Arguments:
#   group: core or device or modules or all
#   sync_option: --force
#######################################
sync_src_group()
{
  info "\nSync code [$1] $2\n"
  local group=${1}
  local group_ws=${1}_ws
  local sync_option=${2}

  # Install necessary tools before code sync
  local system_setup_exec
  system_setup_exec=$(get_current_product_dir)/$group/scripts/system_setup.sh
  if [[ -f "${system_setup_exec}" ]] ; then
    info "\nSystem Setup before code sync: ${system_setup_exec}\n"
    execute "${system_setup_exec}"
  fi

  # local repo and src directory
  local repo_dir
  local src_dir

  repo_dir=$(get_current_product_dir)/$group/repos
  src_dir=$(get_current_sdk_ws)/$group_ws/src

  # sync ros2 packages
  sync_src_execute "${repo_dir}" "${src_dir}" "${sync_option}"

  ok "\nSuccessful sync-ed source to ./sdk_ws/$group_ws/src\n"
}

#######################################
# Sync the source code for all repos files under repos folder including ROS2.
# Arguments:
#   sync_option: --force
#######################################
sync_src_all()
{
  info "\nSync code [all] $2\n"
  sync_option=$1
  sync_src_group core "${sync_option}"
  sync_src_group device "${sync_option}"
  sync_src_group modules "${sync_option}"
}

#######################################
# Common sync command entry
# Arguments:
#   group: core or device or modules or all
#   sync_option: --force
#######################################
sync_src()
{
  local group=${1}

  if [[ "${group}" != "core" ]] && [[ "${group}" != "device" ]] && [[ "${group}" != "modules" ]] && [[ "${group}" != "all" ]]; then
    error "Please select sync group [core|device|modules|all]"
    exit 1
  fi

  shift
  local sync_option=$*

  if [[ -z "$(get_current_product)" ]]; then
    error "Please select product via command \"rdk.sh product\"."
    exit 1
  fi

  if [[ "${sync_option}" ]] && [[ "${sync_option}" != "--force" ]]; then
    error "./rdk.sh: error: unrecognized arguments:'${sync_option}'"
    exit 1
  fi

  case $group in
    core | device | modules)
      sync_src_group "$group" "${sync_option}"
      ;;
    all)
      sync_src_all "${sync_option}"
      ;;
    *)
      error "\nThe group should be \"core\" or \"device\" or \"modules\" or \"all\" .\n"
      exit 1
      ;;
  esac
}

unset CURRENT_DIR
