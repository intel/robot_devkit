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
      info "Execute vcs-import ${src_dir} < $file"
      vcs-import "${src_dir}" < "$file"
      vcs-pull "${src_dir}" < "$file" 1>/dev/null || true
    done <  <(find "${repo_dir}" -name '*.repos' -print0)
  else
    error "$repo_dir does not exist\n"
    exit 1
  fi
}

#######################################
# Sync the source code for groups
# Arguments:
#   pkg: core or device or modules or all
#   sync_option: --force
#######################################
sync_src_pkg()
{
  info "Sync code [$1] [$2]\n"
  local pkg=${1}
  local pkg_ws=${pkg}_ws
  local sync_option=${2}

  # local repo and src directory
  local repo_dir
  local src_dir

  repo_dir=$(get_packages_dir)/$pkg/repos
  src_dir=$(get_rdk_ws_dir)/$pkg_ws/src

  if [[ -d "$repo_dir" ]]; then
    # sync ros2 packages
    sync_src_execute "${repo_dir}" "${src_dir}" "${sync_option}"

    ok "Successful sync-ed source to ../rdk_ws/$pkg_ws/src.\n"
  else
    warn "$repo_dir directory does not exist"
  fi
}

#######################################
# delete the unselected package
#######################################
del_sync_src_pkg()
{
  local pkg=$1
  local src_dir
  local pkg_ws=${pkg}_ws
  local flag

  src_dir=$(get_rdk_ws_dir)/$pkg_ws

  if [[ -d "$src_dir" ]];then
    echo -e -n "\n${FG_BLUE}Are you sure to delete the [$pkg] package:[Y/n]?${FG_NONE}"
    read -r flag
    shopt -s nocasematch
    case $flag in
      n|no)
        info "skip package [$pkg] :$src_dir"
        ;;
      y|yes)
        info "delete package [$pkg] :$src_dir"
        rm -rf "$src_dir"
        ;;
      *)
        info "default delete [$pkg] :$src_dir"
        rm -rf "$src_dir"
        ;;
    esac
  fi
}

#######################################
# Common sync command entry
# Arguments:
#   group: core or device or modules or all
#   sync_option: --force
#######################################
sync_src()
{
  info "Sync code ...\n"
  local sync_option=${1}

  if [[ "${sync_option}" ]] && [[ "${sync_option}" != "--force" ]]; then
    error "./rdk.sh: error: unrecognized arguments:'${sync_option}'"
    exit 1
  fi

  read -r -a del_pkgs <<< "$(get_packages_delete_list)"
  for del_pkg in "${del_pkgs[@]}"
  do
    del_sync_src_pkg "$del_pkg"
  done

  read -r -a array <<< "$(get_packages_list)"
  for pkg in "${array[@]}"
  do
    sync_src_pkg "$pkg" "${sync_option}"
  done

}

unset CURRENT_DIR
