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
# Install package dependence
#######################################
install_package_deps()
{

  # Add "common" and configured package list
  read -r -a array <<< "common $(get_packages_list)"

  # Execute install_deps.sh in each packages/scripts
  for pkg in "${array[@]}"
  do
    echo "Install dependence for [$pkg]"

    local deps_dir
    deps_dir=$(get_packages_dir)/${pkg}/deps
    local target_dir
    target_dir=$(get_current_product_deps_dir)

    ## find *.deps and do execution
    dep_list=$(find "${deps_dir}" -name '*.deps' | sort)
    for file in ${dep_list[@]}
    do
      echo -e "\nExecute $file $target_dir"
      bash $file $target_dir
    done
    cat "$deps_dir"/version.ini

  done
}

#######################################
# Common install dependences entry
#######################################
install_deps()
{
  info "\nInstall dependencies ...\n"
  echo $(get_packages_dir)
  if [[ ! -d "$(get_packages_dir)" ]]; then
    error "$(get_packages_dir) does not exist."
    exit 1
  fi

  # Set sudo time invalid
  echo "Disable sudo timeout settings"
  sudo sh -c 'echo "Defaults timestamp_timeout=-1" > /etc/sudoers.d/timeout'


  install_package_deps

  # Delete sudo time invalid operation
  sudo rm -rf /etc/sudoers.d/timeout
}

unset CURRENT_DIR
