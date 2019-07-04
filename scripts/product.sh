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

. "$CURRENT_DIR"/common.sh

#######################################
# Get current selected product
#######################################
get_current_product()
{
  if [[ -d "$(get_products_dir)/current" ]]
  then
    basename "$(readlink -f "$(get_products_dir)/current")"
  else
    echo ""
  fi
}

#######################################
# Get package list
#######################################
get_packages()
{
  local -a packages=()
  local package
  for i in $(get_packages_dir)/*
  do
    package=$(basename "$i")
    # common is always required and needn't configure, skip here
    if [[ $package != "common" ]]; then
      packages=( "${packages[@]}  $package")
    fi
  done
  echo "${packages[@]}"
}

#######################################
# Select a product for build
#######################################
set_current_product()
{
  local to_select=$1

  IFS=', ' read -r -a array <<< "$(get_products)"
  for p in "${array[@]}"
  do
    if [[ "$p" == "$to_select" ]]
    then
      rm -rf "$(get_products_dir)/current"
      ln -s "$(get_products_dir)/$to_select" "$(get_products_dir)/current"
      info "Product \"$1\" is selected for current build!"
      exit 0
    fi
  done
  error "Product $to_select not found."
}

#######################################
# Get the repository directory path for current product
#######################################
get_current_product_dir()
{
  if [[ -d "$(get_products_dir)/current" ]]
  then
    echo "$(get_products_dir)/current"
  else
    error "No product has been selected"
    exit 1
  fi
}

#######################################
# Get the dependence directory path for current product
#######################################
get_current_product_deps_dir()
{
  echo "$(get_rdk_ws_dir)/third_party"
}

#######################################
# Get package delete list
#######################################
get_packages_delete_list()
{
  local package_file
  package_file="$(get_config_dir)/package.cfg"
  if [[ -s "$package_file" ]]; then
    awk 'BEGIN { FS="=" } $2 == "false" {print $1}' "$package_file" |tr "\n" " "
  else
    echo "Profile does not exist!"
    exit 1
  fi

}

#######################################
# Get package list
#######################################
get_packages_list()
{
  local package_file
  package_file="$(get_config_dir)/package.cfg"
  if [[ -s "$package_file" ]]; then
    awk 'BEGIN { FS="=" } $2 == "true" {print $1}' "$package_file" |tr "\n" " "
  else
    echo "Profile does not exist!"
    exit 1
  fi

}

#######################################
# Select a package for working on
#######################################
config_package()
{
  local flag
  local silent=$1
  local silent_file=$2
  config_file="$(get_config_dir)/package.cfg"
  config_file_tmp="$(get_config_dir)/package.tmp"
  IFS=', ' read -r -a array <<< "$(get_packages)"

  if [[ "$silent" =~ "--silent" && -s "$silent_file" ]];then
    read -r -a package <<< $(awk 'BEGIN { FS="=" } $2 == "true" {print $1}' "$silent_file" |tr "\n" " ")
    for p in "${package[@]}"
    do
      echo "$p=true" >> "$config_file_tmp"
    done
  elif [[ "$silent" =~ "--default" ]];then
    read -r -a package <<< "$(get_packages_list)"
  echo -e "${FG_BLUE}Save the configuration to ${FG_NONE}${FG_YELLOW}$(get_config_dir)/package.cfg${FG_NONE}"
    echo -e "${FG_BLUE}Will be install${FG_NONE} ${FG_YELLOW}$(get_packages_list)${FG_NONE}"
    exit 0
  else
    shopt -s nocasematch
    echo "# packages=flag[true|false]" > "$config_file_tmp"
    echo -e "${FG_BLUE}Please select you install package[Y/n]?${FG_NONE}"
    for p in "${array[@]}"
    do
      echo -e -n "${FG_LIGHT_BLUE}[$p]${FG_NONE}:${FG_RED}"
      read -r flag
      echo -e -n "${FG_NONE}"
      case $flag in
        n|no)
          echo "$p=false" >> "$config_file_tmp"
          ;;
        y|yes|*)
          echo "$p=true" >> "$config_file_tmp"
          #echo -e "${FG_BLUE} $p decided to install ${FG_NONE}"
          ;;
      esac
    done
  fi
  mv "$config_file_tmp" "$config_file"
  echo -e "${FG_BLUE}Save the configuration to${FG_NONE}${FG_YELLOW} $(get_config_dir)/package.cfg${FG_NONE}"
  echo -e "${FG_BLUE}Will be install ${FG_NONE}${FG_YELLOW}$(get_packages_list)${FG_NONE}"

}

unset CURRENT_DIR
