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
# Get product list
#######################################
get_products()
{
  local -a products=()
  local product
  for i in $(get_products_dir)/*
  do
    product=$(basename "$i")
    if [[ "$product" =~ "current" ]]; then
      continue
    fi
    products=( "${products[@]} $product" )
  done
  echo "${products[@]}"
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
# Get sdk workspace for current product
#######################################
get_current_sdk_ws()
{
  if [[ -d "$(get_products_dir)/current" ]]
  then
    get_sdk_ws_dir
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
  if [[ -d "$(get_products_dir)/current" ]]
  then
    echo "$(get_sdk_ws_dir)/third_party"
  else
    error "No product has been selected"
    exit 1
  fi
}


#######################################
# Select a product for working on
#######################################
select_product()
{
  if [[ -n "$1" ]]; then
    set_current_product "$1"
    exit
  fi

  local index=0
  IFS=', ' read -r -a array <<< "$(get_products)"

  info "Select new product:"
  for p in "${array[@]}"
  do
    if [[ "$(get_current_product)" == "$p" ]]
    then
      echo " *[$((index++))]: $p"
    else
      echo "  [$((index++))]: $p"
    fi
  done

  if [[ -n "$(get_current_product)" ]]
  then
    info "Input selection (current=$(get_current_product)):"
  else
    info "Input selection:"
  fi

  read -rn 1 input_selection
  echo ""

  # check whether select or not
  if [[ -z "$input_selection" ]]; then
    ok "Selection not changed.\n"
    exit 1
  fi

  # check whether input is number
  re='^[0-9]+$'
  if ! [[ $input_selection =~ $re ]] ; then
    error "Invalid input"
    exit 1
  fi

  # check whether input number is in valid range
  if (( input_selection > $((index - 1)) )); then
    error "Invalid selection"
    exit 1
  fi

  # select product according to input
  set_current_product "${array[$input_selection]}"
}

unset CURRENT_DIR
