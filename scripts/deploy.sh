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


# This function will be ran on the target device
install_target() {

  # Execute installation remotely
  FILENAME=$1

  if [[ -n "$2" ]]; then
    echo "set http_proxy=$2"
    export http_proxy=$2
  fi

  cd "$HOME"
  tar xvf "$FILENAME"
  cd rdk_release
  ./install.sh
}


#######################################
# Deploy the generated RDK to remote target platform.
#######################################
deploy_rdk()
{
  info "Deploy rdk ...\n"

  if [[ "$#" -lt 6 ]] ; then
    error "Require target platform's user and ip with \"-u <user> -h <ip> -f <file>\""
    exit 1
  fi

  # get command line arguments
  while [ $# -gt 0 ]; do
    case "$1" in
      -u|--user)
        TARGET_USER="$2"
        ;;
      -h|--host)
        TARGET_HOST="$2"
        ;;
      -f|--file)
        FILE="$2"
        ;;
      -p|--proxy)
        PROXY="$2"
        ;;
      *)
        echo "Error: Invalid arguments: $1 $2"
        exit 1
    esac
    shift
    shift
  done

  # Copy rdk binary package to remote device
  info "Copy $FILE $TARGET_USER@$TARGET_HOST:~"
  scp "$FILE" "$TARGET_USER"@"$TARGET_HOST":~

  FILENAME=$(basename "$FILE")
  # Installs dependencies on target platform
  ssh -t "$TARGET_USER"@"$TARGET_HOST" "$(declare -pf install_target); install_target $FILENAME $PROXY"

  ok "Successfully deployed ros2 on $TAREGET@$TARGET_HOST \n"
}

unset CURRENT_DIR
