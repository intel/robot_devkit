#!/bin/bash

set -e

# Build ROS2 packages
./robot_sdk.sh product tb3                   # select tb3 as target product
./robot_sdk.sh sync-src all --force          # sync all ROS2 source code to sdk_ws folder, clean folder with "--force"
./robot_sdk.sh build all --include-deps --cmake-args -DCMAKE_BUILD_TYPE=Release
                                             # build all ROS2 packges and build dependences with "--include-deps"
./robot_sdk.sh install                       # install generated ROS2 workspace to /opt/robot_sdk/ folder
