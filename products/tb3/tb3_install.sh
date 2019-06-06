#!/bin/bash

set -e

# Build ROS2 packages
./rdk.sh product tb3                   # select tb3 as target product
./rdk.sh sync-src all --force          # sync all ROS2 source code to sdk_ws folder, clean folder with "--force"
./rdk.sh build all --include-deps --cmake-args -DCMAKE_BUILD_TYPE=Release
                                             # build all ROS2 packges and build dependences with "--include-deps"
./rdk.sh install                       # install generated ROS2 workspace to /opt/robot_devkit/ folder
