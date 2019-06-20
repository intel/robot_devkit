#!/bin/bash

set -e

./rdk.sh install-deps
./rdk.sh sync-src
./rdk.sh build --cmake-args -DCMAKE_BUILD_TYPE=Release
./rdk.sh install
