#!/bin/bash

set -e

./rdk.sh config --default
./rdk.sh install-deps
./rdk.sh sync-src
./rdk.sh build --cmake-args -DCMAKE_BUILD_TYPE=Release
./rdk.sh install


# Generate tarball and install to remote device
# ./tools/generate_tarball.sh
# ./rdk.sh deploy --user rdktest --host 10.239.89.4 --file rdk_ws/release/rdk_release_201908051110.tar.gz --proxy http://<ip>:<port>
