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

set -e

# pre build device

# Install Cmake
sudo apt install -y \
    cmake

# Install Cartographer dependencies
sudo apt install -y \
    google-mock \
    libceres-dev \
    liblua5.3-dev \
    libboost-dev \
    libboost-iostreams-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libcairo2-dev \
    libpcl-dev \
    python3-sphinx

# Install Navigation2 dependencies
sudo apt install -y \
    libsdl-image1.2 \
    libsdl-image1.2-dev \
    libsdl1.2debian \
    libsdl1.2-dev

# Install gazebo
sudo apt install -y \
    libgazebo9-dev \
    gazebo9
