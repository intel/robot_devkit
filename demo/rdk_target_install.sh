#!/bin/bash

set -e

CURRENT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

add_apt_repo()
{
  # Authorize gpg key with apt
  sudo apt update && sudo apt install -y git curl gnupg2 lsb-release
  curl http://repo.ros2.org/repos.key | sudo apt-key add -

  # Add the repository to sources list
  sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

  # TODO: workaround for "public key is not available: NO_PUBKEY F42ED6FBAB17C654" (https://github.com/ros2/ros2/issues/742)
  sudo apt-key add ${CURRENT_DIR}/keys/*

  # Install development tools and ROS tools
  sudo apt update && sudo apt install -y \
    python-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions
}

install_ros2()
{
  sudo cp -rf $CURRENT_DIR/rootfs/* /
}

install_ros2_deps()
{
  # TODO: workaround to unattach installation of libssl depends by pyqt5-dev
  sudo -H DEBIAN_FRONTEND=noninteractive apt-get install -y pyqt5-dev

  # TODO: manually update deps list here to avoid "rosdep update always timeout" issue
  sudo -H apt-get install -y \
   clang-format \
   cmake \
   cppcheck \
   curl \
   libassimp-dev \
   libconsole-bridge-dev \
   libcppunit-dev \
   libcurl4-openssl-dev \
   libeigen3-dev \
   libfreetype6 \
   libfreetype6-dev \
   libgl1-mesa-dev \
   libglu1-mesa-dev \
   liblog4cxx-dev \
   libopencv-dev \
   libpcre3-dev \
   libpoco-dev \
   libqt5core5a \
   libqt5gui5 \
   libqt5opengl5 \
   libqt5widgets5 \
   libtinyxml2-dev \
   libtinyxml-dev \
   libx11-dev \
   libxaw7-dev \
   libxml2-utils \
   libxrandr-dev \
   libyaml-dev \
   openssl \
   pkg-config \
   pydocstyle \
   pyflakes3 \
   pyqt5-dev \
   python3-catkin-pkg-modules \
   python3-empy \
   python3-flake8 \
   python3-lark-parser \
   python3-lxml \
   python3-matplotlib \
   python3-mock \
   python3-numpy \
   python3-pep8 \
   python3-pkg-resources \
   python3-psutil \
   python3-pydot \
   python3-pygraphviz \
   python3-pyqt5 \
   python3-pyqt5.qtsvg \
   python3-pytest \
   python3-setuptools \
   python3-sip-dev \
   python3-yaml \
   qt5-qmake \
   qtbase5-dev \
   ros-dashing-sqlite3-vendor \
   tango-icon-theme \
   uncrustify \
   zlib1g-dev
}

install_librealsense()
{
  sudo sh -c 'echo "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo `lsb_release -cs` main" > /etc/apt/sources.list.d/librealsense.list'
  sudo apt update && sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev
}

install_opencv()
{
  sudo apt-get install -y build-essential
  sudo apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
  sudo apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
}

install_openvino()
{
  sudo apt install -y libcairo2-dev libpango1.0-dev libglib2.0-dev \
    libgtk2.0-dev libswscale-dev libavcodec-dev libavformat-dev \
    libgstreamer1.0-0 gstreamer1.0-plugins-base \
    build-essential cmake libusb-1.0-0-dev libdrm-dev
  sudo -E apt-get install -y libpng-dev
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  sudo ldconfig
}

install_turtlebot3()
{
  sudo udevadm control --reload-rules
  sudo udevadm trigger

  # Install dependencies for LDS
  sudo apt install -y libboost-all-dev ntpdate
}

install_turtlebot3_deps()
{
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
    gazebo*
}

function main()
{
   add_apt_repo
   install_ros2
   install_ros2_deps
   install_librealsense
   install_opencv
   install_openvino
   install_turtlebot3
   install_turtlebot3_deps
}

main "$@"
