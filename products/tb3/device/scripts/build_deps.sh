#!/bin/bash

set -e

#
# Micro-XRCE-DDS-Agent
#
install_micro_xrce_dds()
{
  echo "Install Micro-XRCE-DDS(Agent & Client) on your SBC ..."
  local src_path=${1}/opencr
  mkdir -p "$src_path"

  # check if ros2 environment has been sourced, if sourced then exit.
  result="$ROS_DISTRO"
  if [[ "$result" != "" ]]; then
    echo "ROS_DISTRO=$result"
    echo "ROS2 environment has been sourced, please open new terminal to re-build Micro-XRCE-DDS)"
    exit 1
  fi

  # Agent
  sudo apt update && sudo apt install -y build-essential cmake git
  cd "${src_path}"
  if [[ -d Micro-XRCE-DDS-Agent ]]
  then
    echo "Micro-XRCE-DDS-Agent does not need to clone again"
  else
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  fi
  cd Micro-XRCE-DDS-Agent && git checkout a495c65faa964ddc068ac6e1249f17f5c9f92787
  mkdir -p build && cd build

  # TODO Temporary solution
  find /usr/local/ |grep fastrtps | sudo xargs rm -rf
  find /usr/local/ |grep fastcdr | sudo xargs rm -rf
  find /usr/local/ |grep microxrcedds_agent | sudo xargs rm -rf

  cmake -DTHIRDPARTY=ON -DCONFIG_UDP_TRANSPORT_MTU=8192 -DCONFIG_SERIAL_TRANSPORT_MTU=8192 ..
  sudo make install
  mkdir -p ~/turtlebot3
  cp -f ../DEFAULT_FASTRTPS_PROFILES.xml ~/turtlebot3
  sudo ldconfig /usr/local/lib/

  # Client
  cd "${src_path}"
  if [[ -d Micro-XRCE-DDS-Client ]]
  then
    echo "Micro-XRCE-DDS-Client does not need to clone again"
  else
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Client.git
  fi
  cd Micro-XRCE-DDS-Client && git checkout 4549ef06040db30532604473f9aac20f9ad1559f
  sed -i 's/CONFIG_UDP_TRANSPORT_MTU=512/CONFIG_UDP_TRANSPORT_MTU=8192/g' client.config
  mkdir -p build && cd build
  cmake -DTHIRDPARTY=ON ..
  sudo make install


  # build_lidar
  echo "Build LIDAR Client"
  cd "${src_path}"/Micro-XRCE-DDS-Client/examples
  wget -c https://github.com/ROBOTIS-GIT/turtlebot3/raw/ros2/turtlebot3_lidar/turtlebot3_lidar.tar.bz2
  tar -xvjf ./turtlebot3_lidar.tar.bz2
  cd "${src_path}"/Micro-XRCE-DDS-Client
  if ! test "$(grep "add_subdirectory(examples/turtlebot3_lidar)" CMakeLists.txt)"
  then
    sed -i '/if(EPROSIMA_BUILD_EXAMPLES)/a\    add_subdirectory(examples/turtlebot3_lidar)' CMakeLists.txt
  fi
  cd "${src_path}"/Micro-XRCE-DDS-Client/build
  cmake -DTHIRDPARTY=ON -DEPROSIMA_BUILD_EXAMPLES=ON ..
  sudo make install
  cp -f ./examples/turtlebot3_lidar/turtlebot3_lidar ~/turtlebot3
}

#
# Install dependencies for OpenCR
#
install_opencr()
{
  echo "Install dependencies for OpenCR ..."

  # Install udev rules for OpenCR
  cd /etc/udev/rules.d/
  sudo wget -c https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger
}

#
# Install dependencies for LDS
#
install_lds()
{
  echo "Install dependencies for LDS ..."

  # Install udev rules for LDS
  cd /etc/udev/rules.d/
  sudo wget -c https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/master/turtlebot3_bringup/99-turtlebot3-cdc.rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger

  # Install boost library
  sudo apt install -y libboost-all-dev

  # Install ntpdate
  sudo apt install -y ntpdate
}

#
# Main entry to build deps
#
function main()
{
  deps_dir=${1}

  if [ ! -d "${deps_dir}" ]; then
    mkdir "${deps_dir}" -p
    echo "No such directory, create \"${deps_dir}\""
  fi

  # Ref to [http://emanual.robotis.com/docs/en/popup/turtlebot3_ros2_sbc_setting]
  install_lds
  install_micro_xrce_dds "${deps_dir}"
  install_opencr
}

main "$@"
