#!/bin/bash

set -e

echo "update rosdep install list"

CURRENT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
src_paths="$CURRENT_DIR"/../sdk_ws/core_ws/src/
rosdep_install_file="$CURRENT_DIR"/../products/tb3/core/scripts/rosdep_install.sh
ROS2_DISTRO=crystal

echo "#!/bin/bash" > "$rosdep_install_file"
rosdep install -s --reinstall --from-paths "$src_paths" --ignore-src --rosdistro "$ROS2_DISTRO" -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69  rti-connext-dds-5.3.1 urdfdom_headers"  >> "$rosdep_install_file"
sudo chmod +x "$rosdep_install_file"
