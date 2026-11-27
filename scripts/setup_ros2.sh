#!/bin/bash
# Install ROS2 Humble + the ROS packages the racecar driver depends on.
set -eo pipefail

# Bootstrap tools needed to add the ROS2 apt repo. A fresh image's package
# lists are stale, so refresh them first or this install fails to fetch.
sudo apt-get update
sudo apt-get install -y -qq curl wget gnupg ca-certificates lsb-release

# The ROS apt source package. The newest release is looked up on GitHub. A
# failed lookup, such as a rate limit or an outage, uses the version known to
# work, as upstream does. Without the fallback, the empty lookup would stop
# this script under pipefail with no message.
ROS_APT_SOURCE_FALLBACK=1.2.0
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    ROS_APT_SOURCE_VERSION=$(curl -fsS --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F 'tag_name' | head -1 | awk -F'"' '{print $4}' || true)
    if [ -z "$ROS_APT_SOURCE_VERSION" ]; then
        echo "GitHub lookup failed; using ros-apt-source ${ROS_APT_SOURCE_FALLBACK}."
        ROS_APT_SOURCE_VERSION="$ROS_APT_SOURCE_FALLBACK"
    fi
    DEB="/tmp/ros2-apt-source.deb"
    wget -q --tries=3 "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(lsb_release -cs)_all.deb" -O "$DEB"
    sudo dpkg -i "$DEB"
fi

sudo apt-get update

# Runtime: ros-base + the message/driver packages used by the racecar driver.
# Dev:     ros-dev-tools (ament_cmake et al.), colcon, rosdep.
# SLAM:    slam_toolbox for mapping; nav2 AMCL, map_server and lifecycle_manager
#          for localization on a saved map (autonomy_node).
# Diag:    rviz2, rqt, foxglove_bridge for visualization on a remote laptop.
sudo apt-get install -y \
    ros-humble-ros-base \
    ros-humble-ackermann-msgs \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-tf2-tools \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-topic-tools \
    ros-humble-diagnostic-updater \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-rqt-graph \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-image-view \
    ros-humble-foxglove-bridge \
    ros-humble-rosbag2-storage-mcap \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete
