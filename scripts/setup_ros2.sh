#!/bin/bash
# Install ROS2 Jazzy + the ROS packages the racecar driver depends on.
set -eo pipefail

# Bootstrap tools needed to add the ROS2 apt repo.
sudo apt-get install -y -qq curl wget gnupg ca-certificates lsb-release

if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F 'tag_name' | head -1 | awk -F'"' '{print $4}')
    DEB="/tmp/ros2-apt-source.deb"
    wget -q "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(lsb_release -cs)_all.deb" -O "$DEB"
    sudo dpkg -i "$DEB"
fi

sudo apt-get update

# Runtime: ros-base + the message/driver packages used by the racecar driver.
# Dev:     ros-dev-tools (ament_cmake et al.), colcon, rosdep.
# Diag:    rviz2, rqt, foxglove_bridge for visualization on a remote laptop.
sudo apt-get install -y \
    ros-humble-ros-base \
    ros-humble-ackermann-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-tf2-tools \
    ros-humble-topic-tools \
    ros-humble-diagnostic-updater \
    ros-humble-rqt-graph \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-image-view \
    ros-humble-foxglove-bridge \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete
