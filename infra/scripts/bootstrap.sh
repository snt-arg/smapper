#!/bin/bash

# Check if command has been executed with sudo
# Create a list of apt packages to be installed
# - nvidia-jetpack
# - nvidia-container
# APT update
# Install ros2 isaac version
# check if docker is installed
# check if docker is able to run with --gpus
# Install econsystems kernel drivers -> this reboots the pc, so should be done last
#   or come up with a different system
# Install ouster deps

#sudo apt update
#sudo apt upgrade -y
#sudo apt install -y nvidia-jetpack
#
#sudo apt install -y python3-pip python3-venv
#sudo pip3 install jetson-stats
#
#sudo groupadd docker
#sudo usermod -aG docker $USER
#newgrp docker

# Install ros2 humble
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update

sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/smapper/software/ros_ws/install/setup.bash" >> ~/.bashrc

sudo apt install ros-humble-rosbag2-storage-mcap


# UV astral package manager
curl -LsSf https://astral.sh/uv/install.sh | sh


# Ouster ROS
sudo apt install -y             \
    ros-$ROS_DISTRO-pcl-ros     \
    ros-$ROS_DISTRO-tf2-eigen   \
    ros-$ROS_DISTRO-rviz2

sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake                   \
    python3-colcon-common-extensions
