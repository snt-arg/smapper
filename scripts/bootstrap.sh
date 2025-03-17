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

sudo apt update
sudo apt upgrade -y
sudo apt install -y nvidia-jetpack

sudo apt install -y python3-pip
sudo pip3 install jetson-stats

sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
