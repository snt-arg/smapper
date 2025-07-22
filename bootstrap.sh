#!/bin/bash

SMAPPER_REPO_PATH=$HOME/smapper
SMAPPER_APP_PATH=$SMAPPER_REPO_PATH/software/smapper_app
ROS_WS_PATH=$SMAPPER_REPO_PATH/software/ros_ws

API_URL="http://10.42.0.1:8000/api/v1"
ROSBOARD_URL="http://10.42.0.1:8888"

log_info(){
    echo -e "\033[32mINFO: \033[0m${1}"
}

log_warning() {
    echo -e "\033[33mWARN: \033[0m${1}"
}

log_error() {
    echo -e "\033[31mERROR: \033[0m${1}"
}

update_system(){
    log_info "Updating and Upgrading system..."

    sudo apt upgrade -y
    sudo apt update

    log_info "Done!"
}

install_ros2(){
    log_info "Installing ROS2 Humble Distribution..."

    sudo apt install -y software-properties-common
    sudo add-apt-repository universe

    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
    sudo dpkg -i /tmp/ros2-apt-source.deb

    sudo apt update

    sudo apt install -y ros-humble-desktop
    sudo apt install -y ros-dev-tools

    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source ~/smapper/software/ros_ws/install/setup.bash" >> ~/.bashrc

    log_info "Done!"
}

install_ros_deps(){
    log_info "Installing ROS2 Dependencies..."

    sudo apt install -y             \
        ros-$ROS_DISTRO-pcl-ros     \
        ros-$ROS_DISTRO-tf2-eigen   \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-rmw-cyclonedds-cpp

    # sudo apt install -y ros-humble-image-transport

    # Not using this for realsense for now
    # sudo apt install -y ros-humble-realsense2-camera

    log_info "Done!"
}

install_dependencies(){
    log_info "Installing Dependencies..."

    # Python
    sudo apt install -y python3-pip python3-venv

    # OusterRos Deps
    sudo apt install -y         \
        build-essential         \
        libeigen3-dev           \
        libjsoncpp-dev          \
        libspdlog-dev           \
        libcurl4-openssl-dev    \
        cmake                   \
        python3-colcon-common-extensions

    # Jetson Related
    sudo apt install -y nvidia-jetpack
    sudo pip3 install jetson-stats

    log_info "Done!"
}

configure_docker(){
    log_info "Adding Docker to user group..."


    sudo groupadd docker
    sudo usermod -aG docker $USER

    log_info "Installing jq for modifying docker daemon config"
    sudo apt install -y jq

    DAEMON_JSON="/etc/docker/daemon.json"

    # To fix problem with docker setting up network tables
    tmpfile=$(mktemp)
    jq '. + {iptables: false}' "$DAEMON_JSON" > "$tmpfile" && mv "$tmpfile" "$DAEMON_JSON"

    log_info "Done!"
}

setup_network(){
    log_info "Setting up Networking Configuration..."

    log_info "Installing linuxptp"
    sudo apt install linuxptp

    # Symlink services
    log_info "Creating symlinks for system services daemons"
    sudo ln -s $SMAPPER_REPO_PATH/infra/daemons/ptp4l.service /etc/systemd/system/ptp4l.service
    sudo ln -s $SMAPPER_REPO_PATH/infra/daemons/phc2sys.service /etc/systemd/system/phc2sys.service
    sudo ln -s $SMAPPER_REPO_PATH/infra/daemons/multicast-lo.service /etc/systemd/system/multicast-lo.service

    log_info "Enabling & Starting services"
    sudo systemctl daemon-reload
    sudo systemctl enable ptp4l.service
    sudo systemctl enable phc2sys.service
    sudo systemctl enable multicast-lo.service

    sudo systemctl start ptp4l.service
    sudo systemctl start phc2sys.service
    sudo systemctl start multicast-lo.service

    # Create udev rule for nvpps0 for support with PTP
    log_info "Creating udev rule for nvpps0 clock"
    sudo cp $SMAPPER_REPO_PATH/infra/udev/nvpps.rules /etc/udev/rules.d/

    sudo cp $SMAPPER_REPO_PATH/config/network/10-cyclone-max.conf /etc/sysctl.d/
    sudo cp $SMAPPER_REPO_PATH/config/network/ouster_dnsmasq.conf /etc/dnsmask.d/

    echo 'export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"' >> ~/.bashrc
    echo 'export CYCLONEDDS_URI="file://$SMAPPER_REPO_PATH/config/network/cyclonedds.xml"' >> ~/.bashrc

    log_info "Done!"
}

deploy_smapper_api(){
    log_info "Deploying SMapper API..."

    # Python - UV package manager
    log_info "Installing UV Python Package Manager"
    curl -LsSf https://astral.sh/uv/install.sh | sh

    log_info "Creating symlink for smapper_api daemon"
    sudo ln -s $SMAPPER_REPO_PATH/infra/daemons/smapper_api.service /etc/systemd/system/ptp4l.service

    log_info "Enabling & Starting service"
    sudo systemctl daemon-reload
    sudo systemctl enable smapper_api.service
    sudo systemctl start smapper_api.service

    log_info "Done!"
}

deploy_smapper_app(){
    log_info "Deploying SMapper APP..."

    pushd $SMAPPER_APP_PATH > /dev/null

    export API_URL=$API_URL
    export ROSBOARD_URL=$ROSBOARD_URL

    log_info "Building SMapper APP Docker Image"
    docker compose build

    log_info "Deploying APP"
    docker compose up app_prod -d

    popd > /dev/null

    log_info "Done!"
}

clone_smapper_repo(){
    log_info "Deploying SMapper APP..."

    git clone --recurse-submodules https://github.com/snt-arg/smapper.git $SMAPPER_REPO_PATH

    log_info "Done!"
}

realsense_support(){
    log_info "Adding Support to Realsense..."

    log_error "Not Yet Implemented"

    log_info "Done!"
}

apply_patches(){
    log_info "Applying Kernel Module Patches..."

    log_error "Not Yet Implemented"
    # unzip from infra/patches and cp to /lib/modules

    log_info "Done!"
}

build_rosws(){
    log_info "Applying Kernel Module Patches..."

    pushd ROS_WS_PATH > /dev/null

    rosdep install --from-paths --ignore-src -r
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

    popd > /dev/null

    log_info "Done!"
}

install_mprocs(){
    log_info "Applying Kernel Module Patches..."

    log_error "Not Yet Implemented"

    log_info "Done!"
}

# echo "alias smapper=mprocs -c /home/smapper/smapper/config/mprocs.yaml" >> ~/.bashrc
