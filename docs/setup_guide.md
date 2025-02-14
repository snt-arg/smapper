# Setup Guide

This guide provides step-by-step instructions for setting up and bootstrapping a Jetson AGX Orin development board, preparing it as the computing unit for the SMapper handheld device.

## Board Setup

To start, an SSD (NVMe) must be installed. It is recommended to use this drive as the primary partition for the OS installation.
Once the SSD is mounted, proceed with installing JetPack 6.0 using the NVIDIA SDK Manager.

!!! warning

    When prompted for the installation location, ensure you select the SSD.

## E-Con Systems Cameras Setup

As of this writing, E-Con Systems cameras are only compatible with JetPack 6.0.

To ensure the cameras are recognized, a kernel-level installation is required. E-Con Systems provides the necessary installation scripts along with a dedicated tool for camera visualization and interaction.

Setup Steps:

- Download the required software: [E-Con Systems Software](https://spaces.e-consystems.com/nextcloud/index.php/s/YsHGaLYqxPsexAs)
    - **Password**: `.|&~[09>"=`
- Follow the provided installation instructions.
- Verify camera functionality (replace `<n>` with the appropriate device number in `/dev/video<n>`):

To ensure the cameras are fully working and the installation has been installed successfully, run `eCamArgus (to be checked)`

Alternatively, one can use GStream command (replace `<n>` with the camera id).

```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=<n> ! "video/x-raw(memory:NVMM),width=(int)1920,height=(int)1080, format=(string)NV12" ! nv3dsink -e
```

## Bootstrap

2. Install nvidia-jetpack: `sudo apt install nvidia-jetpack`
3. Install nvida-container: `sudo apt install nvida-container`
4. Check if docker is installed with `docker --version`
5. If not, install [docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
6. Check if the following works: `docker run --gpus all hello-world`

- If it does not work check [solution](#docker-solution)

6. Reboot device
7. Install [Isaac development setup](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)

- You will need to clone `https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common`, to get access to the `run_dev.sh` script
- Install the Isaac APT repository for ros packages instead

8. Install [Ouster ROS](https://github.com/ouster-lidar/ouster-ros/tree/ros2)

## Pytorch Nvidia Docker container

## Docker Solution

If docker fails to run when using `--gpus`, then most likely it means that the runtime is not set correctly.
To check if it works with the correct runtime, run `docker run --runtime=nvidia --gpus all hello-world`

If it works, then just run the following command to set runtime to nvidia by default `sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker.service`

## Cuda upgrade

https://developer.nvidia.com/blog/simplifying-cuda-upgrades-for-nvidia-jetson-users/
`https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=aarch64-jetson&Compilation=Native&Distribution=Ubuntu&target_version=22.04&target_type=deb_local`

## Additional Resources

- https://docs.nvidia.com/jetson/jetpack/install-setup/index.html#list-of-debian-packages
- https://www.e-consystems.com/blog/camera/products/what-is-the-nvidia-isaac-ros-how-to-get-started-with-it/
