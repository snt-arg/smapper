!!! warning


    Never upgrade the distribution with `sudo apt upgrade` as it can break stuff

1. Flash board with JetPack 6.0 (e-con ARGUS cameras only supports 6.0, time of writing)
  - Requires the SDK tool.
  - Make sure to use the SSD (NVME drive)
2. Install nvidia-jetpack: `sudo apt install nvidia-jetpack`
3. Install nvida-container: `sudo apt install nvida-container`
3. Check if docker is installed with `docker --version`
4. If not, install [docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
5. Check if the following works: `docker run --gpus all hello-world`
  - If it does not work check [solution](#docker-solution)
6. Reboot device
7. Install [Isaac development setup](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)
  - You will need to clone `https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common`, to get access to the `run_dev.sh` script
  - Install the Isaac APT repository for ros packages instead
8. Install [Ouster ROS](https://github.com/ouster-lidar/ouster-ros/tree/ros2)


## Cameras Setup
- Download software: https://spaces.e-consystems.com/nextcloud/index.php/s/YsHGaLYqxPsexAs
  - Password: `.|&~[09>"=`
- Follow instructions
- To verify cameras are working (change `<n>` to match the camera on `/dev/video<n>`):
```
gst-launch-1.0 nvarguscamerasrc sensor-id=<n> ! "video/x-raw(memory:NVMM),width=(int)1920,height=(int)1080, format=(string)NV12" ! nv3dsink -e
```

https://www.e-consystems.com/blog/camera/products/what-is-the-nvidia-isaac-ros-how-to-get-started-with-it/

## Pytorch Nvidia Docker container



## Docker Solution
If docker fails to run when using `--gpus`, then most likely it means that the runtime is not set correctly.
To check if it works with the correct runtime, run `docker run --runtime=nvidia --gpus all hello-world`

If it works, then just run the following command to set runtime to nvidia by default `sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker.service`

## Cuda upgrade
https://developer.nvidia.com/blog/simplifying-cuda-upgrades-for-nvidia-jetson-users/
`https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=aarch64-jetson&Compilation=Native&Distribution=Ubuntu&target_version=22.04&target_type=deb_local`




## Good to know

https://docs.nvidia.com/jetson/jetpack/install-setup/index.html#list-of-debian-packages
