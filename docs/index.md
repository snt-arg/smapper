# Notes

- resources:

  - https://developer.nvidia.com/embedded/learn/jetson-agx-orin-devkit-user-guide/howto.html

- Installed packages

  - `jetson-stats`: HTOP utility for jeson. Can be executed using `jtop`
  - `python3-pip`, `python3-venv`, `uv`

  - docker

  - Isaac ROS

    1. Install `sudo apt install nvidia-jetpack`
    2. Reboot
    3. Confirm installation with `cat /etc/nv_tegra_release` and check the output contains R36 (release), REVISION 4.0
    4. Set GPU and CPU max freq: `sudo /usr/bin/jetson_clocks`
    5. Set max power: `sudo /usr/sbin/nvpmodel -m 0`
    6. Add user to docker group:
       - `sudo usermod -aG docker $USER`
       - `newgrp docker`

  - ros2 humble
  - ros ouster driver + deps
  -

- Cameras System
  - Download software: https://spaces.e-consystems.com/nextcloud/index.php/s/YsHGaLYqxPsexAs
    - Password: `.|&~[09>"=`
- For cameras to work (change `<n>` to match the camera on `/dev/video<n>`):

```
gst-launch-1.0 nvarguscamerasrc sensor-id=<n> ! "video/x-raw(memory:NVMM),width=(int)1920,height=(int)1080, format=(string)NV12" ! nv3dsink -e
```

https://www.e-consystems.com/blog/camera/products/what-is-the-nvidia-isaac-ros-how-to-get-started-with-it/
