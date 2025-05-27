import subprocess


def convert_ros2_to_ros1_bag(src: str, dest: str) -> bool:
    ret = subprocess.call(["rosbags-convert", "--src", src, "--dst", dest])
    return ret == 0
