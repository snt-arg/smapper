import subprocess


def passthrough_xhost_to_docker() -> bool:
    ret = subprocess.call(["xhost", "+local:docker"], stdout=subprocess.DEVNULL)
    return ret == 0
