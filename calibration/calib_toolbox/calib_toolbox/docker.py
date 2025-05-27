import os
from typing import Dict, List, Optional
from annotated_types import doc
import docker
import subprocess
from docker.errors import ImageNotFound

client = docker.from_env()


class DockerHelper:
    def __init__(self):
        self.client = docker.from_env()

    def image_exists(self, image_name: str) -> bool:
        try:
            self.client.images.get(image_name)
            return True
        except ImageNotFound:
            return False

    def build_image(self, tag: str, path: str, dockerfile: str) -> bool:
        stdout = subprocess.PIPE
        stderr = subprocess.PIPE
        cmd = ["docker", "build", "-t", tag, "-f", dockerfile, path]
        ret = subprocess.call(
            cmd,
            stdout=stdout,
            stderr=stderr,
            text=True,
        )

        return ret == 0

    def run_container(
        self,
        img_tag: str,
        command: List[str],
        env_var: Optional[Dict[str, str]] = None,
        volumes: Optional[List[str]] = None,
    ):
        cmd = ["docker", "run", "--rm"]
        if env_var:
            for key, val in env_var.items():
                cmd.extend(["-e", f"{key}={os.path.expandvars(val)}"])

        if volumes:
            for volume in volumes:
                cmd.extend(["-v", volume])

        cmd.append(img_tag)
        cmd.extend(command)

        ret = subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        return ret == 0

    def get_run_container_cmd(
        self,
        img_tag: str,
        command: List[str],
        env_var: Optional[Dict[str, str]] = None,
        volumes: Optional[List[str]] = None,
    ):
        cmd = ["docker", "run", "--rm"]
        if env_var:
            for key, val in env_var.items():
                cmd.extend(["-e", f"{key}={os.path.expandvars(val)}"])

        if volumes:
            for volume in volumes:
                cmd.extend(["-v", volume])

        cmd.append(img_tag)
        cmd.extend(command)

        return cmd

    def create_persistant_container(
        self,
        container_name: str,
        img_tag: str,
        command: List[str],
        env_var: Optional[Dict[str, str]] = None,
        volumes: Optional[List[str]] = None,
    ):
        cmd = ["docker", "run", "-dt", "--name", container_name]
        if env_var:
            for key, val in env_var.items():
                cmd.extend(["-e", f"{key}={os.path.expandvars(val)}"])

        if volumes:
            for volume in volumes:
                cmd.extend(["-v", volume])

        cmd.append(img_tag)
        cmd.extend(command)

        ret = subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        return ret == 0
