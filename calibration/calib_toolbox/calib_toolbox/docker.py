import os
from typing import Dict, List, Optional, Union
from pathlib import Path
import docker
import subprocess
from docker.errors import ImageNotFound, BuildError, APIError
from docker.models.containers import Container
from calib_toolbox.logger import logger

client = docker.from_env()


class DockerError(Exception):
    """Custom exception for Docker-related errors"""

    pass


class DockerRunner:
    def __init__(self):
        try:
            self.client = docker.from_env()
        except Exception as e:
            raise DockerError(f"Failed to initialize Docker client: {str(e)}")

        if not self._passthrough_xhost_to_docker():
            logger.error(
                "Failed to passthrough xhost to docker. GUI apps might not work"
            )

    def image_exists(self, image_name: str) -> bool:
        try:
            self.client.images.get(image_name)
            return True
        except ImageNotFound:
            return False

    def build_image(
        self, tag: str, path: Union[str, Path], dockerfile: Union[str, Path]
    ) -> None:
        """Build a Docker image with better error handling and logging.

        Args:
            tag: Image tag
            path: Build context path
            dockerfile: Path to Dockerfile

        Raises:
            DockerError: If build fails
        """
        try:
            logger.info(f"Building Docker image {tag}")
            self.client.images.build(
                path=str(path),
                dockerfile=str(dockerfile),
                tag=tag,
                rm=True,  # Remove intermediate containers
            )
        except BuildError as e:
            raise DockerError(f"Failed to build image {tag}: {str(e)}")
        except APIError as e:
            raise DockerError(f"Docker API error while building {tag}: {str(e)}")

    def _prepare_container_config(
        self,
        env_var: Optional[Dict[str, str]] = None,
        volumes: Optional[List[str]] = None,
    ) -> Dict:
        """Helper method to prepare container configuration.

        Args:
            env_var: Dictionary of environment variables to pass to container
            volumes: List of volume mount strings in format "src:dst"

        Returns:
            Dict containing the container configuration with expanded paths
        """
        config = {
            "remove": True,  # Auto-remove container when it exits
            "environment": self._expand_environment_vars(env_var or {}),
            "volumes": self._prepare_volumes(volumes or []),
        }
        return config

    def _expand_environment_vars(self, env_vars: Dict[str, str]) -> Dict[str, str]:
        """Expands environment variables in both keys and values.

        Args:
            env_vars: Dictionary of environment variables

        Returns:
            Dictionary with expanded environment variables
        """
        expanded = {}
        for key, value in env_vars.items():
            expanded_key = os.path.expandvars(str(key)) if key else key
            expanded_value = os.path.expandvars(str(value)) if value else value
            expanded[expanded_key] = expanded_value
        return expanded

    def _prepare_volumes(self, volumes: List[str]) -> Dict[str, Dict[str, str]]:
        """Prepares volume mounts with expanded paths.

        Args:
            volumes: List of volume strings in "src:dst" format

        Returns:
            Dictionary of volume mount configurations
        """
        volume_config = {}
        for volume in volumes:
            src, dst = volume.split(":", 1)
            src = os.path.expandvars(src)
            dst = os.path.expandvars(dst.split(":")[0])
            volume_config[src] = {"bind": dst, "mode": "rw"}
        return volume_config

    def run_container(
        self,
        img_tag: str,
        command: List[str],
        env_var: Optional[Dict[str, str]] = None,
        volumes: Optional[List[str]] = None,
    ) -> bool:
        """Run a container with improved error handling"""
        try:
            config = self._prepare_container_config(env_var, volumes)
            # NOTE: stdout/sterr of container could be saved to a logs folder
            self.client.containers.run(
                img_tag,
                command,
                **config,
                detach=False,  # Wait for container to complete
            )
            return True
        except Exception as e:
            logger.error(f"Failed to run container: {str(e)}")
            return False

    def create_persistent_container(
        self,
        container_name: str,
        img_tag: str,
        command: List[str],
        env_var: Optional[Dict[str, str]] = None,
        volumes: Optional[List[str]] = None,
    ) -> Container:
        """Create a persistent container with improved error handling"""
        try:
            config = self._prepare_container_config(env_var, volumes)
            container = self.client.containers.run(
                img_tag,
                command,
                **config,
                name=container_name,
                detach=True,
                tty=True,
            )
            return container
        except Exception as e:
            raise DockerError(
                f"Failed to create persistent container {container_name}: {str(e)}"
            )

    def cleanup_container(
        self,
        container_obj: Optional[Container] = None,
        container_name: Optional[str] = None,
    ) -> None:
        """Clean up a container by name with proper error handling"""

        if container_obj is None:
            assert container_name, "A container object or name must be passed"
            container = self.client.containers.get(container_name)
        else:
            container = container_obj

        try:
            container.stop()
            # container.remove()
        except Exception as e:
            logger.warning(f"Failed to cleanup container {container_name}: {str(e)}")

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

    def create_persistent_container_subprocess(
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

    def _passthrough_xhost_to_docker(self) -> bool:
        ret = subprocess.call(["xhost", "+local:docker"], stdout=subprocess.DEVNULL)
        return ret == 0
