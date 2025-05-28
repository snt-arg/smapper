from abc import ABC, abstractmethod
from typing import List
import os
import shutil
import subprocess
from rich.progress import Progress, SpinnerColumn, TextColumn

from calib_toolbox.config import Config
from calib_toolbox.docker import DockerRunner
from calib_toolbox.executor import execute_pool
from calib_toolbox.logger import logger


class CalibrationBase(ABC):
    def __init__(self, config: Config, docker_helper: DockerRunner):
        self.config = config
        self.docker_helper = docker_helper
        self.docker_data_path = "/data"
        self.docker_bags_path = "/bags"

    @abstractmethod
    def run(self):
        pass


class CameraCalibration(CalibrationBase):
    def run_single_calibration(
        self, bag_name: str, topics: List[str], rolling_shutter: bool = False
    ):
        cmd = [
            "rosrun",
            "kalibr",
            (
                "kalibr_calibrate_rs_cameras"
                if rolling_shutter
                else "kalibr_calibrate_cameras"
            ),
            "--bag",
            f"/bags/{bag_name}",
            "--bag-freq",
            "10",
            "--target",
            os.path.join(self.docker_data_path, self.config.april_tag_filename),
            "--models",
            "pinhole-radtan",
            "--dont-show-report",
            "--topics",
        ]
        cmd.extend(topics)

        return self.docker_helper.get_run_container_cmd(
            "kalibr",
            cmd,
            env_var={"DISPLAY": "$DISPLAY"},
            volumes=[
                "/tmp/.X11-unix:/tmp/.X11-unix:rw",
                f"{self.config.calibration_dir}:{self.docker_data_path}",
                f"{os.path.join(self.config.rosbags_dir, 'ros1')}:{self.docker_bags_path}",
            ],
        )

    def run(self):
        logger.info(
            "Running camera calibrations. Only rosbags with prefix calib are considered!"
        )
        cmds = []

        for output in os.listdir(os.path.join(self.config.rosbags_dir, "ros1")):
            if "calib" not in output:
                continue

            bag_split = output.split("_")
            if len(bag_split) < 2:
                logger.warning(
                    f"A bag was found with an invalid name [{output}]. Name must be of the form calibxx_[camera_name].bag"
                )
                logger.warning("Discarding it")
                continue

            topics = ["_".join(bag_split[1:]).split(".")[0]]
            topics[0] = "/camera/" + topics[0] + "/image_raw"

            cmds.append(self.run_single_calibration(output, topics))

        execute_pool(
            cmds,
            "Running calibrations in Kalibr...",
            self.config.parallel_jobs,
        )

        logger.info(
            f"Moving generated files into {os.path.join(self.config.calibration_dir, 'static')}"
        )
        for output in os.listdir(os.path.join(self.config.rosbags_dir, "ros1")):
            file_extension = output.split(".")[-1]
            if "calib" not in output:
                continue

            if file_extension not in ["yaml", "pdf", "txt"]:
                continue

            file = os.path.join(self.config.rosbags_dir, "ros1", output)
            shutil.move(
                file, os.path.join(self.config.calibration_dir, "static", output)
            )


class IMUCalibration(CalibrationBase):
    def _cleanup_container(self):
        logger.info("Cleaning up temporary container")
        cmd3 = ["docker", "stop", "kalibr"]
        cmd4 = ["docker", "rm", "kalibr"]
        subprocess.call(cmd3, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.call(cmd4, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def run_single_calibration(self):
        logger.info("Creating temporary persistant container for roscore")
        self.docker_helper.create_persistant_container(
            "kalibr",
            "kalibr",
            ["roscore"],
            env_var={"DISPLAY": "$DISPLAY"},
            volumes=[
                "/tmp/.X11-unix:/tmp/.X11-unix:rw",
                f"{self.config.calibration_dir}:{self.docker_data_path}",
                f"{os.path.join(self.config.rosbags_dir, 'ros1')}:{self.docker_bags_path}",
            ],
        )

        cmd1 = [
            "docker",
            "exec",
            "kalibr",
            "bash",
            "-c",
            f"source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun allan_variance_ros allan_variance /bags/temp /data/{self.config.imu_config_filename}",
        ]
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            transient=True,
        ) as progress:
            progress.add_task(description="Running Allan Variance...", total=None)
            stdout = subprocess.DEVNULL
            stderr = subprocess.PIPE
            ret = subprocess.call(cmd1, stdout=stdout, stderr=stderr, text=True)
            if ret != 0:
                logger.error("Something went wrong while running Allan Variance")
                print(stdout)
                print(stderr)
                self._cleanup_container()
                return False

        cmd2 = [
            "docker",
            "exec",
            "kalibr",
            "bash",
            "-c",
            f"source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash &&  rosrun allan_variance_ros analysis.py --data /bags/temp/allan_variance.csv --output /data/static/imu/imu.yaml --config /data/{self.config.imu_config_filename}",
        ]

        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            transient=True,
        ) as progress:
            progress.add_task(description="Analyzing Allan Variance...", total=None)
            stdout = subprocess.PIPE
            stderr = subprocess.PIPE
            ret = subprocess.call(cmd2, stdout=stdout, stderr=stderr, text=True)
            if ret != 0:
                logger.error("Something went wrong while analyzing Allan Variance")
                print(stdout)
                print(stderr)
                self._cleanup_container()
                return False

        self._cleanup_container()
        return True

    def run(self):
        ros1_bags_dir = os.path.join(self.config.rosbags_dir, "ros1")
        os.makedirs(os.path.join(ros1_bags_dir, "temp"), exist_ok=True)

        for file in os.listdir(ros1_bags_dir):
            if "imu" not in file or os.path.isdir(os.path.join(ros1_bags_dir, file)):
                continue

            shutil.move(
                os.path.join(ros1_bags_dir, file),
                os.path.join(ros1_bags_dir, "temp", file),
            )

            if self.run_single_calibration():
                ros1_bags_dir = os.path.join(self.config.rosbags_dir, "ros1")
                shutil.move(
                    f"{ros1_bags_dir}/temp/allan_variance.csv",
                    os.path.join(
                        self.config.calibration_dir,
                        "static",
                        "imu",
                        "allan_variance.csv",
                    ),
                )

            shutil.move(
                os.path.join(ros1_bags_dir, "temp", file),
                os.path.join(ros1_bags_dir, file),
            )

        # Cleanup
        for file in os.listdir(os.path.join(ros1_bags_dir, "temp")):
            shutil.move(
                os.path.join(ros1_bags_dir, "temp", file),
                os.path.join(ros1_bags_dir, file),
            )

        os.removedirs(os.path.join(ros1_bags_dir, "temp"))


class CameraIMUCalibration(CalibrationBase):
    def run_single_calibration(self, bag_name: str, camera_yaml: str, imu_yaml: str):
        cmd = [
            "rosrun",
            "kalibr",
            "kalibr_calibrate_imu_camera",
            "--bag",
            f"/bags/{bag_name}",
            "--cam",
            os.path.join(self.docker_data_path, camera_yaml),
            "--imu",
            os.path.join(self.docker_data_path, imu_yaml),
            "--target",
            os.path.join(self.docker_data_path, self.config.april_tag_filename),
            "--dont-show-report",
        ]

        return self.docker_helper.get_run_container_cmd(
            "kalibr",
            cmd,
            env_var={"DISPLAY": "$DISPLAY"},
            volumes=[
                "/tmp/.X11-unix:/tmp/.X11-unix:rw",
                f"{self.config.calibration_dir}:{self.docker_data_path}",
                f"{os.path.join(self.config.rosbags_dir, 'ros1')}:{self.docker_bags_path}",
            ],
        )

    def run(self):
        logger.info(
            "Running camera-IMU calibrations. Only rosbags with prefix cam_imu are considered!"
        )
        cmds = []

        # Find camera calibration files
        camera_yamls = []
        imu_yaml = os.path.join("static", "imu", "imu.yaml")

        for file in os.listdir(os.path.join(self.config.calibration_dir, "static")):
            if file.endswith(".yaml") and "calib" in file:
                camera_yamls.append(os.path.join("static", file))

        if not camera_yamls:
            logger.error(
                "No camera calibration files found. Please run camera calibration first."
            )
            return

        if not os.path.exists(os.path.join(self.config.calibration_dir, imu_yaml)):
            logger.error(
                "No IMU calibration file found. Please run IMU calibration first."
            )
            return

        for output in os.listdir(os.path.join(self.config.rosbags_dir, "ros1")):
            if "cam_imu" not in output:
                continue

            for camera_yaml in camera_yamls:
                cmds.append(self.run_single_calibration(output, camera_yaml, imu_yaml))

        if not cmds:
            logger.warning(
                "No camera-IMU calibration bags found. Bags should have prefix 'cam_imu'."
            )
            return

        execute_pool(
            cmds,
            "Running camera-IMU calibrations in Kalibr...",
            self.config.parallel_jobs,
        )

        logger.info(
            f"Moving generated files into {os.path.join(self.config.calibration_dir, 'static')}"
        )
        for output in os.listdir(os.path.join(self.config.rosbags_dir, "ros1")):
            file_extension = output.split(".")[-1]
            if "cam_imu" not in output:
                continue

            if file_extension not in ["yaml", "pdf", "txt"]:
                continue

            file = os.path.join(self.config.rosbags_dir, "ros1", output)
            shutil.move(
                file, os.path.join(self.config.calibration_dir, "static", output)
            )

