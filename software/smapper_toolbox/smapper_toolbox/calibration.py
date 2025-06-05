from abc import ABC, abstractmethod
from typing import List
import os
import shutil
import subprocess
from rich.progress import Progress, SpinnerColumn, TextColumn

from smapper_toolbox.config import Config
from smapper_toolbox.docker import DockerError, DockerRunner
from smapper_toolbox.executor import execute_pool
from smapper_toolbox.logger import logger

# NOTE: Command for imu-camera calibration
# rosrun kalibr kalibr_calibrate_imu_camera --bag /bags/calib02_side_left.bag \
# --cams /data/static/calib02_side_left-camchain.yaml --imu /data/static/imu/imu.yaml \
# --target /data/april_6x6_80x80cm.yaml --imu-models calibrated --reprojection-sigma 1.0


class CalibrationBase(ABC):
    """Base class for all calibration procedures.

    This abstract class defines the interface for calibration procedures
    and provides common functionality used by all calibration types.

    Args:
        config: Configuration settings for the calibration.
        docker_helper: Helper class for Docker operations.

    Attributes:
        config: Configuration settings for the calibration.
        docker_helper: Helper class for Docker operations.
        docker_data_path: Mount path for data inside Docker container.
        docker_bags_path: Mount path for ROS bags inside Docker container.
    """

    def __init__(
        self,
        config: Config,
        docker_helper: DockerRunner,
        data_path: str,
        bags_path: str,
    ):
        self.config = config
        self.docker_helper = docker_helper
        self.docker_data_path = data_path
        self.docker_bags_path = bags_path

    @abstractmethod
    def run(self):
        """Executes the calibration procedure.

        This method must be implemented by concrete calibration classes
        to perform their specific calibration procedure.
        """
        pass


class CameraCalibration(CalibrationBase):
    """Handles camera calibration using Kalibr.

    This class implements camera calibration for one or more cameras
    using the Kalibr toolbox. It processes ROS bags with camera data
    and generates calibration files.
    """

    def run_single_calibration(
        self, bag_name: str, topics: List[str], rolling_shutter: bool = False
    ):
        """Runs calibration for a single camera.

        Args:
            bag_name: Name of the ROS bag file containing camera data.
            topics: List of ROS topics containing camera images.
            rolling_shutter: Whether to use rolling shutter calibration.

        Returns:
            list: Docker command to run the calibration.
        """
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
        """Runs the camera calibration process.

        Processes all ROS bags with prefix 'calib_' and generates
        calibration files for each camera. The results are stored
        in the static directory within calibration_dir.
        """
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
    """Handles IMU calibration using Allan Variance analysis.

    This class implements IMU calibration using the Allan Variance
    method. It processes ROS bags with IMU data and generates
    calibration parameters.
    """

    def _cleanup_container(self):
        """Cleans up the temporary Docker container used for IMU calibration."""
        logger.info("Cleaning up temporary container")
        self.docker_helper.cleanup_container(container_name="kalibr")

    def run_single_calibration(self):
        """Runs calibration for a single IMU.

        This method performs Allan Variance analysis on IMU data
        and generates calibration parameters.

        Returns:
            bool: True if calibration was successful, False otherwise.
        """
        logger.info("Creating temporary persistant container for roscore")
        self.docker_helper.create_persistent_container(
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
        """Runs the IMU calibration process.

        Processes all ROS bags with prefix 'imu_' and generates
        calibration parameters using Allan Variance analysis.
        The results are stored in the static/imu directory.
        """
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


class IMUCameraCalibration(CalibrationBase):
    """Handles camera-IMU calibration using Kalibr.

    This class implements the calibration of the transformation
    between cameras and IMU using the Kalibr toolbox. It requires
    both camera and IMU calibration to be completed first.
    """

    def run_single_calibration(self, bag_name: str, camera_yaml: str, imu_yaml: str):
        """Runs calibration for a single camera-IMU pair.

        Args:
            bag_name: Name of the ROS bag file containing synchronized data.
            camera_yaml: Path to the camera calibration YAML file.
            imu_yaml: Path to the IMU calibration YAML file.

        Returns:
            list: Docker command to run the calibration.
        """
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
        """Runs the camera-IMU calibration process.

        Processes all ROS bags with prefix 'cam_imu_' and generates
        calibration files for each camera-IMU pair. Requires both
        camera and IMU calibration files to exist. The results are
        stored in the static directory.
        """
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


class Calibrators:
    def __init__(self, config: Config):
        self.config = config
        self.docker_runner = DockerRunner()
        self.docker_data_path = "/data"
        self.docker_bags_path = "/bags"

        self.camera_calibrator = CameraCalibration(
            self.config,
            self.docker_runner,
            self.docker_data_path,
            self.docker_bags_path,
        )
        self.imu_calibrator = IMUCalibration(
            self.config,
            self.docker_runner,
            self.docker_data_path,
            self.docker_bags_path,
        )
        self.imu_camera_calibrator = IMUCameraCalibration(
            self.config,
            self.docker_runner,
            self.docker_data_path,
            self.docker_bags_path,
        )

    def setup(self) -> bool:
        if not self._validate_dirs():
            return False
        if not self._prepare_kalibr_image():
            return False
        return True

    def calibrate_cameras(self):
        self.camera_calibrator.run()

    def calibrate_imu(self):
        self.imu_calibrator.run()

    def calibrate_imu_camera(self):
        self.imu_camera_calibrator.run()

    def _prepare_kalibr_image(self) -> bool:
        if self.docker_runner.image_exists(self.config.kalibr_image_tag):
            logger.info(f"Kablir Docker image <{self.config.kalibr_image_tag}> found.")
            return True

        logger.info("Kablir Docker image does not yet exist. Building it")
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            transient=True,
        ) as progress:
            progress.add_task(
                description="Building docker image. (Patience)", total=None
            )
            try:
                self.docker_runner.build_image(
                    tag=self.config.kalibr_image_tag,
                    path=self.config.smapper_dir,
                    dockerfile=os.path.join(
                        self.config.smapper_dir, "docker", "kalibr", "Dockerfile"
                    ),
                )
            except DockerError as e:
                logger.error("Failed to build kablir image")
                return False

            logger.info("Kablir Docker image has been created!")

        return True

    def _validate_dirs(self) -> bool:
        calib_dir = self.config.calibration_dir
        smapper_dir = self.config.smapper_dir
        logger.info(f"Validating file contents of {calib_dir}")

        # Check if april tag file exists
        april_tag_path = os.path.join(calib_dir, self.config.april_tag_filename)
        if not os.path.isfile(april_tag_path):
            logger.error(f"April Tag config file {april_tag_path} does not exist")
            return False

        # Check if SMapper repository exists
        if not os.path.isdir(smapper_dir):
            logger.error(f"SMapper directory {smapper_dir} does not exist")
            return False

        return True
