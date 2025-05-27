from rich.progress import Progress, SpinnerColumn, TextColumn
import typer
import os
from calib_toolbox.config import Config
from calib_toolbox.docker import DockerHelper
from calib_toolbox.executor import execute_pool
from calib_toolbox.logger import logger
from calib_toolbox.calibration import (
    CameraCalibration,
    IMUCalibration,
    CameraIMUCalibration,
)
from calib_toolbox.utils import passthrough_xhost_to_docker


def verify_calibration_dir(config: Config) -> bool:
    logger.info(f"Verifying file contents in {config.calibration_dir}")
    # Check if SMapper repository exists
    if not os.path.isdir(config.smapper_dir):
        logger.error(f"SMapper directory {config.smapper_dir} does not exist")
        return False

    # Check if april tag file exists
    april_tag_path = os.path.join(config.calibration_dir, config.april_tag_filename)
    if not os.path.isfile(april_tag_path):
        logger.error(f"April Tag config file {april_tag_path} does not exist")
        return False

    # Check if rosbags directory exists, and structure is correct
    # rosbags_dir/
    #   |- ros1/
    #   |- ros2/
    if not os.path.isdir(config.rosbags_dir):
        logger.error(f"Rosbags {config.rosbags_dir} directory does not exist")
        return False

    ros1_bag_dir = os.path.join(config.rosbags_dir, "ros1")
    ros2_bag_dir = os.path.join(config.rosbags_dir, "ros2")
    if not os.path.isdir(ros2_bag_dir):
        logger.error(
            f"Ros2 bags {ros2_bag_dir} directory does not exist. Make sure you place ros2 bags inside {ros2_bag_dir}"
        )
        return False

    if not os.path.isdir(ros1_bag_dir):
        logger.info(
            f"Ros1 bags {ros1_bag_dir} directory does not yet exist. Creating it."
        )
        os.makedirs(ros1_bag_dir)

    return True


def convert_ros2_bags(config: Config):
    ros1_bags_dir = os.path.join(config.rosbags_dir, "ros1")
    ros2_bags_dir = os.path.join(config.rosbags_dir, "ros2")

    logger.info("Looking for ros2 bags to be converted")
    cmds = []
    for bag in os.listdir(ros2_bags_dir):
        src = os.path.join(ros2_bags_dir, bag)
        dest = os.path.join(ros1_bags_dir, f"{bag}.bag")
        if os.path.isfile(dest):
            logger.debug(
                f"{bag} has already been converted. If this is not the case, remove the bag {dest}"
            )
            continue
        logger.info(f"Found ros2 bag {bag}")
        cmd = ["rosbags-convert", "--src", src, "--dst", dest]
        cmds.append(cmd)

    if len(cmds) == 0:
        return
    logger.info(f"A total of {len(cmds)} ros2 bags will be converted.")
    execute_pool(cmds, "Converting ros2 bags", config.parallel_jobs)


def prepare_kalibr_docker(config: Config) -> DockerHelper:
    docker_helper = DockerHelper()
    if not docker_helper.image_exists(config.kalibr_image_tag):
        logger.info("Kablir Docker image does not yet exist. Building it")
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            transient=True,
        ) as progress:
            progress.add_task(description="Building docker image...", total=None)
            docker_helper.build_image(
                tag=config.kalibr_image_tag,
                path=config.smapper_dir,
                dockerfile=os.path.join(
                    config.smapper_dir, "docker", "kalibr", "Dockerfile"
                ),
            )
            logger.info("Kablir Docker image has been created!")
        return docker_helper
    logger.info(f"Kablir Docker image <{config.kalibr_image_tag}> found.")
    return docker_helper


def main(
    camera_calib: bool = typer.Option(
        False, "--camera-calib", "-c", help="Run camera calibration"
    ),
    imu_calib: bool = typer.Option(
        False, "--imu-calib", "-i", help="Run IMU calibration"
    ),
    camera_imu_calib: bool = typer.Option(
        False, "--camera-imu-calib", "-ci", help="Run camera-IMU calibration"
    ),
):
    """
    Run camera and IMU calibration using Kalibr.

    This tool helps calibrate cameras and IMUs using the Kalibr toolbox in a Docker container.
    It supports camera calibration, IMU calibration, and camera-IMU calibration.

    The tool expects a specific directory structure and naming convention for the rosbags:
    - Camera calibration bags should have prefix 'calib_'
    - IMU calibration bags should have prefix 'imu_'
    - Camera-IMU calibration bags should have prefix 'cam_imu_'
    """
    logger.info("Hello from calib-toolbox!")

    config = Config()
    if not verify_calibration_dir(config):
        return

    docker_helper = prepare_kalibr_docker(config)
    convert_ros2_bags(config)

    logger.info("Running xhost for docker display support.")
    passthrough_xhost_to_docker()

    if camera_calib:
        calibrator = CameraCalibration(config, docker_helper)
        calibrator.run()

    if imu_calib:
        logger.info(
            "Running IMU calibrations. Only rosbags with prefix imu are considered!"
        )
        calibrator = IMUCalibration(config, docker_helper)
        calibrator.run()

    if camera_imu_calib:
        calibrator = CameraIMUCalibration(config, docker_helper)
        calibrator.run()


if __name__ == "__main__":
    typer.run(main)
