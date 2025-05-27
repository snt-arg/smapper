import subprocess
from typing import List
from rich.progress import Progress, SpinnerColumn, TextColumn
import typer
import os
import shutil
from calib_toolbox.config import Config
from calib_toolbox.docker import DockerHelper
from calib_toolbox.executor import Commands, execute_pool
from calib_toolbox.logger import logger
from rich.progress import Progress, SpinnerColumn, TextColumn

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


def prepare_kalibr_docker(config: Config):
    docker_helper = DockerHelper()
    if not docker_helper.image_exists(config.kalibr_image_tag):
        logger.info("Kablir Docker image does not yet exist. Bulding it")
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            transient=True,
        ) as progress:
            progress.add_task(description="Bulding docker image...", total=None)
            docker_helper.build_image(
                tag=config.kalibr_image_tag,
                path=config.smapper_dir,
                dockerfile=os.path.join(
                    config.smapper_dir, "docker", "kalibr", "Dockerfile"
                ),
            )
            logger.info("Kablir Docker image has been created!")
        return
    logger.info(f"Kablir Docker image <{config.kalibr_image_tag}> found.")


def convert_ros2_bags(config: Config):
    ros1_bags_dir = os.path.join(config.rosbags_dir, "ros1")
    ros2_bags_dir = os.path.join(config.rosbags_dir, "ros2")

    logger.info("Looking for ros2 bags to be converted")
    cmds: Commands = []
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


def run_camera_calibration(
    config: Config, bag_name: str, topics: List[str], rolling_shutter: bool = False
):
    docker_data_path = "/data"
    docker_bags_path = "/bags"
    docker_helper = DockerHelper()

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
        os.path.join(docker_data_path, config.april_tag_filename),
        "--models",
        "pinhole-radtan",
        "--dont-show-report",
        "--topics",
    ]

    cmd.extend(topics)

    return docker_helper.get_run_container_cmd(
        "kalibr",
        cmd,
        env_var={"DISPLAY": "$DISPLAY"},
        volumes=[
            "/tmp/.X11-unix:/tmp/.X11-unix:rw",
            f"{config.calibration_dir}:{docker_data_path}",
            f"{os.path.join(config.rosbags_dir, "ros1")}:{docker_bags_path}",
        ],
    )


def run_camera_calibrations(config: Config):
    logger.info(
        f"Running camera calibrations. Only rosbags with prefix calib are considered!"
    )

    cmds = []

    for output in os.listdir(os.path.join(config.rosbags_dir, "ros1")):
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

        cmds.append(run_camera_calibration(config, output, topics))

    execute_pool(
        cmds,
        "Running calibrations in Kalibr...",
        config.parallel_jobs,
    )

    logger.info(
        f"Moving generated files into {os.path.join(config.calibration_dir, "static")}"
    )
    for output in os.listdir(os.path.join(config.rosbags_dir, "ros1")):
        file_extension = output.split(".")[-1]
        if "calib" not in output:
            continue

        if file_extension not in ["yaml", "pdf", "txt"]:
            continue

        file = os.path.join(config.rosbags_dir, "ros1", output)
        shutil.move(file, os.path.join(config.calibration_dir, "static", output))


def run_imu_calibration(config: Config):
    docker_data_path = "/data"
    docker_bags_path = "/bags"
    docker_helper = DockerHelper()

    logger.info("Creating temporary persistant container for roscore")
    docker_helper.create_persistant_container(
        "kalibr",
        "kalibr",
        ["roscore"],
        env_var={"DISPLAY": "$DISPLAY"},
        volumes=[
            "/tmp/.X11-unix:/tmp/.X11-unix:rw",
            f"{config.calibration_dir}:{docker_data_path}",
            f"{os.path.join(config.rosbags_dir, "ros1")}:{docker_bags_path}",
        ],
    )

    def cleanup():
        logger.info("Cleaning up temporary container")
        cmd3 = ["docker", "stop", "kalibr"]
        cmd4 = ["docker", "rm", "kalibr"]
        subprocess.call(cmd3, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.call(cmd4, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    cmd1 = [
        "docker",
        "exec",
        "kalibr",
        "bash",
        "-c",
        f"source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun allan_variance_ros allan_variance /bags/temp /data/{config.imu_config_filename}",
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
            cleanup()
            return

    cmd2 = [
        "docker",
        "exec",
        "kalibr",
        "bash",
        "-c",
        f"source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash &&  rosrun allan_variance_ros analysis.py --data /bags/temp/allan_variance.csv --output /data/static/imu/imu.yaml --config /data/{config.imu_config_filename}",
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
            cleanup()
            return

    cleanup()

    ros1_bags_dir = os.path.join(config.rosbags_dir, "ros1")
    shutil.move(
        f"{ros1_bags_dir}/temp/allan_variance.csv",
        os.path.join(config.calibration_dir, "static", "imu", "allan_variance.csv"),
    )


def run_imu_calibrations(config: Config):
    ros1_bags_dir = os.path.join(config.rosbags_dir, "ros1")

    # Create a temp folder for IMU bags
    os.makedirs(os.path.join(ros1_bags_dir, "temp"), exist_ok=True)
    for file in os.listdir(ros1_bags_dir):
        if "imu" not in file or os.path.isdir(os.path.join(ros1_bags_dir, file)):
            continue

        shutil.move(
            os.path.join(ros1_bags_dir, file),
            os.path.join(ros1_bags_dir, "temp", file),
        )

        run_imu_calibration(config)

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


def main(camera_calib: bool = False, imu_calib: bool = False):
    logger.info("Hello from calib-toolbox!")
    config = Config()
    if not verify_calibration_dir(config):
        return
    prepare_kalibr_docker(config)
    convert_ros2_bags(config)

    logger.info("Running xhost for docker display suppport.")
    passthrough_xhost_to_docker()

    if camera_calib:
        run_camera_calibrations(config)

    if imu_calib:
        logger.info(
            f"Running imu calibrations. Only rosbags with prefix imu are considered!"
        )
        run_imu_calibrations(config)


if __name__ == "__main__":
    typer.run(main)
