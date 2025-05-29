import typer
from calib_toolbox.config import Config
from calib_toolbox.logger import logger
from calib_toolbox.calibration import Calibrators
from calib_toolbox.rosbags import RosbagsConverter


def main(
    camera_calib: bool = typer.Option(
        False, "--camera-calib", "-c", help="Run camera calibration"
    ),
    imu_calib: bool = typer.Option(
        False, "--imu-calib", "-i", help="Run IMU calibration"
    ),
    imu_camera_calib: bool = typer.Option(
        False, "--imu-camera-calib", "-ic", help="Run IMU-camera calibration"
    ),
):
    """
    A toolbox to help automate the calibration process using dockerised kalibr.

    This tool helps calibrate cameras and IMUs using the Kalibr toolbox in a Docker container.
    It supports camera calibration, IMU calibration, and camera-IMU calibration.

    The tool expects a specific directory structure and naming convention for the rosbags:

        - Camera calibration bags should have prefix 'calib_'

        - IMU calibration bags should have prefix 'imu_'

        - IMU-camera calibration bags should have prefix 'cam_imu_'
    """
    logger.info("Hello from calib-toolbox!")

    config = Config()
    rosbags_converter = RosbagsConverter(config.rosbags_dir)
    calibrators = Calibrators(config)

    if not rosbags_converter.convert():
        logger.error("Something went wrong when converting rosbags")
        exit(1)

    calibrators.setup()

    if camera_calib:
        calibrators.calibrate_cameras()

    if imu_calib:
        calibrators.calibrate_imu()

    if imu_camera_calib:
        calibrators.calibrate_imu_camera()


if __name__ == "__main__":
    typer.run(main)
