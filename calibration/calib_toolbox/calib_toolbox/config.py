from typing import Tuple, Type, Optional, ClassVar
import os
from pydantic import field_validator
from pydantic_settings import (
    BaseSettings,
    PydanticBaseSettingsSource,
    SettingsConfigDict,
    YamlConfigSettingsSource,
)


class Config(BaseSettings):
    """Configuration for the calibration toolbox.

    Attributes:
        calibration_dir: Directory containing calibration files and results
        rosbags_dir: Directory containing ROS bags
        april_tag_filename: Name of the AprilTag configuration file
        parallel_jobs: Number of parallel jobs to run
        kalibr_image_tag: Docker image tag for Kalibr
        imu_config_filename: Name of the IMU configuration file
        smapper_dir: Directory containing the SMapper repository
        camera_model: Camera model to use for calibration (default: pinhole-radtan)
        camera_topic_prefix: Prefix for camera topics (default: /camera/)
        camera_topic_suffix: Suffix for camera topics (default: /image_raw)
        bag_frequency: Frequency to process the bag at (default: 10)
    """

    calibration_dir: str = ""
    rosbags_dir: str = ""
    april_tag_filename: str = ""
    parallel_jobs: int = 1
    kalibr_image_tag: str = "kalibr"
    imu_config_filename: str = "static/imu/config.yaml"
    smapper_dir: str = ""
    camera_model: str = "pinhole-radtan"
    camera_topic_prefix: str = "/camera/"
    camera_topic_suffix: str = "/image_raw"
    bag_frequency: int = 10

    config_file: str = "config/config.yaml"

    model_config = SettingsConfigDict(
        yaml_file=config_file,
    )

    @classmethod
    def settings_customise_sources(
        cls,
        settings_cls: Type[BaseSettings],
        init_settings: PydanticBaseSettingsSource,
        env_settings: PydanticBaseSettingsSource,
        dotenv_settings: PydanticBaseSettingsSource,
        file_secret_settings: PydanticBaseSettingsSource,
    ) -> Tuple[PydanticBaseSettingsSource, ...]:
        return (YamlConfigSettingsSource(settings_cls),)

    @field_validator(
        "calibration_dir",
        "rosbags_dir",
        "april_tag_filename",
        "smapper_dir",
        mode="before",
    )
    @classmethod
    def expand_path(cls, v: str) -> str:
        return os.path.expandvars(os.path.expanduser(v)) if isinstance(v, str) else v
