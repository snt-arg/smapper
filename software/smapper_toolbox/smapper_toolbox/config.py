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
    """Configuration settings for the calibration toolbox.
    
    This class handles all configuration settings for the calibration process,
    including paths, parameters, and Docker settings. It uses Pydantic for 
    validation and YAML file loading.
    
    Attributes:
        calibration_dir (str): Directory containing calibration files and results.
        rosbags_dir (str): Directory containing ROS bags for calibration.
        april_tag_filename (str): Name of the AprilTag configuration file.
        parallel_jobs (int): Number of parallel jobs to run during calibration.
        kalibr_image_tag (str): Docker image tag for Kalibr container.
        imu_config_filename (str): Path to IMU configuration file, relative to calibration_dir.
        smapper_dir (str): Directory containing the SMapper repository.
        camera_model (str): Camera model to use for calibration (default: pinhole-radtan).
        camera_topic_prefix (str): ROS topic prefix for camera topics.
        camera_topic_suffix (str): ROS topic suffix for camera topics.
        bag_frequency (int): Frequency at which to process the ROS bags.
        config_file (str): Path to the YAML configuration file.
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
        """Customizes the configuration sources for Pydantic settings.
        
        This method is called by Pydantic to determine how to load settings.
        It configures the settings to be loaded from a YAML file.
        
        Args:
            settings_cls: The settings class type.
            init_settings: Settings from initialization.
            env_settings: Settings from environment variables.
            dotenv_settings: Settings from .env file.
            file_secret_settings: Settings from secrets file.
            
        Returns:
            tuple: A tuple containing the YAML config source.
        """
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
        """Expands environment variables and user paths in path strings.
        
        This validator is applied to path fields to expand variables like $HOME
        and ~/ into their full paths.
        
        Args:
            v: The path string to expand.
            
        Returns:
            str: The expanded path.
        """
        return os.path.expandvars(os.path.expanduser(v)) if isinstance(v, str) else v
