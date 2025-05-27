from typing import Tuple, Type
import os
from pydantic import field_validator
from pydantic_settings import (
    BaseSettings,
    PydanticBaseSettingsSource,
    SettingsConfigDict,
    YamlConfigSettingsSource,
)


class Config(BaseSettings):
    calibration_dir: str = ""
    rosbags_dir: str = ""
    april_tag_filename: str = ""
    parallel_jobs: int = 1
    kalibr_image_tag: str = "kalibr"
    imu_config_filename: str = "static/imu/config.yaml"
    smapper_dir: str = ""

    model_config = SettingsConfigDict(yaml_file="config/config.yaml")

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
