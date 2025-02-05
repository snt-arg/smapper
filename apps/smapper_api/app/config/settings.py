from pydantic_settings import BaseSettings, SettingsConfigDict


class APISettings(BaseSettings):
    version: str = "0.1.0"
    name: str = "Smapper API"
    db_uri: str
    model_config = SettingsConfigDict(
        env_file=".env",
    )


class ServicesSettings(BaseSettings):
    lidar_sensor_service_name: str = "lidar_sensor.service"
    cameras_sensor_service_name: str = "cameras_sensor.service"
    s_graphs_service_name: str = "s_graphs.service"
