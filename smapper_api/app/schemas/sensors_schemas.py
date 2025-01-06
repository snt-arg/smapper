from pydantic import dataclasses


@dataclasses.dataclass
class BMSSensorSchema:
    soc: int  # Percentage
    temperature: float  # degrees
    voltage: float  # Volts
    current_draw: float  # Amps
    status: str


@dataclasses.dataclass
class LidarSensorSchema:
    status: str


@dataclasses.dataclass
class LidarDataSchema:
    status: str
    points: list[float]


@dataclasses.dataclass
class CamerasSensorSchema:
    cam1_status: str
    cam2_status: str
    cam4_status: str
    cam3_status: str
