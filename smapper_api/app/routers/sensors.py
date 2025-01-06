from functools import lru_cache
from typing import Annotated
from fastapi import APIRouter, Depends

from app.config.settings import ServicesSettings
from app.utils.system_services import service_start, service_stop, service_status


router = APIRouter(prefix="/sensors")


@lru_cache
def get_service_settings():
    return ServicesSettings()  # type: ignore


@router.post("/lidar/start")
def lidar_start(settings: Annotated[ServicesSettings, Depends(get_service_settings)]):
    status = service_start(settings.lidar_sensor_service_name)
    return {"status": status}


@router.post("/lidar/stop")
def lidar_stop(settings: Annotated[ServicesSettings, Depends(get_service_settings)]):
    status = service_stop(settings.lidar_sensor_service_name)
    return {"status": status}


@router.get("/lidar/status")
def lidar_status(settings: Annotated[ServicesSettings, Depends(get_service_settings)]):
    status = service_status(settings.lidar_sensor_service_name)
    return {"status": status.name}


@router.post("/cameras/start")
def cameras_start(settings: Annotated[ServicesSettings, Depends(get_service_settings)]):
    status = service_start(settings.cameras_sensor_service_name)
    return {"status": status}


@router.post("/cameras/stop")
def cameras_stop(settings: Annotated[ServicesSettings, Depends(get_service_settings)]):
    status = service_stop(settings.cameras_sensor_service_name)
    return {"status": status}


@router.get("/cameras/status")
def cameras_status(
    settings: Annotated[ServicesSettings, Depends(get_service_settings)]
):
    status = service_status(settings.cameras_sensor_service_name)
    return {"status": status.name}
