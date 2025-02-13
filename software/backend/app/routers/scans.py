import logging
from fastapi import APIRouter

LOGGER = logging.getLogger('uvicorn.error')

router = APIRouter(prefix="/api/scan")


@router.post("/scan/start")
def scan_start():
    # TODO: Start the actual scanning:
    #   - Starts recording Rosbag
    #   - Starts Odometry
    #   - Starts SGraphs
    LOGGER.info("Starting scan")

@router.post("/scan/stop")
def scan_stop():
    # TODO: Stop the current scanning, if in progress:
    #   - Save current scan
    #   - Stops recording Rosbag (We need to keep track of the current process PID)
    #   - Stops Odometry service
    #   - Stops SGraphs service
    LOGGER.info("Stoping scan")



@router.post("/scan/save")
def scan_save():
    # TODO: Save the current scan, if in progress:
    #   - Save current scan
    LOGGER.info("Saving scan")
