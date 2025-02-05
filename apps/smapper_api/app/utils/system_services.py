from enum import Enum
import subprocess


class ServiceAction(Enum):
    """Systemctl actions"""

    START = "start"
    STOP = "stop"
    RESTART = "restart"
    STATUS = "status"


class ServiceStatus(Enum):
    """Systemctl Status return codes"""

    ACTIVE = 0
    INACTIVE = 3
    UNKOWN = 4
    UNDIFINED = -1


def _service_command(
    action: ServiceAction, service_name: str
) -> subprocess.CompletedProcess:
    """Run a systemctl command as user based on an action

    Params:
        action: Action to be performed
        service_name: name of the service to execute

    Returns:
        Subprocess completed process output
    """
    cmd = ["systemctl", "--user", action.value, service_name]
    ret = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return ret


def service_start(service_name: str) -> bool:
    ret = _service_command(ServiceAction.START, service_name)
    return True if ret.returncode == 0 else False


def service_stop(service_name: str) -> bool:
    ret = _service_command(ServiceAction.STOP, service_name)
    return True if ret.returncode == 0 else False


def service_status(service_name: str) -> ServiceStatus:
    ret = _service_command(ServiceAction.STATUS, service_name)

    try:
        status = ServiceStatus(ret.returncode)
    except ValueError:
        status = ServiceStatus(-1)

    return status
