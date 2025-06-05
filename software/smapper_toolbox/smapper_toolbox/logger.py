import logging
import coloredlogs

LOG_LEVEL = "INFO"

level_styles = dict(
    spam=dict(color="green", faint=True),
    debug=dict(color="green"),
    verbose=dict(color="blue"),
    info=dict(color="green"),
    notice=dict(color="magenta"),
    warning=dict(color="yellow"),
    success=dict(color="green", bold=True),
    error=dict(color="red"),
    critical=dict(color="red", bold=True),
)

field_styles = dict(
    asctime=dict(color="green"),
    hostname=dict(color="magenta"),
    levelname=dict(color="white", bold=True),
    name=dict(color="blue"),
    programname=dict(color="cyan"),
    username=dict(color="yellow"),
    filename=dict(color="cyan"),
)

logger = logging.getLogger("smapper_api")
coloredlogs.install(
    level=LOG_LEVEL,
    logger=logger,
    fmt="%(levelname)s [%(filename)s]: \t %(message)s",
    level_styles=level_styles,
    field_styles=field_styles,
)
