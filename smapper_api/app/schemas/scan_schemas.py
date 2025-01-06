from pydantic import dataclasses


@dataclasses.dataclass
class ScanSchema:
    scan_id: str
    scan_name: str
    site: str
    building: str


@dataclasses.dataclass
class ScanMetadataSchema:
    id: str
    name: str
    site: str
    building: str
    date: str
