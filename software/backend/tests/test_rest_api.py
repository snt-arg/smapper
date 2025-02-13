"""Unit Test for REST API endpoints
Checks whether the endpoints are active.
"""

import requests

API_VERSION = "v1"
API_URL = f"http://127.0.0.1:8000/api/{API_VERSION}"


def test_bms_endpoint():
    res = requests.get(API_URL + "/sensor/bms")
    assert res.status_code == 200


def test_lidar_endpoint():
    res = requests.get(API_URL + "/sensor/lidar")
    assert res.status_code == 200


def test_cameras_endpoint():
    res = requests.get(API_URL + "/sensor/cameras")
    assert res.status_code == 200
