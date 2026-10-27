import math
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import NavSatStatus

from airborne_telemetry.gps import navsat_fix


@pytest.mark.parametrize("fix_type,status", [
    (0, -1), (1, -1), (2, 0), (3, 0), (4, 0), (5, 2), (6, 2),
    (7, 0), (8, 0), (99, -1),
])
def test_fix_quality_and_coordinates(fix_type, status):
    message = SimpleNamespace(fix_type=fix_type, lat=182345678, lon=-661234567)
    result = navsat_fix(message, Time(sec=42), "antenna")
    assert result.status.status == status
    assert result.header.stamp.sec == 42
    assert result.header.frame_id == "antenna"
    if status == NavSatStatus.STATUS_NO_FIX:
        assert math.isnan(result.latitude) and math.isnan(result.longitude)
    else:
        assert result.latitude == pytest.approx(18.2345678)
        assert result.longitude == pytest.approx(-66.1234567)
    assert math.isnan(result.altitude)
    assert result.position_covariance_type == 0


def test_ellipsoid_height_does_not_use_msl_or_dop():
    message = SimpleNamespace(fix_type=6, lat=0, lon=0, alt=100000,
                              alt_ellipsoid=73000, eph=80, epv=120)
    result = navsat_fix(message, Time(), "gps")
    assert result.altitude == 73.0
    assert result.position_covariance_type == 0
    message.alt_ellipsoid = 0
    assert math.isnan(navsat_fix(message, Time(), "gps").altitude)
