"""Convert primary GPS telemetry without inventing altitude or accuracy."""

import math

from sensor_msgs.msg import NavSatFix, NavSatStatus


def navsat_fix(message, stamp, frame_id):
    fix = NavSatFix()
    fix.header.stamp = stamp
    fix.header.frame_id = frame_id
    valid = message.fix_type in (2, 3, 4, 5, 6, 7, 8)
    fix.status.status = NavSatStatus.STATUS_FIX if valid else NavSatStatus.STATUS_NO_FIX
    if message.fix_type in (5, 6):
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
    # GPS_RAW_INT does not identify the constellations or distinguish DGPS/SBAS.
    fix.status.service = 0
    fix.latitude = message.lat / 1e7 if valid else math.nan
    fix.longitude = message.lon / 1e7 if valid else math.nan
    # MAVLink alt is MSL, but NavSatFix requires ellipsoid height. Extension
    # fields can be absent/zero-filled: publish NaN unless a nonzero height exists.
    ellipsoid_mm = getattr(message, "alt_ellipsoid", 0)
    fix.altitude = ellipsoid_mm / 1000.0 if valid and message.fix_type != 2 and ellipsoid_mm else math.nan
    # HDOP/VDOP are not variances. Leave covariance explicitly unknown.
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    return fix
