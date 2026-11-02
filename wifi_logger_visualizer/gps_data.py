"""Validate optional NavSatFix metadata without coupling it to cell geometry."""
import json
import math


GPS_COLUMNS = {
    "latitude": "REAL",
    "longitude": "REAL",
    "altitude": "REAL",
    "gps_stamp_ns": "INTEGER",
    "gps_frame_id": "TEXT",
    "gps_status": "INTEGER",
    "gps_service": "INTEGER",
    "gps_covariance": "TEXT",
    "gps_covariance_type": "INTEGER",
}


def gps_metadata(message, now_ns: int, max_age: float) -> dict:
    """Return a fresh, valid fix; unavailable altitude is independently nullable.

    Source stamps and now_ns must use the same ROS clock. Zero, future, and
    stale stamps are rejected. Coordinates refer to the antenna frame; no
    map conversion or antenna offset correction is performed here.
    """
    if message is None:
        return {}
    stamp = message.header.stamp
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    age = (now_ns - stamp_ns) / 1e9
    if stamp_ns <= 0 or not 0 <= age <= max_age or message.status.status < 0:
        return {}
    latitude, longitude = message.latitude, message.longitude
    if not (math.isfinite(latitude) and -90 <= latitude <= 90
            and math.isfinite(longitude) and -180 <= longitude <= 180):
        return {}
    covariance_type = int(message.position_covariance_type)
    covariance = list(message.position_covariance)
    covariance_valid = (
        covariance_type in (1, 2, 3) and len(covariance) == 9
        and all(math.isfinite(v) for v in covariance)
        and all(covariance[i] >= 0 for i in (0, 4, 8))
    )
    return {
        "latitude": latitude,
        "longitude": longitude,
        "altitude": message.altitude if math.isfinite(message.altitude) else None,
        "gps_stamp_ns": stamp_ns,
        "gps_frame_id": message.header.frame_id,
        "gps_status": int(message.status.status),
        "gps_service": int(message.status.service),
        "gps_covariance": json.dumps(covariance) if covariance_valid else None,
        "gps_covariance_type": covariance_type if covariance_valid else 0,
    }
