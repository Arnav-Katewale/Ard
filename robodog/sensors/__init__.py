"""Sensor interfaces (IMU, ultrasonic) and simulated sensor backends."""

from enum import Enum


class SensorStatus(Enum):
    OK = "OK"
    INVALID = "INVALID"            # value read but physically implausible (NaN, out of range)
    TIMEOUT = "TIMEOUT"            # no response within the allowed time
    OUT_OF_RANGE = "OUT_OF_RANGE"  # outside the sensor's configured reliable range
    ERROR = "ERROR"                # communication / driver failure
