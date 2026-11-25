"""The MAVLink dialect SpiriMav speaks: ardupilotmega, MAVLink 2, and Spiri's messages.

Import it before using :data:`mavlink`: it settles the wire protocol and adds
Spiri's messages, which a later :func:`pymavlink.mavutil.set_dialect` would
undo.

Spiri's ArduPilot firmware sends messages of its own, defined in
itsOverFactory/mavlink's ``ardupilotmega.xml``. pymavlink doesn't know them, so
without this it would drop them as bad data.

``DISTANCE_SENSOR_MOUNT`` (14000) is a gimbal's rangefinder: ``DISTANCE_SENSOR``
with 32-bit distances (a laser reaches past ``DISTANCE_SENSOR``'s 655 m). Its
``quaternion`` is said to be the sensor's orientation in the body frame, but
the firmware sends the gimbal's own ``GIMBAL_DEVICE_ATTITUDE_STATUS.q`` (roll
and pitch from the horizon, yaw from the vehicle's heading), so the gimbal's
pose, which SpiriMav has anyway, says the same. Its ``crc_extra`` is the one
mavgen computes from the definition: 95, with ``orientation`` before
``covariance``, as of itsOverFactory/mavlink@ac17ff3.
"""

# The message class mirrors pymavlink's generated ones, list class attributes and all.
# ruff: noqa: RUF012

from __future__ import annotations

import os
import struct

from pymavlink import mavutil

# MAVLink 2: we rely on its extension fields (GPS_RAW_INT accuracies and
# alt_ellipsoid, COMMAND_ACK.target_system, ...). pymavlink picks v1 or v2 from
# MAVLINK20 when mavutil is first imported, which may have happened already.
if mavutil.mavlink.WIRE_PROTOCOL_VERSION != "2.0":
    os.environ["MAVLINK20"] = "1"
    mavutil.set_dialect(mavutil.current_dialect)

mavlink = mavutil.mavlink

MAVLINK_MSG_ID_DISTANCE_SENSOR_MOUNT = 14000


class MAVLink_distance_sensor_mount_message(mavlink.MAVLink_message):
    """Distance sensor information for a camera mount/gimbal rangefinder."""

    id = MAVLINK_MSG_ID_DISTANCE_SENSOR_MOUNT
    msgname = "DISTANCE_SENSOR_MOUNT"
    fieldnames = [
        "time_boot_ms",
        "min_distance",
        "max_distance",
        "current_distance",
        "type",
        "id",
        "orientation",
        "covariance",
        "horizontal_fov",
        "vertical_fov",
        "quaternion",
        "signal_quality",
    ]
    ordered_fieldnames = fieldnames
    fieldtypes = [
        "uint32_t",
        "uint32_t",
        "uint32_t",
        "uint32_t",
        "uint8_t",
        "uint8_t",
        "uint8_t",
        "uint8_t",
        "float",
        "float",
        "float",
        "uint8_t",
    ]
    fielddisplays_by_name: dict[str, str] = {}
    fieldenums_by_name = {
        "type": "MAV_DISTANCE_SENSOR",
        "orientation": "MAV_SENSOR_ORIENTATION",
    }
    fieldunits_by_name = {
        "time_boot_ms": "ms",
        "min_distance": "cm",
        "max_distance": "cm",
        "current_distance": "cm",
        "covariance": "cm^2",
        "horizontal_fov": "rad",
        "vertical_fov": "rad",
        "signal_quality": "%",
    }
    native_format = bytearray(b"<IIIIBBBBfffB")
    orders = list(range(12))
    lengths = [1] * 10 + [4, 1]
    array_lengths = [0] * 10 + [4, 0]
    crc_extra = 95
    unpacker = struct.Struct("<IIIIBBBBff4fB")
    instance_field = "id"
    instance_offset = 17

    def __init__(
        self,
        time_boot_ms: int,
        min_distance: int,
        max_distance: int,
        current_distance: int,
        type: int,
        id: int,
        orientation: int,
        covariance: int,
        horizontal_fov: float = 0,
        vertical_fov: float = 0,
        quaternion=(0, 0, 0, 0),
        signal_quality: int = 0,
    ):
        mavlink.MAVLink_message.__init__(
            self, MAVLINK_MSG_ID_DISTANCE_SENSOR_MOUNT, self.msgname
        )
        self._fieldnames = self.fieldnames
        self._instance_field = self.instance_field
        self._instance_offset = self.instance_offset
        self.time_boot_ms = time_boot_ms
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.current_distance = current_distance
        self.type = type
        self.id = id
        self.orientation = orientation
        self.covariance = covariance
        self.horizontal_fov = horizontal_fov
        self.vertical_fov = vertical_fov
        self.quaternion = quaternion
        self.signal_quality = signal_quality

    def pack(self, mav, force_mavlink1: bool = False) -> bytes:
        return self._pack(
            mav,
            self.crc_extra,
            self.unpacker.pack(
                self.time_boot_ms,
                self.min_distance,
                self.max_distance,
                self.current_distance,
                self.type,
                self.id,
                self.orientation,
                self.covariance,
                self.horizontal_fov,
                self.vertical_fov,
                *self.quaternion,
                self.signal_quality,
            ),
            force_mavlink1=force_mavlink1,
        )


mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR_MOUNT = MAVLINK_MSG_ID_DISTANCE_SENSOR_MOUNT
mavlink.MAVLink_distance_sensor_mount_message = MAVLink_distance_sensor_mount_message
mavlink.mavlink_map[MAVLINK_MSG_ID_DISTANCE_SENSOR_MOUNT] = (
    MAVLink_distance_sensor_mount_message
)
