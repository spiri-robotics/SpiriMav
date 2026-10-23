"""MAVLink backend for :class:`~spiri_mav.robot.Robot`.

The robot hangs in the frame tree under the autopilot's EKF origin::

    ECEF ─▶ <robot>_origin ─▶ <robot> ─▶ <robot>_gimbal<id> ─▶ cameras, ...
     │          │                 │         └─ mount = GIMBAL_DEVICE_ATTITUDE_STATUS, in body FRD
     │          │                 └─ mount = LOCAL_POSITION_NED + ATTITUDE_QUATERNION
     │          ├─ mount = GPS_GLOBAL_ORIGIN, oriented as NED there
     │          └─▶ <robot>_home    mount = HOME_POSITION (local NED)
     └─▶ <robot>_roi    mount = the ROI we last set, in ECEF

so autopilot telemetry goes into mounts unconverted, and the part of the tree
below the origin works without GPS. The origin, home and ROI are sibling topics,
not ``<robot>/origin``: an object's subscriber on ``<robot>/**`` would
otherwise receive every put of the objects nested under it.

Safety rules from ``docs/design.md`` that this module enforces: the companion
heartbeats as its own system and refuses to heartbeat or command if that
sysid is the FCU's GCS sysid; ``failsafe`` only mirrors what the FCU reports;
``gcs_seen`` is display-only.
"""

from __future__ import annotations

import math
import os
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Literal

from loguru import logger
from pymavlink import mavutil
from SpiriSynq.example_types.position import (
    Accuracy,
    Offset,
    Orientation,
    Pose,
    Position,
    RootFrame,
)
from SpiriSynq.link import Link

from spiri_mav.gimbal import Gimbal
from spiri_mav.robot import (
    Battery,
    BatteryState,
    CommandRefused,
    Failsafe,
    GpsFix,
    GpsFixType,
    Robot,
    Velocity,
)

# MAVLink 2: we rely on its extension fields (GPS_RAW_INT accuracies and
# alt_ellipsoid, COMMAND_ACK.target_system, ...). pymavlink picks v1 or v2 from
# MAVLINK20 when mavutil is first imported, which may have happened already.
if mavutil.mavlink.WIRE_PROTOCOL_VERSION != "2.0":
    os.environ["MAVLINK20"] = "1"
    mavutil.set_dialect(mavutil.current_dialect)

mavlink = mavutil.mavlink

HEARTBEAT_TIMEOUT = 3.0  # s without an autopilot heartbeat before `connected` drops
GCS_TIMEOUT = 5.0  # s without another GCS's heartbeat before `gcs_seen` drops
COMMAND_TIMEOUT = 1.5  # s to wait for a COMMAND_ACK, per attempt
COMMAND_ATTEMPTS = 3
RECONNECT_DELAY = 1.0  # s before reconnecting a lost link; doubles while it stays down
RECONNECT_MAX_DELAY = 10.0

# What we ask the autopilot to stream, in Hz.
STREAMS = {
    mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 10,
    mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION: 10,
    mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 2,
    mavlink.MAVLINK_MSG_ID_BATTERY_STATUS: 1,
    mavlink.MAVLINK_MSG_ID_HOME_POSITION: 0.2,
    mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN: 0.2,
    mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS: 10,
    mavlink.MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS: 1,
}

# The FCU's GCS sysid: ArduPilot's old and new names, PX4 doesn't have one.
GCS_SYSID_PARAMS = ("SYSID_MYGCS", "MAV_GCS_SYSID")

_FIX_TYPES: dict[int, GpsFixType] = {
    mavlink.GPS_FIX_TYPE_2D_FIX: "2d",
    mavlink.GPS_FIX_TYPE_3D_FIX: "3d",
    mavlink.GPS_FIX_TYPE_DGPS: "dgps",
    mavlink.GPS_FIX_TYPE_RTK_FLOAT: "rtk_float",
    mavlink.GPS_FIX_TYPE_RTK_FIXED: "rtk_fixed",
    mavlink.GPS_FIX_TYPE_STATIC: "3d",
    mavlink.GPS_FIX_TYPE_PPP: "3d",
}

_BATTERY_STATES: dict[int, BatteryState] = {
    mavlink.MAV_BATTERY_CHARGE_STATE_OK: "ok",
    mavlink.MAV_BATTERY_CHARGE_STATE_LOW: "low",
    mavlink.MAV_BATTERY_CHARGE_STATE_CRITICAL: "critical",
    mavlink.MAV_BATTERY_CHARGE_STATE_EMERGENCY: "emergency",
    mavlink.MAV_BATTERY_CHARGE_STATE_FAILED: "failed",
    mavlink.MAV_BATTERY_CHARGE_STATE_UNHEALTHY: "unhealthy",
}

_CRITICAL_STATES = (mavlink.MAV_STATE_CRITICAL, mavlink.MAV_STATE_EMERGENCY)

# Classifying failsafe STATUSTEXTs. Neither autopilot publishes a failsafe
# bitmask, so this is a heuristic: whether *a* failsafe is active comes from
# HEARTBEAT.system_status, and the text only says which. ArduPilot and PX4 word
# these differently and change them between releases -- verify in SITL.
_FAILSAFE_WORDS = re.compile(r"\b(failsafe|lost|breach|violation)\b")
_CLEARED_WORDS = re.compile(r"\b(cleared|regained|resolved)\b")
_FAILSAFE_KINDS: tuple[tuple[re.Pattern, Failsafe], ...] = (
    (re.compile(r"\b(gcs|data link|ground station)\b"), "gcs"),
    (re.compile(r"\b(radio|rc|manual control)\b"), "rc"),
    (re.compile(r"\bbattery\b"), "battery"),
    (re.compile(r"\b(geo)?fence\b"), "geofence"),
    (re.compile(r"\b(ekf|estimator|navigation)\b"), "ekf"),
)


def classify_failsafe_text(text: str) -> tuple[Failsafe, bool] | None:
    """(kind, cleared) for a failsafe STATUSTEXT, or None if it isn't one."""
    lowered = text.lower()
    if not _FAILSAFE_WORDS.search(lowered):
        return None
    kind: Failsafe = next(
        (k for pattern, k in _FAILSAFE_KINDS if pattern.search(lowered)), "other"
    )
    return kind, bool(_CLEARED_WORDS.search(lowered))


def _enum_name(enum: str, value: int, prefix: str) -> str:
    entry = mavlink.enums[enum].get(value)
    return entry.name.removeprefix(prefix).lower() if entry else str(value)


class _LinkLost(ConnectionError):
    """The FCU link closed; :meth:`MavRobot.run` reconnects it."""


def _open_link(url: str, sysid: int, compid: int):
    """A pymavlink connection to *url* that raises :class:`_LinkLost` when it closes.

    pymavlink's TCP link, without autoreconnect, meets EOF by printing and
    returning nothing; the dead socket then always polls readable, so
    recv_match spins a core forever. With autoreconnect it sleeps and retries
    inside recv, out of our hands. So: one connect attempt, and EOF raises,
    for :meth:`MavRobot.run` to retry at its own pace. A reset needs nothing:
    recv raises it anyway, and a write to a dead link, from whichever thread,
    is dropped.
    """
    conn = mavutil.mavlink_connection(
        url, source_system=sysid, source_component=compid, retries=0
    )

    def eof():
        raise _LinkLost("link closed by peer")

    if hasattr(conn, "handle_eof"):  # stream links only
        conn.handle_eof = eof
        conn.handle_disconnect = lambda: None
    return conn


@dataclass
class _AckWaiter:
    event: threading.Event = field(default_factory=threading.Event)
    result: int | None = None


def _inverse(q: Orientation) -> Orientation:
    q = q.normalize()
    return Orientation(q.w, -q.x, -q.y, -q.z)


def _wrap180(degrees: float) -> float:
    return (degrees + 180) % 360 - 180


@dataclass
class MavGimbal(Gimbal):
    """A gimbal behind the autopilot's gimbal manager (MAVLink Gimbal Protocol v2).

    Its MavRobot creates it on the first message about it and feeds it
    messages. Commands go to the gimbal manager on the autopilot through the
    robot's link. Pointing takes primary control from whoever has it: there's
    no arbitration, as with any two GCSs, and QGC can take it back.
    """

    device_id: int = 0
    primary_control: tuple[int, int] | None = (
        None  # (sysid, compid) in control; display only
    )

    _robot: MavRobot | None = None

    def handle(self, msg) -> None:
        kind = msg.get_type()
        if kind == "GIMBAL_DEVICE_ATTITUDE_STATUS":
            self._on_attitude(msg)
        elif kind == "GIMBAL_MANAGER_STATUS":
            in_control = (msg.primary_control_sysid, msg.primary_control_compid)
            self.primary_control = in_control if msg.primary_control_sysid else None
        elif kind == "GIMBAL_MANAGER_INFORMATION":
            self.pitch_range = (
                math.degrees(msg.pitch_min),
                math.degrees(msg.pitch_max),
            )
            self.yaw_range = (math.degrees(msg.yaw_min), math.degrees(msg.yaw_max))

    def _on_attitude(self, msg) -> None:
        body = self._robot._attitude if self._robot is not None else None
        if body is None or any(math.isnan(c) for c in msg.q):
            return  # can't place it in the robot's body frame yet

        # Roll and pitch are from the horizon; yaw is from north or from the
        # vehicle's heading, depending on the flags.
        flags = msg.flags
        if flags & mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME:
            earth_yaw = True
        elif flags & mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME:
            earth_yaw = False
        else:
            earth_yaw = bool(
                flags & mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK
            )  # devices predating those flags
        q = Orientation(*msg.q)
        in_ned = q if earth_yaw else Orientation.from_euler(0, 0, body.yaw) * q

        _, pitch, yaw = in_ned.to_euler(degrees=True)
        self.pitch = pitch
        self.yaw = _wrap180(yaw - math.degrees(body.yaw))
        self.yaw_lock = bool(flags & mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK)
        offset = self.mount.offset if self.mount is not None else Offset()
        self.mount = Pose(offset, _inverse(body) * in_ned)

    def _robot_or_refuse(self) -> MavRobot:
        if self._robot is None:
            raise CommandRefused("gimbal isn't attached to a robot")
        return self._robot

    def _take_control(self) -> None:
        robot = self._robot_or_refuse()
        if self.primary_control == (robot.sysid, robot.compid):
            return
        robot._command(
            mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
            (robot.sysid, robot.compid, -1, -1, 0, 0, self.device_id),
        )

    def _do_point(self, pitch: float, yaw: float, yaw_lock: bool) -> None:
        flags = (
            mavlink.GIMBAL_MANAGER_FLAGS_ROLL_LOCK
            | mavlink.GIMBAL_MANAGER_FLAGS_PITCH_LOCK
        )
        if yaw_lock:
            flags |= mavlink.GIMBAL_MANAGER_FLAGS_YAW_LOCK
        self._take_control()
        self._robot_or_refuse()._command(
            mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            (pitch, yaw, math.nan, math.nan, flags, 0, self.device_id),
        )

    def _do_point_at(self, target: Offset) -> None:
        robot = self._robot_or_refuse()
        lat, lon, height = target.to_geodetic()
        self._take_control()
        robot._command(
            mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            (self.device_id,),
            int_frame=mavlink.MAV_FRAME_GLOBAL,
            x=round(lat * 1e7),
            y=round(lon * 1e7),
            z=height - robot._separation(),
        )

    def _do_neutral(self) -> None:
        self._take_control()
        self._robot_or_refuse()._command(
            mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            (
                math.nan,
                math.nan,
                math.nan,
                math.nan,
                mavlink.GIMBAL_MANAGER_FLAGS_NEUTRAL,
                0,
                self.device_id,
            ),
        )

    def _do_release(self) -> None:
        robot = self._robot_or_refuse()
        robot._command(mavlink.MAV_CMD_DO_SET_ROI_NONE, (self.device_id,))
        if self.primary_control == (robot.sysid, robot.compid):
            robot._command(
                mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                (-3, -3, -1, -1, 0, 0, self.device_id),
            )


@dataclass
class MavRobot(Robot):
    """A Robot backed by a MAVLink autopilot, from the companion computer.

    Feed it every message from the FCU link with :meth:`handle` and call
    :meth:`tick` regularly, or let :meth:`run` do both.
    """

    autopilot: Literal["ardupilot", "px4", "unknown"] = "unknown"
    vehicle_type: str = ""  # MAV_TYPE, e.g. "quadrotor"
    system_status: str = ""  # MAV_STATE, e.g. "standby", "active", "critical"
    sysid: int = 254  # ours; must never be the FCU's GCS sysid
    compid: int = mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
    target_sysid: int = 1
    target_compid: int = mavlink.MAV_COMP_ID_AUTOPILOT1
    gcs_seen: bool = False  # display only: never gate anything on it
    last_statustext: str | None = None

    _conn: Any = None  # pymavlink connection
    _origin: Position | None = None
    _home: Position | None = None
    _roi: Position | None = None
    _gimbals: dict[int, MavGimbal] = field(default_factory=dict)  # by gimbal device id
    _gimbal_cameras: dict[int, tuple[Link, ...]] = field(
        default_factory=dict
    )  # config: MAVLink doesn't say
    _mav_type: int | None = None
    _local: Offset | None = None
    _attitude: Orientation | None = None
    _accuracy: Accuracy | None = None
    _origin_geo: tuple[float, float, float] | None = None  # lat°, lon°, m AMSL
    _geoid_separation: float | None = None  # ellipsoid - MSL, m
    _home_amsl: float | None = None
    _failsafe_kinds: frozenset[Failsafe] = frozenset()
    _failsafe_text: str | None = None
    _was_critical: bool = False
    _sysid_conflict: bool = False
    _last_fcu_heartbeat: float = 0.0
    _last_gcs_heartbeat: float = 0.0
    _last_heartbeat_sent: float = 0.0
    _pending: dict[int, list[_AckWaiter]] = field(default_factory=dict)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def __post_init__(self):
        super().__post_init__()
        if not self.synq_authoritive:
            return
        self._origin = Position(
            f"{self.synq_topic}_origin",
            synq_authoritive=True,
            relative_to=RootFrame("ECEF"),
        )
        self._home = Position(
            f"{self.synq_topic}_home",
            synq_authoritive=True,
            relative_to=Link.to(self._origin),
        )
        self.home = Link.to(self._home)
        self._roi = Position(
            f"{self.synq_topic}_roi",
            synq_authoritive=True,
            relative_to=RootFrame("ECEF"),
        )
        self.relative_to = Link.to(self._origin)

    def close(self):
        for child in (*self._gimbals.values(), self._home, self._roi, self._origin):
            if child is not None:
                child.close()
        super().close()

    # ── Link loop ─────────────────────────────────────────────────────────────

    def run(self, url: str, stop: threading.Event | None = None) -> None:
        """Connect to the FCU at *url* (a pymavlink connection string) and serve until *stop*.

        A link that fails or closes is reconnected, after RECONNECT_DELAY,
        doubling up to RECONNECT_MAX_DELAY while it stays down.
        """
        stop = stop or threading.Event()
        delay = RECONNECT_DELAY
        while not stop.is_set():
            try:
                self._conn = _open_link(url, self.sysid, self.compid)
                logger.info(
                    "MAVLink on {} as sysid {} compid {}", url, self.sysid, self.compid
                )
                while not stop.is_set():
                    msg = self._conn.recv_match(blocking=True, timeout=0.1)
                    if msg is not None:
                        # The link works, so a later loss retries promptly.
                        delay = RECONNECT_DELAY
                        self.handle(msg)
                    self.tick()
            except (_LinkLost, OSError) as e:
                self._drop_link()
                logger.warning(
                    "MAVLink on {}: {}; reconnecting in {:g} s", url, e, delay
                )
                stop.wait(delay)
                delay = min(delay * 2, RECONNECT_MAX_DELAY)
        self._drop_link()

    def _drop_link(self) -> None:
        # The closed connection stays as _conn: writes to it are dropped, and
        # commands are refused while not connected.
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
        if self.connected:
            logger.warning("Lost autopilot link")
            self.connected = False

    def tick(self, now: float | None = None) -> None:
        """Send our heartbeat and time out stale links."""
        now = time.monotonic() if now is None else now
        if now - self._last_heartbeat_sent >= 1.0:
            self._last_heartbeat_sent = now
            self._send_heartbeat()
        if self.connected and now - self._last_fcu_heartbeat > HEARTBEAT_TIMEOUT:
            logger.warning("Lost autopilot heartbeat")
            self.connected = False
        if self.gcs_seen and now - self._last_gcs_heartbeat > GCS_TIMEOUT:
            self.gcs_seen = False

    def handle(self, msg) -> None:
        """Apply one MAVLink message."""
        kind = msg.get_type()
        if kind == "HEARTBEAT":
            self._on_heartbeat(msg)
            return
        if msg.get_srcSystem() != self.target_sysid:
            return
        if kind.startswith("GIMBAL_"):
            # A gimbal device may be a component of its own, not the autopilot.
            self._on_gimbal_message(msg)
            return
        if msg.get_srcComponent() != self.target_compid:
            return
        handler = getattr(self, f"_on_{kind.lower()}", None)
        if handler is not None:
            handler(msg)

    def _on_gimbal_message(self, msg) -> None:
        # Device id 0 means "the sender is the gimbal device".
        device_id = getattr(msg, "gimbal_device_id", 0) or msg.get_srcComponent()
        gimbal = self._gimbals.get(device_id)
        if gimbal is None:
            if not self.synq_authoritive:
                return
            gimbal = MavGimbal(
                f"{self.synq_topic}_gimbal{device_id}",
                synq_authoritive=True,
                relative_to=Link.to(self),
                device_id=device_id,
                _robot=self,
                cameras=self._gimbal_cameras.get(device_id, ()),
            )
            self._gimbals[device_id] = gimbal
            self.gimbals = tuple(Link.to(g) for _, g in sorted(self._gimbals.items()))
        gimbal.handle(msg)

    # ── Incoming messages ─────────────────────────────────────────────────────

    def _on_heartbeat(self, msg) -> None:
        now = time.monotonic()
        if msg.type == mavlink.MAV_TYPE_GCS:
            if msg.get_srcSystem() != self.sysid:
                self._last_gcs_heartbeat = now
                self.gcs_seen = True
            return
        if (msg.get_srcSystem(), msg.get_srcComponent()) != (
            self.target_sysid,
            self.target_compid,
        ):
            return
        if msg.autopilot == mavlink.MAV_AUTOPILOT_INVALID:
            return

        self._last_fcu_heartbeat = now
        if not self.connected:
            self.connected = True
            self._on_connect()

        self.autopilot = {
            mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA: "ardupilot",
            mavlink.MAV_AUTOPILOT_PX4: "px4",
        }.get(msg.autopilot, "unknown")
        if msg.type != self._mav_type:
            self._mav_type = msg.type
            self.vehicle_type = _enum_name("MAV_TYPE", msg.type, "MAV_TYPE_")
            self.available_modes = self._modes()
        self.armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.mode = mavutil.mode_string_v10(msg)
        self.system_status = _enum_name("MAV_STATE", msg.system_status, "MAV_STATE_")

        critical = msg.system_status in _CRITICAL_STATES
        if self._was_critical and not critical:
            self._failsafe_kinds = frozenset()  # the FCU says it's over
            self._failsafe_text = None
        self._was_critical = critical
        self._publish_failsafe()

    def _on_statustext(self, msg) -> None:
        text = msg.text.rstrip("\x00")
        self.last_statustext = text
        classified = classify_failsafe_text(text)
        if classified is None:
            return
        kind, cleared = classified
        self._failsafe_kinds = (
            self._failsafe_kinds - {kind} if cleared else self._failsafe_kinds | {kind}
        )
        if not cleared:
            self._failsafe_text = text
        self._publish_failsafe()

    def _publish_failsafe(self) -> None:
        # Text can arrive before the CRITICAL heartbeat; hold it until the FCU confirms.
        if self._was_critical:
            self.failsafe = tuple(sorted(self._failsafe_kinds or {"other"}))
            self.failsafe_text = self._failsafe_text
        else:
            self.failsafe = ()
            self.failsafe_text = None

    def _on_local_position_ned(self, msg) -> None:
        self._local = Offset(msg.x, msg.y, msg.z)
        self.velocity = Velocity(msg.vx, msg.vy, msg.vz)
        self._update_mount()

    def _on_attitude_quaternion(self, msg) -> None:
        # Body (FRD) to local NED -- the tree's convention as it stands.
        self._attitude = Orientation(msg.q1, msg.q2, msg.q3, msg.q4)
        self._update_mount()

    def _update_mount(self) -> None:
        if self._local is None:
            return
        self.mount = Pose(self._local, self._attitude or Orientation(), self._accuracy)

    def _on_gps_raw_int(self, msg) -> None:
        self.gps = GpsFix(
            fix=_FIX_TYPES.get(msg.fix_type, "none"),
            satellites=None
            if msg.satellites_visible == 255
            else msg.satellites_visible,
            hdop=None if msg.eph == 65535 else msg.eph / 100,
        )
        # h_acc/v_acc are roughly 1-sigma; Accuracy is CEP90/LEP90. A stand-in
        # for the EKF's own accuracy, which MAVLink doesn't report in metres.
        h_acc = getattr(msg, "h_acc", 0) or None
        v_acc = getattr(msg, "v_acc", 0) or None
        self._accuracy = Accuracy.from_cot(
            h_acc and h_acc / 1000, v_acc and v_acc / 1000
        )

        alt_ellipsoid = getattr(msg, "alt_ellipsoid", 0)
        if alt_ellipsoid and msg.fix_type >= mavlink.GPS_FIX_TYPE_3D_FIX:
            separation = (alt_ellipsoid - msg.alt) / 1000
            if (
                self._geoid_separation is None
                or abs(separation - self._geoid_separation) > 0.5
            ):
                self._geoid_separation = separation
                self._update_origin()

    def _on_gps_global_origin(self, msg) -> None:
        self._origin_geo = (
            msg.latitude / 1e7,
            msg.longitude / 1e7,
            msg.altitude / 1000,
        )
        self._update_origin()

    def _update_origin(self) -> None:
        if self._origin_geo is None or self._origin is None:
            return
        lat, lon, amsl = self._origin_geo
        self._origin.mount = Pose(
            Offset.from_geodetic(lat, lon, amsl + self._separation()),
            Orientation.ned_to_ecef(lat, lon),
        )

    def _separation(self) -> float:
        # Without it, MSL heights stand in for ellipsoid heights. Targets taken
        # from this robot's own tree still round-trip exactly; targets from
        # elsewhere are off vertically by the local geoid separation.
        return self._geoid_separation if self._geoid_separation is not None else 0.0

    def _on_home_position(self, msg) -> None:
        self._home_amsl = msg.altitude / 1000
        if self._home is not None:
            self._home.mount = Pose(Offset(msg.x, msg.y, msg.z))

    def _on_battery_status(self, msg) -> None:
        cells = [v for v in msg.voltages if v != 65535]
        battery = Battery(
            id=msg.id,
            voltage=sum(cells) / 1000 if cells else None,
            current=None if msg.current_battery == -1 else msg.current_battery / 100,
            remaining=None
            if msg.battery_remaining == -1
            else msg.battery_remaining / 100,
            state=_BATTERY_STATES.get(getattr(msg, "charge_state", 0), "unknown"),
        )
        others = tuple(b for b in self.batteries if b.id != battery.id)
        self.batteries = tuple(sorted(others + (battery,), key=lambda b: b.id))

    def _on_param_value(self, msg) -> None:
        name = msg.param_id.rstrip("\x00")
        if name in GCS_SYSID_PARAMS and int(msg.param_value) == self.sysid:
            logger.critical(
                "Our sysid {} is the FCU's GCS sysid ({}): our heartbeat would mask a lost QGC. "
                "Not heartbeating or commanding until restarted with another sysid.",
                self.sysid,
                name,
            )
            self._sysid_conflict = True

    def _on_command_ack(self, msg) -> None:
        target = getattr(msg, "target_system", 0)
        if target not in (0, self.sysid):
            return  # QGC's
        if msg.result == mavlink.MAV_RESULT_IN_PROGRESS:
            return
        with self._lock:
            waiters = self._pending.pop(msg.command, [])
        for waiter in waiters:
            waiter.result = msg.result
            waiter.event.set()

    # ── Outgoing messages ─────────────────────────────────────────────────────

    def _on_connect(self) -> None:
        logger.info("Autopilot sysid {} connected", self.target_sysid)
        self._show_roi(None)  # it may have rebooted, and MAVLink can't tell us its ROI
        for name in GCS_SYSID_PARAMS:
            self._conn.mav.param_request_read_send(
                self.target_sysid, self.target_compid, name.encode(), -1
            )
        for message_id, hz in STREAMS.items():
            self._send_command_long(
                mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, (message_id, 1e6 / hz)
            )
        self._send_command_long(
            mavlink.MAV_CMD_REQUEST_MESSAGE,
            (mavlink.MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION,),
        )

    def _send_heartbeat(self) -> None:
        if self._conn is None or self._sysid_conflict:
            return
        self._conn.mav.heartbeat_send(
            mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavlink.MAV_STATE_ACTIVE,
        )

    def _send_command_long(
        self, command: int, params=(), confirmation: int = 0
    ) -> None:
        p = (tuple(params) + (0.0,) * 7)[:7]
        self._conn.mav.command_long_send(
            self.target_sysid, self.target_compid, command, confirmation, *p
        )

    def _command(
        self,
        command: int,
        params=(),
        *,
        int_frame: int | None = None,
        x: int = 0,
        y: int = 0,
        z: float = 0.0,
    ) -> None:
        """Send a command and wait for its ACK; raise CommandRefused unless accepted.

        Blocks, so never call it from the thread that runs :meth:`handle`.
        With *int_frame*, sends COMMAND_INT (params 1-4, then x/y/z).
        """
        name = _enum_name("MAV_CMD", command, "MAV_CMD_")
        if self._sysid_conflict:
            raise CommandRefused(f"{name}: our sysid is the FCU's GCS sysid")
        if self._conn is None or not self.connected:
            raise CommandRefused(f"{name}: autopilot not connected")

        waiter = _AckWaiter()
        with self._lock:
            self._pending.setdefault(command, []).append(waiter)
        try:
            for attempt in range(COMMAND_ATTEMPTS):
                if int_frame is None:
                    self._send_command_long(command, params, confirmation=attempt)
                else:
                    p = (tuple(params) + (0.0,) * 4)[:4]
                    self._conn.mav.command_int_send(
                        self.target_sysid,
                        self.target_compid,
                        int_frame,
                        command,
                        0,
                        0,
                        *p,
                        x,
                        y,
                        z,
                    )
                if waiter.event.wait(COMMAND_TIMEOUT):
                    break
        finally:
            with self._lock:
                waiters = self._pending.get(command, [])
                if waiter in waiters:
                    waiters.remove(waiter)

        if waiter.result is None:
            raise CommandRefused(f"{name}: no COMMAND_ACK")
        if waiter.result != mavlink.MAV_RESULT_ACCEPTED:
            raise CommandRefused(
                f"{name}: {_enum_name('MAV_RESULT', waiter.result, 'MAV_RESULT_')}"
            )

    def _modes(self) -> tuple[str, ...]:
        if self.autopilot == "px4":
            return tuple(mavutil.px4_map)
        if self.autopilot == "ardupilot" and self._mav_type is not None:
            return tuple(mavutil.mode_mapping_byname(self._mav_type) or ())
        return ()

    # ── Robot hooks ───────────────────────────────────────────────────────────

    def _do_arm(self, arm: bool, force: bool) -> None:
        self._command(
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            (1 if arm else 0, 21196 if force else 0),
        )

    def _do_set_mode(self, mode: str) -> None:
        if self.autopilot == "px4":
            base, main, sub = mavutil.px4_map[mode]
            self._command(mavlink.MAV_CMD_DO_SET_MODE, (base, main, sub))
        else:
            number = (mavutil.mode_mapping_byname(self._mav_type) or {}).get(mode)
            if number is None:
                raise CommandRefused(f"Unknown mode {mode!r}")
            self._command(
                mavlink.MAV_CMD_DO_SET_MODE,
                (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, number),
            )

    def _do_takeoff(self, altitude: float) -> None:
        nan = math.nan
        if self.autopilot == "px4":
            # PX4 takes AMSL; ArduPilot takes metres above home.
            if self._home_amsl is None:
                raise CommandRefused("takeoff: home position unknown")
            self._command(
                mavlink.MAV_CMD_NAV_TAKEOFF,
                (nan, 0, 0, nan, nan, nan, self._home_amsl + altitude),
            )
        else:
            self._command(mavlink.MAV_CMD_NAV_TAKEOFF, (0, 0, 0, 0, 0, 0, altitude))

    def _do_goto(self, target: Offset) -> None:
        lat, lon, height = target.to_geodetic()
        self._command(
            mavlink.MAV_CMD_DO_REPOSITION,
            (-1, mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0, math.nan),
            int_frame=mavlink.MAV_FRAME_GLOBAL,
            x=round(lat * 1e7),
            y=round(lon * 1e7),
            z=height - self._separation(),
        )

    def _do_land(self) -> None:
        nan = math.nan
        self._command(mavlink.MAV_CMD_NAV_LAND, (0, 0, 0, nan, nan, nan, nan))

    def _do_return_home(self) -> None:
        self._command(mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def _do_set_roi(self, target: Offset | None) -> None:
        # Gimbal device id 0: all of them. ArduPilot Copter also yaws to face
        # the ROI when no gimbal can pan.
        if target is None:
            self._command(mavlink.MAV_CMD_DO_SET_ROI_NONE, (0,))
        else:
            lat, lon, height = target.to_geodetic()
            self._command(
                mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                (0,),
                int_frame=mavlink.MAV_FRAME_GLOBAL,
                x=round(lat * 1e7),
                y=round(lon * 1e7),
                z=height - self._separation(),
            )
        self._show_roi(target)

    def _show_roi(self, target: Offset | None) -> None:
        # What we last set, not what the FCU is doing: no message reports the
        # active ROI, so one set by QGC, a mission or Gimbal.point_at isn't shown.
        if self._roi is None:
            return
        self._roi.mount = None if target is None else Pose(target)
        self.roi = None if target is None else Link.to(self._roi)
