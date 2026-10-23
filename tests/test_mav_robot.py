"""Tests for MavRobot: MAVLink messages in, Robot fields and commands out."""

from __future__ import annotations

import math
import socket
import threading
import time

import pytest
from pymavlink import mavutil
from SpiriSynq.example_types.position import Offset, Orientation
from SpiriSynq.link import Link

from spiri_mav.gimbal import Gimbal
from spiri_mav.mav_robot import MavRobot, classify_failsafe_text
from spiri_mav.robot import Battery, CommandRefused, GpsFix, Robot, Velocity

mavlink = mavutil.mavlink

FCU = (1, 1)


def _wait_for(predicate, timeout=3.0, interval=0.01):
    start = time.time()
    while time.time() - start < timeout:
        if predicate():
            return True
        time.sleep(interval)
    return False


def from_src(name: str, *args, src=FCU):
    """A decoded MAVLink message, as if received from *src* (sysid, compid)."""
    sender = mavlink.MAVLink(None, srcSystem=src[0], srcComponent=src[1])
    packed = getattr(sender, f"{name}_encode")(*args).pack(sender)
    return mavlink.MAVLink(None).decode(bytearray(packed))


class FakeLink:
    """Stands in for a pymavlink connection; records what we send."""

    def __init__(self, sysid=254, compid=191):
        self.sent: list = []
        self._parser = mavlink.MAVLink(None)
        self.mav = mavlink.MAVLink(self, srcSystem=sysid, srcComponent=compid)

    def write(self, data: bytes) -> None:
        self.sent.append(self._parser.decode(bytearray(data)))

    def of_type(self, name: str) -> list:
        return [m for m in self.sent if m.get_type() == name]


def heartbeat(autopilot=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, custom_mode=4, armed=False,
              status=mavlink.MAV_STATE_STANDBY, mav_type=mavlink.MAV_TYPE_QUADROTOR, src=FCU):
    base = mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0)
    return from_src("heartbeat", mav_type, autopilot, base, custom_mode, status, src=src)


@pytest.fixture
def robot():
    r = MavRobot("test/robot", synq_authoritive=True)
    r._conn = FakeLink(r.sysid, r.compid)
    yield r
    r.close()


@pytest.fixture
def connected(robot):
    robot.handle(heartbeat())
    return robot


def ack_when_sent(robot: MavRobot, command: int, result=mavlink.MAV_RESULT_ACCEPTED):
    """Answer the next *command* we send, from the FCU, as the reader thread would."""
    def responder():
        assert _wait_for(lambda: any(m.command == command for m in
                                     robot._conn.of_type("COMMAND_LONG") + robot._conn.of_type("COMMAND_INT")))
        robot.handle(from_src("command_ack", command, result, 0, 0, robot.sysid, robot.compid))
    thread = threading.Thread(target=responder)
    thread.start()
    return thread


# ── Heartbeat and state ──────────────────────────────────────────────────────

def test_heartbeat_sets_state_and_requests_streams(robot):
    robot.handle(heartbeat(custom_mode=4, armed=True, status=mavlink.MAV_STATE_ACTIVE))
    assert robot.connected
    assert robot.autopilot == "ardupilot"
    assert robot.vehicle_type == "quadrotor"
    assert robot.armed
    assert robot.mode == "GUIDED"
    assert robot.system_status == "active"
    assert "RTL" in robot.available_modes

    intervals = [m for m in robot._conn.of_type("COMMAND_LONG")
                 if m.command == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL]
    assert {int(m.param1) for m in intervals} >= {mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED}
    assert {m.param_id for m in robot._conn.of_type("PARAM_REQUEST_READ")} == {"SYSID_MYGCS", "MAV_GCS_SYSID"}


def test_px4_mode_names(robot):
    custom_mode = (4 << 24) | (4 << 16)  # AUTO.MISSION
    robot.handle(heartbeat(autopilot=mavlink.MAV_AUTOPILOT_PX4, custom_mode=custom_mode))
    assert robot.autopilot == "px4"
    assert robot.mode == "MISSION"
    assert robot.mode in robot.available_modes


def test_other_systems_are_ignored(robot):
    robot.handle(heartbeat(src=(2, 1)))
    robot.handle(from_src("local_position_ned", 0, 1, 2, 3, 0, 0, 0, src=(2, 1)))
    assert not robot.connected
    assert robot.mount is None


def test_heartbeat_timeout(connected):
    connected.tick(now=time.monotonic() + 10)
    assert not connected.connected


def test_gcs_seen_is_display_only(connected):
    connected.handle(heartbeat(mav_type=mavlink.MAV_TYPE_GCS, autopilot=mavlink.MAV_AUTOPILOT_INVALID,
                               src=(255, 190)))
    assert connected.gcs_seen
    connected.tick(now=time.monotonic() + 10)
    assert not connected.gcs_seen


def test_tick_sends_our_heartbeat(robot):
    robot.tick()
    [hb] = robot._conn.of_type("HEARTBEAT")
    assert hb.get_srcSystem() == robot.sysid
    assert hb.type == mavlink.MAV_TYPE_ONBOARD_CONTROLLER


# ── The link ─────────────────────────────────────────────────────────────────

class FakeAutopilot:
    """A TCP port standing in for SITL's serial0: it takes a client and can drop it."""

    def __init__(self):
        self.listener = socket.create_server(("127.0.0.1", 0))
        self.listener.settimeout(5)
        self.url = f"tcp:127.0.0.1:{self.listener.getsockname()[1]}"

    def accept(self) -> socket.socket:
        client, _ = self.listener.accept()
        return client

    @staticmethod
    def heartbeat() -> bytes:
        sender = mavlink.MAVLink(None, srcSystem=FCU[0], srcComponent=FCU[1])
        return sender.heartbeat_encode(
            mavlink.MAV_TYPE_QUADROTOR, mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, mavlink.MAV_STATE_STANDBY,
        ).pack(sender)


@pytest.fixture
def running(monkeypatch):
    """A MavRobot running against a FakeAutopilot, in its own thread."""
    monkeypatch.setattr("spiri_mav.mav_robot.RECONNECT_DELAY", 0.05)
    fcu = FakeAutopilot()
    robot = MavRobot("test/linked", synq_authoritive=True)
    stop = threading.Event()
    thread = threading.Thread(target=robot.run, args=(fcu.url, stop), daemon=True)
    thread.start()
    yield robot, fcu
    stop.set()
    thread.join(timeout=5)
    assert not thread.is_alive()
    fcu.listener.close()
    robot.close()


def test_closed_link_drops_connected_and_reconnects(running):
    robot, fcu = running
    first = fcu.accept()
    first.sendall(fcu.heartbeat())
    assert _wait_for(lambda: robot.connected)
    first.close()
    # At once, not after HEARTBEAT_TIMEOUT.
    assert _wait_for(lambda: not robot.connected, timeout=1.0)
    second = fcu.accept()
    second.sendall(fcu.heartbeat())
    assert _wait_for(lambda: robot.connected)
    second.close()


def test_closed_link_does_not_spin(running):
    # pymavlink alone polls a closed TCP socket in a tight loop, a whole core.
    robot, fcu = running
    fcu.accept().close()
    fcu.listener.close()  # and nothing to reconnect to
    cpu, wall = time.process_time(), time.monotonic()
    time.sleep(1.0)
    used = (time.process_time() - cpu) / (time.monotonic() - wall)
    assert used < 0.3, f"{used:.0%} of a core while the link is down"


# ── Failsafe ─────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("text, expected", [
    ("Radio Failsafe", ("rc", False)),
    ("Radio Failsafe Cleared", ("rc", True)),
    ("GCS Failsafe On", ("gcs", False)),
    ("Failsafe activated: connection to ground station lost", ("gcs", False)),
    ("Battery Failsafe", ("battery", False)),
    ("Fence Breach", ("geofence", False)),
    ("EKF Failsafe", ("ekf", False)),
    ("Failsafe: something new", ("other", False)),
    ("Arming checks passed", None),
])
def test_classify_failsafe_text(text, expected):
    assert classify_failsafe_text(text) == expected


def test_failsafe_follows_system_status(connected):
    connected.handle(from_src("statustext", mavlink.MAV_SEVERITY_CRITICAL, b"Radio Failsafe"))
    assert connected.failsafe == ()  # not until the FCU says it's critical

    connected.handle(heartbeat(status=mavlink.MAV_STATE_CRITICAL))
    assert connected.failsafe == ("rc",)
    assert connected.failsafe_text == "Radio Failsafe"

    connected.handle(heartbeat(status=mavlink.MAV_STATE_ACTIVE))
    assert connected.failsafe == ()
    assert connected.failsafe_text is None


def test_unexplained_critical_is_other(connected):
    connected.handle(heartbeat(status=mavlink.MAV_STATE_CRITICAL))
    assert connected.failsafe == ("other",)


# ── Telemetry and the frame tree ─────────────────────────────────────────────

def test_local_position_and_attitude_make_the_mount(connected):
    connected.handle(from_src("local_position_ned", 0, 1.0, 2.0, -3.0, 0.5, 0, -0.1))
    q = Orientation.from_euler(0, 0, math.pi / 2)
    connected.handle(from_src("attitude_quaternion", 0, q.w, q.x, q.y, q.z, 0, 0, 0))
    assert connected.mount.offset == Offset(1.0, 2.0, -3.0)
    assert math.isclose(connected.mount.orientation.yaw, math.pi / 2, abs_tol=1e-6)
    assert connected.velocity == Velocity(0.5, 0, pytest.approx(-0.1))


def test_origin_places_robot_on_the_globe(connected):
    lat, lon, amsl = 45.42, -75.69, 60.0
    connected.handle(from_src("gps_raw_int", 0, mavlink.GPS_FIX_TYPE_3D_FIX, round(lat * 1e7), round(lon * 1e7),
                              round(amsl * 1000), 80, 120, 0, 0, 12, round((amsl - 34.0) * 1000), 1500, 3000))
    connected.handle(from_src("gps_global_origin", round(lat * 1e7), round(lon * 1e7), round(amsl * 1000)))
    connected.handle(from_src("local_position_ned", 0, 100.0, 0.0, -50.0, 0, 0, 0))

    assert connected.gps == GpsFix("3d", satellites=12, hdop=0.8)
    assert connected.relative_to == Link.to(connected._origin)
    assert _wait_for(lambda: connected.pose is not None)
    got_lat, got_lon, height = connected.pose.offset.to_geodetic()
    assert got_lat == pytest.approx(lat + 100 / 111_100, abs=1e-5)  # 100 m north
    assert got_lon == pytest.approx(lon, abs=1e-6)
    assert height == pytest.approx(amsl - 34.0 + 50.0, abs=0.01)  # ellipsoid height, 50 m up
    assert connected.mount.accuracy.horizontal == pytest.approx(1.5 * 2.146, rel=1e-3)


def test_home_is_a_linked_position(connected):
    connected.handle(from_src("gps_global_origin", 450000000, -750000000, 0))
    connected.handle(from_src("home_position", 450000000, -750000000, 60000, 5.0, 0.0, 0.0,
                              [1, 0, 0, 0], 0, 0, 0))
    assert connected.home == Link.to(connected._home)
    assert connected._home.mount.offset == Offset(5.0, 0.0, 0.0)
    assert _wait_for(lambda: connected._home.pose is not None)


def test_batteries_replace_by_id(connected):
    cells = [4200, 4100, 4000] + [65535] * 7
    connected.handle(from_src("battery_status", 1, 0, 0, 2500, cells, 1250, -1, -1, 80))
    connected.handle(from_src("battery_status", 0, 0, 0, 2500, cells, -1, -1, -1, -1))
    connected.handle(from_src("battery_status", 1, 0, 0, 2500, cells, 1000, -1, -1, 70))
    assert [b.id for b in connected.batteries] == [0, 1]
    assert connected.batteries[1] == Battery(1, voltage=12.3, current=10.0, remaining=0.7)
    assert connected.batteries[0].current is None


def test_mirror_sees_the_generic_robot(connected, synq_session):
    connected.handle(heartbeat(custom_mode=4))
    mirror = Robot.from_topic(connected.synq_absolute_path)
    assert mirror.mode == "GUIDED"
    assert mirror.home == connected.home


# ── Commands ─────────────────────────────────────────────────────────────────

def test_arm_waits_for_ack(connected):
    responder = ack_when_sent(connected, mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    connected.arm()
    responder.join()
    [cmd] = [m for m in connected._conn.of_type("COMMAND_LONG")
             if m.command == mavlink.MAV_CMD_COMPONENT_ARM_DISARM]
    assert (cmd.target_system, cmd.param1, cmd.param2) == (1, 1, 0)


def test_refusal_raises(connected):
    responder = ack_when_sent(connected, mavlink.MAV_CMD_COMPONENT_ARM_DISARM, mavlink.MAV_RESULT_DENIED)
    with pytest.raises(CommandRefused, match="denied"):
        connected.arm()
    responder.join()


def test_ack_for_another_gcs_is_ignored(connected, monkeypatch):
    monkeypatch.setattr("spiri_mav.mav_robot.COMMAND_TIMEOUT", 0.05)
    connected.handle(from_src("command_ack", mavlink.MAV_CMD_NAV_LAND, mavlink.MAV_RESULT_ACCEPTED, 0, 0, 255, 190))
    with pytest.raises(CommandRefused, match="no COMMAND_ACK"):
        connected.land()
    assert len([m for m in connected._conn.of_type("COMMAND_LONG")
                if m.command == mavlink.MAV_CMD_NAV_LAND]) == 3  # retried


def test_set_mode_ardupilot(connected):
    responder = ack_when_sent(connected, mavlink.MAV_CMD_DO_SET_MODE)
    connected.set_mode("RTL")
    responder.join()
    [cmd] = [m for m in connected._conn.of_type("COMMAND_LONG") if m.command == mavlink.MAV_CMD_DO_SET_MODE]
    assert cmd.param2 == 6


def test_set_mode_rejects_unknown_mode(connected):
    with pytest.raises(CommandRefused, match="Unknown mode"):
        connected.set_mode("WARP")


def test_goto_converts_ecef_to_amsl(connected):
    connected._geoid_separation = -34.0
    target = Offset.from_geodetic(45.0, -75.0, 66.0)
    responder = ack_when_sent(connected, mavlink.MAV_CMD_DO_REPOSITION)
    connected.goto(target)
    responder.join()
    [cmd] = connected._conn.of_type("COMMAND_INT")
    assert (cmd.x, cmd.y) == (450000000, -750000000)
    assert cmd.z == pytest.approx(100.0, abs=1e-3)
    assert cmd.frame == mavlink.MAV_FRAME_GLOBAL


def test_set_roi_shows_it_in_the_tree(connected):
    connected._geoid_separation = -34.0
    target = Offset.from_geodetic(45.0, -75.0, 66.0)
    responder = ack_when_sent(connected, mavlink.MAV_CMD_DO_SET_ROI_LOCATION)
    connected.set_roi(target)
    responder.join()
    [cmd] = connected._conn.of_type("COMMAND_INT")
    assert (cmd.param1, cmd.x, cmd.y) == (0, 450000000, -750000000)  # all gimbals
    assert cmd.z == pytest.approx(100.0, abs=1e-3)
    assert connected.roi == Link.to(connected._roi)
    assert _wait_for(lambda: connected._roi.pose is not None)
    assert connected._roi.pose.offset.to_geodetic() == pytest.approx((45.0, -75.0, 66.0), abs=1e-6)

    responder = ack_when_sent(connected, mavlink.MAV_CMD_DO_SET_ROI_NONE)
    connected.clear_roi()
    responder.join()
    assert connected.roi is None


def test_refused_roi_is_not_shown(connected):
    responder = ack_when_sent(connected, mavlink.MAV_CMD_DO_SET_ROI_LOCATION, mavlink.MAV_RESULT_DENIED)
    with pytest.raises(CommandRefused, match="denied"):
        connected.set_roi(Offset.from_geodetic(45.0, -75.0, 66.0))
    responder.join()
    assert connected.roi is None


def test_reconnect_forgets_the_roi(connected):
    connected._show_roi(Offset.from_geodetic(45.0, -75.0, 66.0))
    connected.tick(now=time.monotonic() + 10)
    connected.handle(heartbeat())
    assert connected.roi is None


def test_commands_need_a_connection(robot):
    with pytest.raises(CommandRefused, match="not connected"):
        robot.arm()


def test_gcs_sysid_conflict_stops_heartbeat_and_commands(connected):
    connected.handle(from_src("param_value", b"SYSID_MYGCS", float(connected.sysid),
                              mavlink.MAV_PARAM_TYPE_REAL32, 1, 0))
    connected._conn.sent.clear()
    connected.tick(now=time.monotonic() + 2)
    assert connected._conn.of_type("HEARTBEAT") == []
    with pytest.raises(CommandRefused, match="GCS sysid"):
        connected.arm()


# ── Gimbal ───────────────────────────────────────────────────────────────────

GIMBAL = (1, mavlink.MAV_COMP_ID_GIMBAL)


def gimbal_attitude(q: Orientation, flags=mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME, device_id=0):
    return from_src("gimbal_device_attitude_status", 0, 0, 0, flags, [q.w, q.x, q.y, q.z],
                    0, 0, 0, 0, 0, 0, device_id, src=GIMBAL)


def robot_attitude(robot, roll=0.0, pitch=0.0, yaw=0.0):
    q = Orientation.from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
    robot.handle(from_src("attitude_quaternion", 0, q.w, q.x, q.y, q.z, 0, 0, 0))


def deg_euler(q: Orientation):
    return q.to_euler(degrees=True)


def test_gimbal_is_created_next_to_the_robot(connected):
    robot_attitude(connected)
    connected.handle(gimbal_attitude(Orientation()))
    [gimbal] = connected._gimbals.values()
    assert gimbal.synq_topic == "test/robot_gimbal154"
    assert gimbal.relative_to == Link.to(connected)
    assert connected.gimbals == (Link.to(gimbal),)


def test_gimbal_yaw_in_vehicle_frame(connected):
    robot_attitude(connected, yaw=90)
    q = Orientation.from_euler(0, math.radians(-30), math.radians(20))
    connected.handle(gimbal_attitude(q))
    [gimbal] = connected._gimbals.values()
    assert gimbal.pitch == pytest.approx(-30)
    assert gimbal.yaw == pytest.approx(20)
    _, pitch, yaw = deg_euler(gimbal.mount.orientation)
    assert (pitch, yaw) == (pytest.approx(-30), pytest.approx(20))


def test_gimbal_yaw_in_earth_frame(connected):
    robot_attitude(connected, yaw=90)
    q = Orientation.from_euler(0, 0, math.radians(100))  # facing 100° from north
    connected.handle(gimbal_attitude(q, flags=mavlink.GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME
                                     | mavlink.GIMBAL_DEVICE_FLAGS_YAW_LOCK))
    [gimbal] = connected._gimbals.values()
    assert gimbal.yaw == pytest.approx(10)
    assert gimbal.yaw_lock
    assert deg_euler(gimbal.mount.orientation)[2] == pytest.approx(10)


def test_level_gimbal_on_pitched_robot_points_down_in_body_frame(connected):
    robot_attitude(connected, pitch=10)  # nose up
    connected.handle(gimbal_attitude(Orientation()))  # level with the horizon
    [gimbal] = connected._gimbals.values()
    assert gimbal.pitch == pytest.approx(0)
    assert deg_euler(gimbal.mount.orientation)[1] == pytest.approx(-10)


def test_gimbal_waits_for_robot_attitude(connected):
    connected.handle(gimbal_attitude(Orientation()))
    [gimbal] = connected._gimbals.values()
    assert gimbal.mount is None


def test_gimbal_pose_is_in_the_tree(connected):
    connected.handle(from_src("gps_global_origin", 450000000, -750000000, 0))
    connected.handle(from_src("local_position_ned", 0, 0, 0, -10, 0, 0, 0))
    robot_attitude(connected)
    connected.handle(gimbal_attitude(Orientation()))
    [gimbal] = connected._gimbals.values()
    assert _wait_for(lambda: gimbal.pose is not None)


def test_gimbal_point_takes_control_then_points(connected):
    robot_attitude(connected)
    connected.handle(gimbal_attitude(Orientation()))
    connected.handle(from_src("gimbal_manager_information", 0, 0, 154, 0, 0,
                              math.radians(-90), math.radians(30), math.radians(-180), math.radians(180),
                              src=FCU))
    [gimbal] = connected._gimbals.values()
    assert gimbal.pitch_range == (pytest.approx(-90), pytest.approx(30))

    configure = ack_when_sent(connected, mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE)
    pitchyaw = ack_when_sent(connected, mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)
    gimbal.point(-45, 10)
    configure.join(), pitchyaw.join()
    [cfg] = [m for m in connected._conn.of_type("COMMAND_LONG")
             if m.command == mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE]
    [cmd] = [m for m in connected._conn.of_type("COMMAND_LONG")
             if m.command == mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW]
    assert (cfg.param1, cfg.param2, cfg.param7) == (connected.sysid, connected.compid, 154)
    assert (cmd.param1, cmd.param2, cmd.param7) == (-45, 10, 154)

    # Once the manager reports us in control, pointing doesn't reconfigure.
    connected.handle(from_src("gimbal_manager_status", 0, 0, 154, connected.sysid, connected.compid, 0, 0))
    connected._conn.sent.clear()
    pitchyaw = ack_when_sent(connected, mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)
    gimbal.point(0, 0)
    pitchyaw.join()
    assert [m.command for m in connected._conn.of_type("COMMAND_LONG")] == [
        mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW]


def test_gimbal_point_out_of_range_is_refused(connected):
    robot_attitude(connected)
    connected.handle(gimbal_attitude(Orientation()))
    [gimbal] = connected._gimbals.values()
    gimbal.pitch_range = (-90.0, 30.0)
    with pytest.raises(CommandRefused, match="outside"):
        gimbal.point(45, 0)


def test_gimbal_mirror_from_topic(connected):
    robot_attitude(connected)
    connected.handle(gimbal_attitude(Orientation.from_euler(0, math.radians(-30), 0)))
    [link] = connected.gimbals
    mirror = Gimbal.from_topic(link.topic)
    assert mirror.pitch == pytest.approx(-30)
    assert mirror.relative_to == Link.to(connected)


@pytest.mark.xfail(strict=True, reason=(
    "SpiriSynq bug: tuples go over the wire as YAML lists and aren't converted back, so "
    "mirrors get a list on rehydrate and drop later puts as a type mismatch. Affects every "
    "tuple field (cameras, gimbals, batteries, failsafe, available_modes, ranges)."
))
def test_gimbal_cameras_reach_mirrors(connected):
    robot_attitude(connected)
    connected.handle(gimbal_attitude(Orientation()))
    [gimbal] = connected._gimbals.values()
    gimbal.cameras = (Link("azrael/camera_eo"), Link("azrael/camera_ir"))
    mirror = Gimbal.from_topic(gimbal.synq_absolute_path)
    assert mirror.cameras == gimbal.cameras


def test_configured_cameras_attach_to_their_gimbal():
    cameras = (Link("mu-1/camera_eo"), Link("mu-1/camera_ir"))
    robot = MavRobot("test/robot_cams", synq_authoritive=True, _gimbal_cameras={154: cameras})
    robot._conn = FakeLink(robot.sysid, robot.compid)
    try:
        robot.handle(heartbeat())
        robot_attitude(robot)
        robot.handle(gimbal_attitude(Orientation()))                   # device 154: configured
        robot.handle(gimbal_attitude(Orientation(), device_id=2))      # not configured
        assert robot._gimbals[154].cameras == cameras
        assert robot._gimbals[2].cameras == ()
    finally:
        robot.close()
