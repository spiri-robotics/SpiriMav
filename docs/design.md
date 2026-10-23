# Design

SpiriMav runs on the companion (offboard) computer. It talks MAVLink to the
flight controller (FCU) and exposes the vehicle as a SpiriSynq `Robot` object.

```
QGC ──(any MAVLink path we don't own)──▶ FCU ◀──serial MAVLink── companion [SpiriMav, Robot authoritative] ──Zenoh──▶ SpiriSynq clients
```

## Failsafe: the FCU watches QGC, not us

The operator GCS is QGC, and we do not own or see its connection to the
vehicle (radio, mavlink-router, cellular, ...). So the companion must not sit
in the GCS-failsafe path:

- **QGC keeps the FCU's GCS sysid** (`SYSID_MYGCS` / `MAV_GCS_SYSID` on
  ArduPilot, stock 255). Losing QGC trips the FCU's native GCS failsafe no
  matter which path QGC uses.
- **The companion is a secondary GCS** with a different sysid. It sends
  heartbeats and commands, but the FCU never counts its heartbeat toward the
  GCS failsafe, so a live companion cannot mask a lost QGC.
- Commands from a non-MYGCS sysid are accepted by default on both ArduPilot
  and PX4. Don't enable ArduPilot's `MAV_OPTIONS` "only accept from GCS sysid"
  bit.
- RC, battery and geofence failsafes stay on the FCU, untouched.

### Gaps and how they're covered

- **Companion dies while driving the vehicle** (GUIDED / OFFBOARD): covered by
  the autopilot's own setpoint timeouts — ArduPilot `GUID_TIMEOUT`, PX4
  `COM_OF_LOSS_T` / `COM_OBL_RC_ACT`. A Lua script watching the companion
  heartbeat is the escape hatch if that's not enough. Companion death while
  QGC is in control needs no failsafe.
- **No QGC at all** (fully autonomous runs): ArduPilot only arms the GCS
  failsafe after it has seen a MYGCS heartbeat. For that deployment, set
  MYGCS to the companion's sysid. This is a per-deployment FCU parameter, not
  a code path.
- **Behaviour in AUTO** is governed by `FS_GCS_ENABLE` / `FS_OPTIONS` (RTL vs
  continue mission). That is the real policy knob and it lives on the FCU.

All of the above must be verified in SITL rather than assumed.

## The Robot object

`Robot` is an observer and a command surface; it takes no part in failsafe
decisions. Same split as `GPS`/`MavGPS`: a generic `Robot` base and a
MAVLink-specific `MavRobot`.

- **State flows out as fields**: `armed`, `mode`, `system_status`,
  `failsafe`, GPS, batteries, ...
- **`failsafe` is what the FCU reports**, derived from
  `HEARTBEAT.system_status`, mode changes (e.g. a jump to RTL) and failsafe
  `STATUSTEXT`s. Neither autopilot publishes a clean standard failsafe
  bitmask, so this parsing is per-autopilot and partly heuristic.
- **`gcs_seen` is best-effort display only.** ArduPilot forwards broadcast
  messages like `HEARTBEAT` between links, so QGC's heartbeat usually reaches
  the companion; PX4 needs `MAV_x_FORWARD`. It never feeds a safety decision.
- **`Robot` is a `Position`** in SpiriSynq's frame tree, so payloads mount on it
  with `relative_to=Link.to(robot)`. `MavRobot` hangs under the EKF origin
  (`ECEF → <robot>_origin → <robot>`): `LOCAL_POSITION_NED` and
  `ATTITUDE_QUATERNION` go into its mount unconverted, `GPS_GLOBAL_ORIGIN`
  places the origin, and everything below the origin works without GPS. Body
  frames are FRD.
- **`failsafe` is published only while `system_status` is CRITICAL or
  EMERGENCY** (which is how ArduPilot reports "any failsafe"); failsafe
  `STATUSTEXT`s only say which kind, defaulting to `other`.
- **`roi` echoes what we set, not what the FCU is doing.** No MAVLink message
  reports the active region of interest, so `Robot.roi` is the last ROI
  `set_roi` got accepted, dropped by `clear_roi` or a reconnect. An ROI from
  QGC or a mission doesn't show, and ours may already have been replaced.
- **Intent flows in as `@remote_method` RPCs** (`arm`, `set_mode`, `goto`,
  ...), never as field writes from mirrors: RPCs are acknowledged and can
  refuse. They act alongside QGC with no arbitration, as two GCSs on one
  vehicle normally do; QGC is the operator's override.
