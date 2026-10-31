# Changes

## Unreleased

### Features

- **The ROI from any GCS**: `Robot.roi` and `<robot>_roi` now follow ROI commands that QGC
  or another GCS sends the autopilot, once it accepts them, as well as SpiriMav's own and
  `Gimbal.point_at`. This needs mavlink-router's `SnifferSysid` set to SpiriMav's sysid, so
  other GCSs' commands reach it. Pointing by angle ends the ROI; an ROI that can't be placed
  (above terrain, the next waypoint, another system) hides it. ROIs from a mission still
  don't show.
- **The ROI and home are points**: their poses have no orientation (SpiriSynq 0.6.0), so they
  no longer show made-up angles.
- **Camera lenses from MAVLink**: the field of view from `CAMERA_FOV_STATUS` (streamed at
  1 Hz), or the focal length, sensor size and resolution from `CAMERA_INFORMATION`, go to
  the SpiriCamera cameras on the camera's gimbal through their `set_intrinsics` RPC, as
  the `mavlink` origin: when they change, and every 10 s for a camera that restarted.
  `CAMERA_INFORMATION.gimbal_device_id` says which gimbal a camera is on; one on no
  gimbal, or on one with no cameras configured, is left alone.

### Fixes

- **SpiriSynq from PyPI**, `>=0.6.0`, instead of a `../SpiriSynq` checkout, which broke CI
  and the Docker image build.

### Internal

- Code formatted and linted with ruff, as CI checks.

## 0.1.0 (2026-10-23)

### Features

- Project scaffold from SpiriProjectTemplates, depending on SpiriSynq and pymavlink.
- **`Robot`**, the generic vehicle: a SpiriSynq `Position` with state fields (`connected`,
  `armed`, `mode`, `failsafe`, `velocity`, `gps`, `batteries`, a `home` link) and RPCs
  (`arm`, `disarm`, `set_mode`, `takeoff`, `goto`, `land`, `return_home`) that return on the
  vehicle's acceptance and raise `CommandRefused` otherwise.
- **`MavRobot`**, its MAVLink backend. It hangs under the EKF origin in the frame tree,
  derives `failsafe` from `HEARTBEAT.system_status` plus failsafe `STATUSTEXT`s, heartbeats
  as an onboard controller, and stops heartbeating and commanding if its sysid turns out to
  be the FCU's GCS sysid.

- **`Gimbal`**, a `Position` linked to its robot, with display `pitch`/`yaw`, limits, and
  RPCs `point`, `point_at`, `neutral`, `release`. `MavGimbal` speaks Gimbal Protocol v2:
  `MavRobot` creates one per gimbal device it hears from, at `<robot>_gimbal<id>`, lists
  them in `Robot.gimbals`, and turns the reported attitude into the robot's body frame.
  `Gimbal.cameras` links to the SpiriCamera cameras the gimbal carries.
- **Region of interest**: `Robot.set_roi(target)` points every gimbal at an ECEF target
  (`DO_SET_ROI_LOCATION`), `clear_roi()` lets go, and `Robot.roi` links to a
  `<robot>_roi` Position in the frame tree while one is set. MAVLink doesn't report the
  active ROI, so it shows the ROI SpiriMav set, not one set by QGC or a mission, and is
  cleared when the autopilot reconnects.
- **`SpiriMav run`** bridges an autopilot to SpiriSynq. `--url`, `--sysid`, `--target-sysid`,
  `--topic` and `--zenoh-connect` override the `SPIRI_MAV_*` settings; the default URL is
  ArduPilot SITL's `tcp:127.0.0.1:5760`.
- **`SPIRI_MAV_GIMBAL_CAMERAS`** says which SpiriCamera cameras each gimbal carries, as JSON
  by gimbal device id (`{"1": ["mu-1/camera_eo"]}`), since MAVLink doesn't. They become the
  gimbal's `cameras` links.

### Fixes

- **A lost autopilot link no longer pins a CPU core, and reconnects.** When the FCU's TCP
  peer closed (SITL or its container restarting), pymavlink polled the dead socket in a
  tight loop, about 100% of a core and hundreds of thousands of "EOF on TCP socket" lines a
  second, and never reconnected. `MavRobot.run` now treats EOF as a lost link: `connected`
  drops at once, and it reconnects after 1 s, backing off to 10 s while the link stays down.

### Tests

- `MavRobot` against recorded-style MAVLink messages and a fake link: state, failsafe,
  frame tree, batteries, command ACKs, ROI, and the sysid guard.
- The link against a local TCP stand-in for SITL: a closed link drops `connected` and
  reconnects, and stays under a third of a core while down.
- Loguru goes back to the current stderr after each test, so threads logging after a CLI
  test don't write to its closed capture stream.

### Internal

- Failsafe and `Robot` object design in `docs/design.md`.
