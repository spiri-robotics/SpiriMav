# Changes

## Unreleased

### Features

- **HUD widgets**: SpiriMav publishes the robot's overlays for its cameras, as SpiriCamera
  `HudWidget`s a camera shows by naming them in its `overlay_widgets`. The generic ones,
  any camera's to show, are under `hud/widgets/`: a virtual horizon (`horizon`, an
  attitude indicator of the airframe's roll and pitch), a compass tape (`heading`), status
  (`status`: armed, mode, failsafe, GPS, or the autopilot link down), height above home,
  climb and ground speed (`flight`), the batteries (`battery`), and where each gimbal
  points (`gimbal<id>`). A gimbal's rangefinder's stays with the gimbal, as it's drawn on
  its cameras' axis (`<robot>_gimbal<id>_range_hud`: a reticle, the reading, and the
  spot's coordinates), only once its sensor reports. They read `<robot>_instruments`, an `Instruments` object of the robot's state as plain,
  rounded numbers and strings, updated at 10 Hz: MiniJinja templates can't do
  trigonometry, and a camera mirrors them untyped. With the autopilot's link down it
  says nothing but that, rather than show stale values.
- **Markers on home and the ROI**: `<robot>_home_hud` and `<robot>_roi_hud` are drawn
  where home and the region of interest are in the picture, with their distance from the
  camera, using SpiriCamera's `{# anchor: #}`, so the camera needs a pose (it is mounted
  on its gimbal) and a lens. They aren't drawn while their place is out of shot or unset.

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
- **Gimbal rangefinders**: where a gimbal's rangefinder hits is a `BearingDetection`, a
  Position at `<robot>_gimbal<id>_range` under the gimbal (`Gimbal.rangefinder` links to
  it): a point at the reading's distance along the gimbal's x axis, so its pose has
  coordinates like any frame. It also carries the reading, `status` (`ok`, `near`,
  `far`), `distance`, the sensor's `min` and `max`, and the beam's accuracies. With no
  return it has no mount or pose, and after 2 s without a reading its status goes to
  `None`. Readings come from `DISTANCE_SENSOR` (streamed at 10 Hz), since ArduPilot
  reports a rangefinder only as fixed to the airframe; which sensor id is which gimbal's
  is config, `SPIRI_MAV_GIMBAL_RANGEFINDERS`, e.g. `{"1": 0}`. Config only says whose a
  sensor would be: the spot is created when that sensor first reports, so a robot without
  one has none.
- **Configured accuracies**: neither autopilot reports how accurately a gimbal knows where
  it points or how accurate a rangefinder is, so they're config, as 90% bounds:
  `SPIRI_MAV_GIMBAL_POINTING_ACCURACY` (± degrees off the axis, default 1.0, mostly the
  vehicle's compass heading), published as `Gimbal.pointing_accuracy` and the spot's
  `pointing_accuracy`; and `SPIRI_MAV_RANGEFINDER_ACCURACY` plus
  `SPIRI_MAV_RANGEFINDER_ACCURACY_PER_M` (± metres along the beam, default 1.0 m + 0.25% of
  the distance, a ZT30-class laser's spec), as the spot's `range_accuracy`. The defaults
  are estimates; set a datasheet's figures. The spot's mount carries the beam's error as
  an `Accuracy`, horizontal and vertical by the beam's elevation, so its pose accuracy is
  the whole tree's.

### Fixes

- **SpiriSynq from PyPI**, `>=0.6.0`, instead of a `../SpiriSynq` checkout, which broke CI
  and the Docker image build.

### Tests

- Every HUD widget is rendered as a camera renders it (MiniJinja, strict about undefined
  values) against flying, unknown and failing states; `minijinja` is a dev dependency.
- The serve test keeps its robot alive, as `SyncableObject.__del__` closes it again.

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
