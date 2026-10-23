"""The generic robot: what any SpiriSynq client can rely on, whatever the autopilot.

A :class:`Robot` is a :class:`~SpiriSynq.example_types.position.Position`, so
it sits in the frame tree like anything else with a place in the world:
cameras and gimbals mount on it with ``relative_to=Link.to(robot)``.

State flows out as fields; intent flows in as ``@remote_method`` RPCs. See
``docs/design.md`` -- in particular, the robot takes no part in failsafe
decisions: ``failsafe`` only mirrors what the autopilot reports.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from SpiriSynq.example_types.position import Offset, Position
from SpiriSynq.link import Link
from SpiriSynq.remote_callables import remote_method

Failsafe = Literal["gcs", "rc", "battery", "geofence", "ekf", "other"]

GpsFixType = Literal["none", "2d", "3d", "dgps", "rtk_float", "rtk_fixed"]

BatteryState = Literal["unknown", "ok", "low", "critical", "emergency", "failed", "unhealthy"]


@dataclass(frozen=True)
class Velocity:
    """Velocity in the robot's parent frame (NED for an autopilot), m/s."""

    north: float
    east: float
    down: float


@dataclass(frozen=True)
class GpsFix:
    """GPS receiver status. Where the robot is comes from the frame tree, not from here."""

    fix: GpsFixType
    satellites: int | None = None
    hdop: float | None = None


@dataclass(frozen=True)
class Battery:
    id: int
    voltage: float | None = None    # V
    current: float | None = None    # A, positive when discharging
    remaining: float | None = None  # 0..1
    state: BatteryState = "unknown"


class CommandRefused(RuntimeError):
    """The vehicle rejected, or never acknowledged, a command."""


@dataclass
class Robot(Position):
    """A vehicle as SpiriSynq sees it.

    The ``_do_*`` hooks are the backend's job; the RPC surface lives here so a
    mirror typed as plain ``Robot`` sees the same schema as one typed as a
    backend subclass. Every RPC returns once the vehicle accepts the command
    and raises :class:`CommandRefused` otherwise. Accepted isn't done: watch
    the fields (``armed``, ``mode``, ``pose``) for the outcome.
    """

    connected: bool = False                  # autopilot link alive
    armed: bool = False
    mode: str = ""                           # autopilot-native name, e.g. "GUIDED"
    available_modes: tuple[str, ...] = ()    # what set_mode accepts on this vehicle
    failsafe: tuple[Failsafe, ...] = ()      # sorted; empty when none is active
    failsafe_text: str | None = None         # the message behind failsafe, if any
    home: Link | None = None                 # a Position where return_home goes
    roi: Link | None = None                  # a Position set_roi pointed us at; None when cleared
    gimbals: tuple[Link, ...] = ()           # Gimbals mounted relative_to this robot
    velocity: Velocity | None = None
    gps: GpsFix | None = None
    batteries: tuple[Battery, ...] = ()

    @remote_method()
    def arm(self, force: bool = False) -> None:
        """Arm the motors; *force* skips the autopilot's pre-arm checks."""
        self._do_arm(True, force)

    @remote_method()
    def disarm(self, force: bool = False) -> None:
        """Disarm the motors; *force* disarms even in flight."""
        self._do_arm(False, force)

    @remote_method()
    def set_mode(self, mode: str) -> None:
        """Switch to *mode*, one of ``available_modes``."""
        if self.available_modes and mode not in self.available_modes:
            raise CommandRefused(f"Unknown mode {mode!r}; available: {', '.join(self.available_modes)}")
        self._do_set_mode(mode)

    @remote_method()
    def takeoff(self, altitude: float) -> None:
        """Take off to *altitude* metres above home."""
        self._do_takeoff(altitude)

    @remote_method()
    def goto(self, target: Offset) -> None:
        """Fly to *target*, an ECEF offset (see ``Offset.from_geodetic``)."""
        self._do_goto(target)

    @remote_method()
    def land(self) -> None:
        """Land where the robot is."""
        self._do_land()

    @remote_method()
    def return_home(self) -> None:
        """Return to home and land, as the autopilot's RTL defines it."""
        self._do_return_home()

    @remote_method()
    def set_roi(self, target: Offset) -> None:
        """Point the robot's gimbals at *target*, an ECEF offset, until :meth:`clear_roi`."""
        self._do_set_roi(target)

    @remote_method()
    def clear_roi(self) -> None:
        """Stop pointing at the region of interest."""
        self._do_set_roi(None)

    def _do_arm(self, arm: bool, force: bool) -> None:
        raise NotImplementedError

    def _do_set_mode(self, mode: str) -> None:
        raise NotImplementedError

    def _do_takeoff(self, altitude: float) -> None:
        raise NotImplementedError

    def _do_goto(self, target: Offset) -> None:
        raise NotImplementedError

    def _do_land(self) -> None:
        raise NotImplementedError

    def _do_return_home(self) -> None:
        raise NotImplementedError

    def _do_set_roi(self, target: Offset | None) -> None:
        raise NotImplementedError
