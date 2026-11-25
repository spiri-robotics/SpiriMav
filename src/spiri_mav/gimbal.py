"""The generic gimbal: a pointable mount on a robot.

A :class:`Gimbal` is a :class:`~SpiriSynq.example_types.position.Position`
whose ``relative_to`` links to its robot. ``mount.offset`` is where it sits in
the robot's body frame (FRD) and ``mount.orientation`` is where it's pointing,
also in the body frame, so a camera mounted with ``relative_to=Link.to(gimbal)``
looks wherever the gimbal does.

The gimbal owns its cameras: ``cameras`` links to the SpiriCamera objects it
carries. The motion lives here; a camera is the sensor it points.

A rangefinder on the gimbal, along the cameras' axis, is the gimbal's too.
What it hits is a :class:`RangefinderDetection`, SpiriDetect's
``BearingDetection`` along the gimbal's axis, linked from ``rangefinder``.

Neither autopilot reports how accurately a gimbal knows where it points or
how accurate a rangefinder is, so ``pointing_accuracy`` and the detection's
beam accuracies are configured: a datasheet's figures, or the defaults, which
are estimates (see ``spiri_mav.main.Settings``).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Literal

from SpiriDetect import BearingDetection
from SpiriSynq.example_types.position import Offset, Position
from SpiriSynq.link import Link
from SpiriSynq.remote_callables import remote_method

from spiri_mav.robot import CommandRefused

# How often a rangefinder's detection's ``time`` is updated: a put of its
# own, and staleness needs no finer.
TIME_PERIOD = 1.0  # s

# "near": something closer than ``min``; "far": nothing within ``max``.
RangeStatus = Literal["ok", "near", "far"]


@dataclass
class RangefinderDetection(BearingDetection):
    """What a gimbal's rangefinder hits: a bearing along the gimbal's axis.

    ``relative_to`` links to the gimbal, and the bearing is straight down its
    x axis, the way it points. With a return, ``range`` is the reading and
    ``pose`` the spot, its accuracy the whole tree's: the beam's
    (``range_accuracy`` along it, ``bearing_accuracy`` across it, the gimbal's
    pointing accuracy), the gimbal's and the robot's above it. Something too
    near or nothing within reach is a bearing without a range: the ``ray``,
    no ``pose``. A silent rangefinder has neither.

    ``status`` says which, and ``min``/``max`` are what the sensor can measure.
    ``time`` is the latest reading's to within ``TIME_PERIOD``.
    """

    status: RangeStatus | None = None  # None while the rangefinder is silent
    min: float | None = None  # m: what the rangefinder can measure
    max: float | None = None  # m

    def set_reading(
        self,
        status: RangeStatus | None,
        distance: float | None = None,
        min: float | None = None,
        max: float | None = None,
    ) -> None:
        """Take a reading: *distance* m along the gimbal's axis when *status* is "ok"."""
        self.status, self.min, self.max = status, min, max
        if status is None:
            self.clear()
            return
        now = time.time()
        if self.time is None or now - self.time >= TIME_PERIOD:
            self.time = now
        self.set_bearing(0.0, 0.0, distance if status == "ok" else None)


@dataclass
class Gimbal(Position):
    """A gimbal as SpiriSynq sees it.

    ``pitch`` and ``yaw`` are for display and UI: the frame tree holds the real
    orientation. RPCs follow :class:`~spiri_mav.robot.Robot`'s rules: they
    return once the vehicle accepts and raise :class:`CommandRefused` otherwise.
    """

    pitch: float | None = None  # deg from the horizon, + up
    yaw: float | None = None  # deg from the robot's heading, + right
    yaw_lock: bool = False  # holds an earth heading instead of following the robot
    pitch_range: tuple[float, float] | None = None  # deg
    yaw_range: tuple[float, float] | None = None  # deg, robot-relative
    cameras: tuple[Link, ...] = ()  # SpiriCamera cameras this gimbal carries
    rangefinder: Link | None = (
        None  # its RangefinderDetection; None without a rangefinder
    )
    pointing_accuracy: float | None = None  # ± deg, 90%, off its x axis; configured

    @remote_method()
    def point(self, pitch: float, yaw: float, yaw_lock: bool = False) -> None:
        """Point at *pitch* from the horizon and *yaw* from the robot's heading, in degrees.

        With *yaw_lock*, *yaw* is from north instead, and held as the robot turns.
        """
        _check_range("pitch", pitch, self.pitch_range)
        if not yaw_lock:
            _check_range("yaw", yaw, self.yaw_range)
        self._do_point(pitch, yaw, yaw_lock)

    @remote_method()
    def point_at(self, target: Offset) -> None:
        """Track *target*, an ECEF offset, until :meth:`release`."""
        self._do_point_at(target)

    @remote_method()
    def neutral(self) -> None:
        """Return to the gimbal's neutral (usually forward, level) position."""
        self._do_neutral()

    @remote_method()
    def release(self) -> None:
        """Stop tracking and hand control back."""
        self._do_release()

    def _do_point(self, pitch: float, yaw: float, yaw_lock: bool) -> None:
        raise NotImplementedError

    def _do_point_at(self, target: Offset) -> None:
        raise NotImplementedError

    def _do_neutral(self) -> None:
        raise NotImplementedError

    def _do_release(self) -> None:
        raise NotImplementedError


def _check_range(name: str, value: float, limits: tuple[float, float] | None) -> None:
    if limits is not None and not limits[0] <= value <= limits[1]:
        raise CommandRefused(f"{name} {value}° is outside {limits[0]}°..{limits[1]}°")
