"""The generic gimbal: a pointable mount on a robot.

A :class:`Gimbal` is a :class:`~SpiriSynq.example_types.position.Position`
whose ``relative_to`` links to its robot. ``mount.offset`` is where it sits in
the robot's body frame (FRD) and ``mount.orientation`` is where it's pointing,
also in the body frame, so a camera mounted with ``relative_to=Link.to(gimbal)``
looks wherever the gimbal does.

The gimbal owns its cameras: ``cameras`` links to the SpiriCamera objects it
carries. The motion lives here; a camera is the sensor it points.

A rangefinder on the gimbal, along the cameras' axis, is the gimbal's too.
What it hits is a :class:`BearingDetection`, a point in the frame tree under
the gimbal, linked from ``rangefinder``.

Neither autopilot reports how accurately a gimbal knows where it points or
how accurate a rangefinder is, so ``pointing_accuracy`` and the spot's beam
accuracies are configured: a datasheet's figures, or the defaults, which are
estimates (see ``spiri_mav.main.Settings``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from SpiriSynq.example_types.position import Accuracy, Offset, Pose, Position
from SpiriSynq.link import Link
from SpiriSynq.remote_callables import remote_method

from spiri_mav.robot import CommandRefused

# "near": something closer than ``min``; "far": nothing within ``max``.
RangeStatus = Literal["ok", "near", "far"]


@dataclass
class BearingDetection(Position):
    """Where a gimbal's rangefinder hits: a point in the frame tree.

    A sample of SpiriDetect's first detection kind, a bearing from a sensor
    frame with an optional range, kept here until SpiriDetect exists; see
    ``NOTES-bearing-detection.md`` for what the real one needs that this
    doesn't have.

    ``relative_to`` links to the gimbal, and ``mount`` is a point at
    ``(distance, 0, 0)`` along the gimbal's x axis, the way it points. The
    mount's accuracy is the beam's own error, from ``range_accuracy`` along it
    and ``pointing_accuracy`` across it, as 90% bounds level and plumb, so the
    spot's ``pose.accuracy`` is the whole tree's: the beam's, the gimbal's and
    the robot's above it. With no return there's no spot: ``mount`` and
    ``pose`` are None, and ``status`` says why.

    The rest is the reading, and the beam it was taken along.
    """

    status: RangeStatus | None = None  # None while the rangefinder is silent
    distance: float | None = None  # m along the beam; only when status is "ok"
    min: float | None = None  # m: what the rangefinder can measure
    max: float | None = None  # m
    range_accuracy: float | None = None  # ± m along the beam, 90%; configured
    pointing_accuracy: float | None = None  # ± deg off the beam, 90%; the gimbal's

    def set_reading(
        self,
        status: RangeStatus | None,
        distance: float | None = None,
        min: float | None = None,
        max: float | None = None,
        elevation: float | None = None,
    ) -> None:
        """Take a reading along a beam *elevation* degrees above the horizon."""
        self.status, self.distance, self.min, self.max = status, distance, min, max
        if status != "ok" or distance is None:
            self.mount = None
            return
        self.mount = Pose(
            offset=Offset(x=distance),
            orientation=None,
            accuracy=beam_accuracy(
                distance, elevation, self.range_accuracy, self.pointing_accuracy
            ),
        )


def beam_accuracy(
    distance: float,
    elevation: float | None,
    along: float | None,
    pointing: float | None,
) -> Accuracy | None:
    """The 90% bounds of a spot at *distance* along a beam, level and plumb.

    *along* is the error along the beam (m) and *pointing* the error off it
    (deg), which puts the spot ``distance * tan(pointing)`` off sideways. Each
    is split between horizontal and vertical by the beam's *elevation* (deg
    above the horizon; level if unknown) and summed in quadrature, as
    ``Accuracy`` sums independent errors. A sideways error is horizontal one
    way and partly vertical the other, so horizontal takes all of it. Unknown
    errors are left out; with neither known, so is the accuracy.
    """
    if along is None and pointing is None:
        return None
    e = math.radians(elevation or 0.0)
    a = along or 0.0
    side = distance * math.tan(math.radians(pointing or 0.0))
    return Accuracy(
        horizontal=math.hypot(a * math.cos(e), side),
        vertical=math.hypot(a * math.sin(e), side * math.cos(e)),
    )


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
    rangefinder: Link | None = None  # its BearingDetection; None without a rangefinder
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
