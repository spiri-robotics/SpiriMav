"""The generic gimbal: a pointable mount on a robot.

A :class:`Gimbal` is a :class:`~SpiriSynq.example_types.position.Position`
whose ``relative_to`` links to its robot. ``mount.offset`` is where it sits in
the robot's body frame (FRD) and ``mount.orientation`` is where it's pointing,
also in the body frame, so a camera mounted with ``relative_to=Link.to(gimbal)``
looks wherever the gimbal does.

The gimbal owns its cameras: ``cameras`` links to the SpiriCamera objects it
carries. The motion lives here; a camera is the sensor it points.
"""

from __future__ import annotations

from dataclasses import dataclass

from SpiriSynq.example_types.position import Offset, Position
from SpiriSynq.link import Link
from SpiriSynq.remote_callables import remote_method

from spiri_mav.robot import CommandRefused


@dataclass
class Gimbal(Position):
    """A gimbal as SpiriSynq sees it.

    ``pitch`` and ``yaw`` are for display and UI: the frame tree holds the real
    orientation. RPCs follow :class:`~spiri_mav.robot.Robot`'s rules: they
    return once the vehicle accepts and raise :class:`CommandRefused` otherwise.
    """

    pitch: float | None = None                    # deg from the horizon, + up
    yaw: float | None = None                      # deg from the robot's heading, + right
    yaw_lock: bool = False                        # holds an earth heading instead of following the robot
    pitch_range: tuple[float, float] | None = None  # deg
    yaw_range: tuple[float, float] | None = None    # deg, robot-relative
    cameras: tuple[Link, ...] = ()                # SpiriCamera cameras this gimbal carries

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
