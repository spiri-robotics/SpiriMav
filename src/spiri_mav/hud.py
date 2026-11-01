"""The robot's HUD: overlay widgets for its cameras, and what they read.

A camera draws whatever ``HudWidget`` objects its ``overlay_widgets`` names
(SpiriCamera's ``overlay.py``); SpiriMav publishes the robot's, and a camera,
or its viewer, chooses which to show. Each widget's SVG is a MiniJinja
template that reads one object, :class:`Instruments`: the robot's state as a
HUD wants it, flat numbers and strings. MiniJinja has no trigonometry, so the
attitude is turned into degrees here, and a camera mirrors a widget's objects
untyped, so it gets plain values rather than SpiriMav's types.

The generic widgets, ones any camera can show, are at ``hud/widgets/<name>``
under the base topic, beside the cameras' HUD variants (``hud/<camera>``):

- ``horizon``, a virtual horizon: the airframe's roll and pitch as an
  attitude indicator, not lined up with the picture, which a gimbal holds
  level. Bottom middle.
- ``heading``, a compass tape. Top middle.
- ``status``: armed, mode, failsafe, GPS, or the autopilot link down. Top left.
- ``flight``: height above home, climb rate and ground speed. Bottom right.
- ``battery``, each battery's charge and voltage. Top right.
- and ``gimbal<id>`` for each gimbal: where it points. Bottom left.

A gimbal's rangefinder's isn't generic: its reticle is on the gimbal's
cameras' axis, so it stays with the gimbal, at ``<robot>_gimbal<id>_range_hud``,
once its sensor has reported: where the beam points, what it reads, and where
the spot it hit is. Centre.

Places get markers, drawn where they are in the picture rather than on the
frame (SpiriCamera places them, so a camera needs a pose and a lens to draw
them): ``<robot>_home_hud`` on home and ``<robot>_roi_hud`` on the region of
interest, each with its distance from the camera.

Text is in Miracode, the font SpiriCamera bundles, the only one its renderer
(thorvg) has; it's monospaced, so a box is sized, and text centred, from the
number of characters. thorvg ignores ``text-anchor``, so nothing uses it.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass
from string import Template

from SpiriSynq.example_types.position import Pose, Position
from SpiriSynq.syncable_objects import SyncableObject

from spiri_mav.gimbal import BearingDetection, Gimbal
from spiri_mav.robot import Robot

FONT = "Miracode"
WIDGETS_TOPIC = "hud/widgets"  # the generic widgets', under the base topic
METRES_PER_DEGREE = 111_320  # of latitude, near enough for picking decimals


@dataclass
class HudWidget(SyncableObject):
    """SpiriCamera's ``HudWidget``, the same type on the wire.

    Its fields and name, so cameras mirror it and UIs find it as one, without
    SpiriCamera and its video stack as a dependency. Change the two together.
    """

    svg_template: str = ""
    anchor: str = "top_left"
    custom_anchor_x: float = 0.0
    custom_anchor_y: float = 0.0
    z_index: float = 0.0


@dataclass
class Instruments(SyncableObject):
    """The robot's state as its HUD reads it, rounded for display.

    None is unknown: no attitude, position, home or GPS yet.
    """

    connected: bool = False
    armed: bool = False
    mode: str = ""
    failsafe: str | None = None  # what the autopilot said, or the kinds
    roll: float | None = None  # deg, + right wing down
    pitch: float | None = None  # deg, + nose up
    heading: float | None = None  # deg from true north, 0..360
    altitude: float | None = None  # m above home
    climb: float | None = None  # m/s, + up
    groundspeed: float | None = None  # m/s
    gps_fix: str | None = None  # GpsFixType
    satellites: int | None = None
    # {"id", "remaining" (0..1), "voltage", "state"} each, by id.
    batteries: tuple[dict, ...] = ()
    # {"id", "pitch", "yaw", "yaw_lock", "range"} each, by id; as Gimbal's.
    # "range" is None without a rangefinder, else {"status", "distance",
    # "min", "max", "accuracy", "place"}: BearingDetection's, with
    # range_accuracy as accuracy, and place, where the spot is.
    gimbals: tuple[dict, ...] = ()

    def update(
        self,
        robot: Robot,
        home: Pose | None,
        gimbals: Iterable[tuple[int, Gimbal, BearingDetection | None]],
    ) -> None:
        """Follow *robot*, whose mount is in local NED, as is *home*'s.

        *gimbals* are its gimbals by device id, each with its rangefinder's
        spot, if it has one. With the autopilot's link down, the rest is
        stale, so it's all unknown.
        """
        self.connected = robot.connected
        if not robot.connected:
            self.armed, self.mode, self.failsafe = False, "", None
            self.roll = self.pitch = self.heading = None
            self.altitude = self.climb = self.groundspeed = None
            self.gps_fix = self.satellites = None
            self.batteries = self.gimbals = ()
            return
        self.armed = robot.armed
        self.mode = robot.mode
        self.failsafe = (
            (robot.failsafe_text or ", ".join(robot.failsafe))
            if robot.failsafe
            else None
        )
        mount = robot.mount
        if mount is not None and mount.orientation is not None:
            roll, pitch, yaw = mount.orientation.to_euler(degrees=True)
            self.roll, self.pitch = _round(roll), _round(pitch)
            self.heading = _round(yaw % 360) % 360
        else:
            self.roll = self.pitch = self.heading = None
        self.altitude = (
            _round(home.offset.z - mount.offset.z)
            if mount is not None and home is not None
            else None
        )
        v = robot.velocity
        self.climb = None if v is None else _round(-v.down)
        self.groundspeed = None if v is None else _round(math.hypot(v.north, v.east))
        self.gps_fix = robot.gps.fix if robot.gps else None
        self.satellites = robot.gps.satellites if robot.gps else None
        self.batteries = tuple(
            {
                "id": b.id,
                "remaining": b.remaining,
                "voltage": None if b.voltage is None else _round(b.voltage),
                "state": b.state,
            }
            for b in robot.batteries
        )
        self.gimbals = tuple(
            {
                "id": device_id,
                "pitch": None if g.pitch is None else _round(g.pitch),
                "yaw": None if g.yaw is None else _round(g.yaw),
                "yaw_lock": g.yaw_lock,
                "range": None if spot is None else _reading(spot),
            }
            for device_id, g, spot in gimbals
        )


def _round(value: float) -> float:
    # To a tenth, without a -0.0 to show as "-0.0".
    return round(value, 1) + 0.0


def _reading(spot: BearingDetection) -> dict:
    return {
        "status": spot.status,
        "distance": None if spot.distance is None else _round(spot.distance),
        "min": spot.min,
        "max": spot.max,
        "accuracy": None
        if spot.range_accuracy is None
        else _round(spot.range_accuracy),
        "place": coordinates(spot.pose) if spot.status == "ok" else None,
    }


def coordinates(pose: Pose | None) -> str | None:
    """A pose's place as "lat lon" in degrees, to no more decimals than its accuracy.

    None without a pose: the tree isn't on the globe until the EKF has an
    origin. A grid reference (MGRS) would be a change here.
    """
    if pose is None:
        return None
    lat, lon, _ = pose.offset.to_geodetic()
    accuracy = pose.accuracy.horizontal if pose.accuracy else None
    # The fewest decimals whose last digit is finer than the accuracy; ~1 m
    # when it's unknown.
    decimals = (
        min(7, max(0, math.ceil(math.log10(METRES_PER_DEGREE / accuracy))))
        if accuracy
        else 5
    )
    return f"{lat:.{decimals}f} {lon:.{decimals}f}"


# Every template reads its instruments as `i`, always as `objects.i.<field>`,
# never through a variable set to `objects.i`: a viewer drawing the widgets
# itself (the debug map) follows the fields it can see a template read, and
# would see none. Boxes are black at 55%, as
# SpiriCamera's own widgets are, and every widget has an 8 px margin so it
# isn't flush with the frame's edge. Miracode is 10.6 px a character at 16 px
# and 8.6 at 13 px, as thorvg draws it.

_PREAMBLE = """\
{# object: i = $instruments #}
"""

# A round attitude indicator, 3 px a degree of pitch: sky and ground turn with
# the horizon and its pitch ladder, inside the dial; the roll scale and the
# airframe's symbol stay put. Roll and pitch underneath, to the degree.
# thorvg draws no strokes in the clipped, rotated group, so its lines are
# thin rects. `-objects.i.roll` needs MiniJinja 3 (SpiriCamera's minimum):
# 2.x negates `objects` before reading `.i.roll` from it, and fails.
HORIZON_SVG = (
    _PREAMBLE
    + """\
<svg xmlns="http://www.w3.org/2000/svg" width="196" height="226">
<g transform="translate(8 8)">
<defs><clipPath id="dial"><circle cx="90" cy="90" r="80"/></clipPath></defs>
{% if objects.i.roll is none or objects.i.pitch is none %}
<circle cx="90" cy="90" r="80" fill="black" fill-opacity="0.55"/>
<text x="32" y="96" font-family="$font" font-size="16" fill="white">NO ATTITUDE</text>
{% else %}
<g clip-path="url(#dial)">
<g transform="rotate({{ -objects.i.roll }} 90 90) translate(0 {{ objects.i.pitch * 3 }})">
<rect x="-300" y="-500" width="680" height="590" fill="#2f6fbf" fill-opacity="0.8"/>
<rect x="-300" y="90" width="680" height="590" fill="#7a4e24" fill-opacity="0.8"/>
<rect x="-300" y="89" width="680" height="2" fill="white"/>
{% for p in range(-80, 85, 5) if p %}{% set y = 90 - p * 3 %}
{% if p % 10 %}<rect x="82" y="{{ y - 0.75 }}" width="16" height="1.5" fill="white"/>
{% else %}<rect x="68" y="{{ y - 0.75 }}" width="44" height="1.5" fill="white"/>
<text x="{{ 64 - (p | abs | string | length) * 8.6 }}" y="{{ y + 5 }}" font-family="$font" font-size="13" fill="white">{{ p | abs }}</text>
<text x="116" y="{{ y + 5 }}" font-family="$font" font-size="13" fill="white">{{ p | abs }}</text>{% endif %}
{% endfor %}
</g>
</g>
<circle cx="90" cy="90" r="80" fill="none" stroke="white" stroke-opacity="0.7" stroke-width="2"/>
{% for a in [-60, -45, -30, -20, -10, 10, 20, 30, 45, 60] %}
<path d="M90 10 V{{ 20 if a % 30 == 0 else 16 }}" stroke="white" stroke-width="2" transform="rotate({{ a }} 90 90)"/>
{% endfor %}
<path d="M90 9 l-6 -9 h12 z" fill="white"/>
<path d="M90 12 l-6 10 h12 z" fill="#ffcc00" stroke="black" stroke-width="1" transform="rotate({{ -objects.i.roll }} 90 90)"/>
<g fill="none" stroke-linecap="round" stroke-linejoin="round">
<path d="M42 90 H72 L80 100 M138 90 H108 L100 100" stroke="black" stroke-opacity="0.6" stroke-width="6"/>
<path d="M42 90 H72 L80 100 M138 90 H108 L100 100" stroke="#ffcc00" stroke-width="3"/>
</g>
<circle cx="90" cy="90" r="3" fill="#ffcc00" stroke="black" stroke-width="1"/>
{% endif %}
{% if objects.i.roll is not none and objects.i.pitch is not none %}
{% set p = objects.i.pitch | round | int %}{% set r = objects.i.roll | round | int %}
{% set text = "P " ~ ("+" if p > 0 else "") ~ p ~ "  R " ~ ("+" if r > 0 else "") ~ r %}
<rect x="{{ 90 - (text | length) * 5.3 - 6 }}" y="184" width="{{ (text | length) * 10.6 + 12 }}" height="24" fill="black" fill-opacity="0.55"/>
<text x="{{ 90 - (text | length) * 5.3 }}" y="202" font-family="$font" font-size="16" fill="white">{{ text }}</text>
{% endif %}
</g>
</svg>"""
)

# A compass tape, 3 px a degree, ticks every 5° and labels every 30° (N, 3,
# 6, E, 12, ...), under the heading to the degree.
HEADING_SVG = (
    _PREAMBLE
    + """\
<svg xmlns="http://www.w3.org/2000/svg" width="316" height="68">
<g transform="translate(8 8)">
<defs><clipPath id="tape"><rect x="0" y="24" width="300" height="28"/></clipPath></defs>
{% set h = objects.i.heading %}
{% set text = "---" if h is none else ("00" ~ ((h | round | int) % 360))[-3:] %}
<rect x="122" y="0" width="56" height="22" fill="black" fill-opacity="0.55"/>
<text x="{{ 150 - (text | length) * 5.3 }}" y="17" font-family="$font" font-size="16" fill="white">{{ text }}</text>
<rect x="0" y="24" width="300" height="28" fill="black" fill-opacity="0.55"/>
{% if h is not none %}
<g clip-path="url(#tape)">
{% set first = ((h - 50) // 5) | int * 5 %}
{% for d in range(first, first + 105, 5) %}{% set x = 150 + (d - h) * 3 %}{% set deg = (d + 360) % 360 %}
<path d="M{{ x }} 24 V{{ 34 if deg % 10 == 0 else 29 }}" stroke="white" stroke-width="1.5"/>
{% if deg % 30 == 0 %}{% set label = ["N", "3", "6", "E", "12", "15", "S", "21", "24", "W", "30", "33"][deg // 30] %}
<text x="{{ x - (label | length) * 4.3 }}" y="48" font-family="$font" font-size="13" fill="{{ '#ffcc00' if deg % 90 == 0 else 'white' }}">{{ label }}</text>{% endif %}
{% endfor %}
</g>
<path d="M144 24 L150 31 L156 24 z" fill="#ffcc00"/>
{% endif %}
</g>
</svg>"""
)

# One line each, coloured by what it says: the link, armed, mode, failsafe
# and GPS. With the autopilot's link down, only that: the rest is stale.
STATUS_SVG = (
    _PREAMBLE
    + """\
{% if not objects.i.connected %}{% set lines = [["NO AUTOPILOT LINK", "#ff5555"]] %}
{% else %}
{% set lines = [["ARMED", "#ffcc00"] if objects.i.armed else ["DISARMED", "white"], [objects.i.mode or "--", "white"]] %}
{% if objects.i.failsafe %}{% set lines = lines + [["FAILSAFE " ~ objects.i.failsafe, "#ff5555"]] %}{% endif %}
{% set gps = "GPS " ~ ((objects.i.gps_fix | upper) if objects.i.gps_fix else "--") ~ ((" " ~ objects.i.satellites ~ " sats") if objects.i.satellites is not none else "") %}
{% set lines = lines + [[gps, "white" if objects.i.gps_fix not in [none, "none", "2d"] else "#ffcc00"]] %}
{% endif %}
{% set ns = namespace(w=0) %}{% for l in lines %}{% set ns.w = [ns.w, l[0] | length] | max %}{% endfor %}
<svg xmlns="http://www.w3.org/2000/svg" width="{{ ns.w * 10.6 + 28 }}" height="{{ (lines | length) * 22 + 24 }}">
<rect x="8" y="8" width="{{ ns.w * 10.6 + 12 }}" height="{{ (lines | length) * 22 + 8 }}" fill="black" fill-opacity="0.55"/>
{% for l in lines %}<text x="14" y="{{ 28 + loop.index0 * 22 }}" font-family="$font" font-size="16" fill="{{ l[1] }}">{{ l[0] | escape }}</text>
{% endfor %}
</svg>"""
)

# Height above home, climb rate and ground speed, as QGroundControl names them.
FLIGHT_SVG = (
    _PREAMBLE
    + """\
{% macro value(v, unit, sign=false) %}{% if v is none %}--{% else %}{{ "+" if sign and v > 0 else "" }}{{ v }} {{ unit }}{% endif %}{% endmacro %}
{% set lines = [
  "ALT (REL) " ~ value(objects.i.altitude, "m"),
  "CLIMB     " ~ value(objects.i.climb, "m/s", true),
  "GND SPEED " ~ value(objects.i.groundspeed, "m/s"),
] %}
{% set w = (lines | map("length") | max) * 10.6 %}
<svg xmlns="http://www.w3.org/2000/svg" width="{{ w + 28 }}" height="90">
<rect x="8" y="8" width="{{ w + 12 }}" height="74" fill="black" fill-opacity="0.55"/>
{% for l in lines %}<text x="14" y="{{ 28 + loop.index0 * 22 }}" font-family="$font" font-size="16" fill="white">{{ l }}</text>
{% endfor %}
</svg>"""
)

# Each battery's charge and voltage, numbered when there's more than one, in
# amber when the autopilot calls it low and red past that.
BATTERY_SVG = (
    _PREAMBLE
    + """\
{% set ns = namespace(lines=[]) %}
{% for b in objects.i.batteries %}
{% set name = "BAT" ~ ((b.id + 1) if objects.i.batteries | length > 1 else "") %}
{% set charge = (" " ~ (b.remaining * 100) | round | int ~ "%") if b.remaining is not none else "" %}
{% set volts = (" " ~ b.voltage ~ " V") if b.voltage is not none else "" %}
{% set colour = "#ffcc00" if b.state == "low" else "#ff5555" if b.state in ["critical", "emergency", "failed", "unhealthy"] else "white" %}
{% set ns.lines = ns.lines + [[name ~ charge ~ volts, colour]] %}
{% endfor %}
{% set lines = ns.lines or [["BAT --", "white"]] %}
{% set w = (lines | map(attribute=0) | map("length") | max) * 10.6 %}
<svg xmlns="http://www.w3.org/2000/svg" width="{{ w + 28 }}" height="{{ (lines | length) * 22 + 24 }}">
<rect x="8" y="8" width="{{ w + 12 }}" height="{{ (lines | length) * 22 + 8 }}" fill="black" fill-opacity="0.55"/>
{% for l in lines %}<text x="14" y="{{ 28 + loop.index0 * 22 }}" font-family="$font" font-size="16" fill="{{ l[1] }}">{{ l[0] }}</text>
{% endfor %}
</svg>"""
)

# Where a gimbal points: pitch from the horizon and yaw from the robot's
# heading, as Gimbal has them, and whether it holds an earth heading.
GIMBAL_SVG = (
    _PREAMBLE
    + """\
{% set ns = namespace(g=none) %}
{% for g in objects.i.gimbals if g.id == $gimbal %}{% set ns.g = g %}{% endfor %}
{% set g = ns.g %}
{% macro angle(v) %}{% if v is none %}--{% else %}{{ "+" if v > 0 else "" }}{{ v | round | int }}°{% endif %}{% endmacro %}
{% set lines = ["GIMBAL $gimbal" ~ ("" if g else " --")] %}
{% if g %}{% set lines = lines + ["PITCH " ~ angle(g.pitch), "YAW   " ~ angle(g.yaw) ~ (" LOCK" if g.yaw_lock else "")] %}{% endif %}
{% set w = (lines | map("length") | max) * 10.6 %}
<svg xmlns="http://www.w3.org/2000/svg" width="{{ w + 28 }}" height="{{ (lines | length) * 22 + 24 }}">
<rect x="8" y="8" width="{{ w + 12 }}" height="{{ (lines | length) * 22 + 8 }}" fill="black" fill-opacity="0.55"/>
{% for l in lines %}<text x="14" y="{{ 28 + loop.index0 * 22 }}" font-family="$font" font-size="16" fill="white">{{ l }}</text>
{% endfor %}
</svg>"""
)

# A gimbal's rangefinder: a reticle on the camera's axis, which it shares,
# and the reading in a box to its right, with the spot's coordinates under it
# when it has a place. The reticle sits at the widget's centre, so centred on
# the frame it marks the axis: the widget is as wide again on the reticle's
# left and as tall again above it, so it stays centred whatever it shows.
RANGEFINDER_SVG = (
    _PREAMBLE
    + """\
{% set ns = namespace(r=none) %}
{% for g in objects.i.gimbals if g.id == $gimbal %}{% set ns.r = g.range %}{% endfor %}
{% set r = ns.r %}
{% if r and r.status == "ok" %}{% set reading = r.distance ~ " m" %}
{%- if r.accuracy is not none %}{% set reading = reading ~ " ±" ~ r.accuracy %}{% endif %}
{%- elif r and r.status == "far" %}{% set reading = "> " ~ (r.max | round | int) ~ " m" %}
{%- elif r and r.status == "near" %}{% set reading = "< " ~ r.min ~ " m" %}
{%- else %}{% set reading = "no reading" %}{% endif %}
{% set place = r.place if r and r.place else "" %}
{% set box = ([reading | length, place | length] | max) * 10.6 + 12 %}
{% set c = 22 + box %}
{% set m = 38 if place else 20 %}
<svg xmlns="http://www.w3.org/2000/svg" width="{{ 2 * c }}" height="{{ 2 * m }}">
<g transform="translate({{ c - 150 }} {{ m - 20 }})">
<g stroke-linecap="round">
<path d="M150 4 V14 M150 26 V36 M134 20 H144 M156 20 H166" stroke="black" stroke-opacity="0.6" stroke-width="4"/>
<path d="M150 4 V14 M150 26 V36 M134 20 H144 M156 20 H166" stroke="white" stroke-width="2"/>
</g>
<rect x="172" y="8" width="{{ box }}" height="{{ 50 if place else 24 }}" fill="black" fill-opacity="0.55"/>
<text x="178" y="26" font-family="$font" font-size="16" fill="white">{{ reading }}</text>
{% if place %}<text x="178" y="50" font-family="$font" font-size="16" fill="white">{{ place }}</text>{% endif %}
</g>
</svg>"""
)

# Markers on places in the world, drawn where they are in the picture: the
# camera places each on its object (SpiriCamera's `{# anchor: #}`), so the
# template only draws around its own centre, and gets `view` for how far
# away it is. Not drawn while the place is out of shot, behind the camera,
# or unset (no home yet, no ROI). The widget is as tall again above the
# mark as the label is below it, so the mark is its centre.
MARKER_SVG = """\
{# object: place = $place #}
{# anchor: place #}
{% set label = "$name " ~ (view.range | round | int) ~ " m" %}
{% set w = [(label | length) * 8.6 + 12, 28] | max %}
<svg xmlns="http://www.w3.org/2000/svg" width="{{ w }}" height="68">
<g fill="none" stroke-linejoin="round">
<path d="$mark" transform="translate({{ w / 2 }} 34)" stroke="black" stroke-opacity="0.6" stroke-width="4"/>
<path d="$mark" transform="translate({{ w / 2 }} 34)" stroke="$colour" stroke-width="2"/>
</g>
<rect x="0" y="46" width="{{ w }}" height="20" fill="black" fill-opacity="0.55"/>
<text x="6" y="61" font-family="$font" font-size="13" fill="$colour">{{ label }}</text>
</svg>"""

# name: (label, mark around (0, 0), colour)
MARKERS = {
    "home": ("HOME", "M0 -8 L8 0 L0 8 L-8 0 Z", "white"),
    "roi": ("ROI", "M-8 -8 H8 V8 H-8 Z M0 -3 V3 M-3 0 H3", "#ffcc00"),
}

# name: (template, default anchor)
ROBOT_WIDGETS = {
    "horizon": (HORIZON_SVG, "bottom_middle"),
    "heading": (HEADING_SVG, "top_middle"),
    "status": (STATUS_SVG, "top_left"),
    "flight": (FLIGHT_SVG, "bottom_right"),
    "battery": (BATTERY_SVG, "top_right"),
}


def robot_widgets(instruments: Instruments) -> list[HudWidget]:
    """The robot's widgets, at ``hud/widgets/<name>``, reading *instruments*."""
    return [
        HudWidget(
            synq_topic=f"{WIDGETS_TOPIC}/{name}",
            synq_authoritive=True,
            anchor=anchor,
            svg_template=Template(svg).substitute(
                instruments=instruments.synq_absolute_path, font=FONT
            ),
        )
        for name, (svg, anchor) in ROBOT_WIDGETS.items()
    ]


def marker_widget(name: str, place: Position) -> HudWidget:
    """The :data:`MARKERS` *name* marker on *place*, at ``<place's topic>_hud``."""
    label, mark, colour = MARKERS[name]
    return HudWidget(
        synq_topic=f"{place.synq_topic}_hud",
        synq_authoritive=True,
        svg_template=Template(MARKER_SVG).substitute(
            place=place.synq_absolute_path,
            name=label,
            mark=mark,
            colour=colour,
            font=FONT,
        ),
    )


def gimbal_widget(device_id: int, instruments: Instruments) -> HudWidget:
    """Gimbal *device_id*'s widget, at ``hud/widgets/gimbal<device_id>``."""
    return _gimbal_widget(
        f"{WIDGETS_TOPIC}/gimbal{device_id}",
        GIMBAL_SVG,
        "bottom_left",
        device_id,
        instruments,
    )


def rangefinder_widget(
    gimbal_topic: str, device_id: int, instruments: Instruments
) -> HudWidget:
    """Gimbal *device_id*'s rangefinder widget, at ``<gimbal_topic>_range_hud``."""
    return _gimbal_widget(
        f"{gimbal_topic}_range_hud",
        RANGEFINDER_SVG,
        "center_middle",
        device_id,
        instruments,
    )


def _gimbal_widget(
    topic: str, svg: str, anchor: str, device_id: int, instruments: Instruments
) -> HudWidget:
    return HudWidget(
        synq_topic=topic,
        synq_authoritive=True,
        anchor=anchor,
        svg_template=Template(svg).substitute(
            instruments=instruments.synq_absolute_path,
            font=FONT,
            gimbal=device_id,
        ),
    )
