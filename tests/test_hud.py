"""Tests for the HUD: the instruments its widgets read, and the widgets."""

from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from dataclasses import fields

import minijinja
import pytest
from SpiriSynq.example_types.position import Orientation
from SpiriSynq.syncable_objects import SyncableObject

from spiri_mav import hud, mav_robot
from spiri_mav.hud import HudWidget, Instruments
from spiri_mav.mav_robot import MavRobot

from .test_mav_robot import (
    FakeLink,
    _wait_for,
    distance_sensor,
    from_src,
    gimbal_attitude,
    heartbeat,
    mavlink,
    robot_attitude,
)


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


INSTRUMENT_FIELDS = [
    f.name
    for f in fields(Instruments)
    if f.name not in {f.name for f in fields(SyncableObject)}
]


def home_at(robot, down: float):
    robot.handle(
        from_src("home_position", 0, 0, 0, 0.0, 0.0, down, [1, 0, 0, 0], 0, 0, 0)
    )


def test_instruments_follow_the_robot(connected):
    connected.handle(heartbeat(armed=True, custom_mode=4))
    connected.handle(from_src("local_position_ned", 0, 0.0, 0.0, -12.0, 3.0, 4.0, -1.0))
    robot_attitude(connected, roll=10, pitch=-5, yaw=-90)
    home_at(connected, -2.0)
    connected.handle(
        from_src(
            "gps_raw_int", 0, mavlink.GPS_FIX_TYPE_3D_FIX, 0, 0, 0, 80, 120, 0, 0, 14
        )
    )
    cells = [4200, 4100, 4000] + [65535] * 7
    connected.handle(from_src("battery_status", 0, 0, 0, 2500, cells, 1000, -1, -1, 70))
    connected.tick(100.0)

    i = connected._instruments
    assert (i.connected, i.armed, i.mode) == (True, True, "GUIDED")
    assert (i.roll, i.pitch, i.heading) == (10.0, -5.0, 270.0)
    assert i.altitude == 10.0  # 12 m up, home 2 m up
    assert (i.climb, i.groundspeed) == (1.0, 5.0)
    assert (i.gps_fix, i.satellites) == ("3d", 14)
    assert i.batteries == (
        {"id": 0, "remaining": 0.7, "voltage": 12.3, "state": "unknown"},
    )


def test_instruments_unknown_until_reported(connected):
    connected.tick(100.0)
    i = connected._instruments
    assert i.connected
    assert None is i.roll is i.heading is i.altitude is i.climb is i.gps_fix
    assert i.batteries == i.gimbals == ()


def test_no_altitude_without_home(connected):
    connected.handle(from_src("local_position_ned", 0, 0.0, 0.0, -12.0, 0, 0, 0))
    connected.tick(100.0)
    assert connected._instruments.altitude is None


def test_heading_wraps_to_north(connected):
    robot_attitude(connected, yaw=-0.01)
    connected.handle(from_src("local_position_ned", 0, 0.0, 0.0, 0.0, 0, 0, 0))
    connected.tick(100.0)
    assert connected._instruments.heading == 0.0


def test_link_down_leaves_nothing_known(connected):
    connected.handle(heartbeat(armed=True))
    connected.handle(from_src("local_position_ned", 0, 0.0, 0.0, -12.0, 0, 0, 0))
    robot_attitude(connected, roll=10)
    connected.tick(time.monotonic() + mav_robot.HEARTBEAT_TIMEOUT + 1)
    i = connected._instruments
    assert not i.connected and not i.armed
    assert None is i.roll is i.groundspeed
    assert i.batteries == ()


def test_instruments_update_at_most_every_period(connected):
    robot_attitude(connected, roll=10)
    connected.handle(from_src("local_position_ned", 0, 0.0, 0.0, 0.0, 0, 0, 0))
    connected.tick(100.0)
    robot_attitude(connected, roll=20)
    connected.tick(100.0 + mav_robot.HUD_PERIOD / 2)
    assert connected._instruments.roll == 10.0
    connected.tick(100.0 + mav_robot.HUD_PERIOD * 1.01)
    assert connected._instruments.roll == 20.0


def test_gimbals_are_in_the_instruments_and_get_a_widget(connected):
    robot_attitude(connected)
    q = Orientation.from_euler(0, math.radians(-30), math.radians(20))
    connected.handle(gimbal_attitude(q))
    connected.tick(100.0)
    [device_id] = connected._gimbals
    assert connected._instruments.gimbals == (
        {
            "id": device_id,
            "pitch": -30.0,
            "yaw": 20.0,
            "yaw_lock": False,
            "range": None,
        },
    )
    widget = connected._hud[-1]
    assert widget.synq_topic == f"hud/widgets/gimbal{device_id}"
    assert f"g.id == {device_id}" in widget.svg_template


def test_rangefinder_reading_and_place_are_in_the_instruments(connected):
    connected._gimbal_rangefinders = {154: 0}
    connected.handle(from_src("gps_global_origin", 446470000, -635800000, 0))
    connected.handle(from_src("local_position_ned", 0, 0, 0, -10, 0, 0, 0))
    robot_attitude(connected)
    connected.handle(gimbal_attitude(Orientation()))
    connected.tick(100.0)
    assert connected._instruments.gimbals[0]["range"] is None  # none reported
    connected.handle(distance_sensor(1234))
    spot = connected._spots[154]
    assert _wait_for(lambda: spot.pose is not None)
    connected.tick(101.0)
    reading = connected._instruments.gimbals[0]["range"]
    assert reading == {
        "status": "ok",
        "distance": 12.3,
        "min": 0.2,
        "max": 650.0,
        "accuracy": None,
        "place": hud.coordinates(spot.pose),
    }
    assert reading["place"].startswith("44.64")


def instrument_widgets(widgets):
    """The widgets reading the instruments: all but the markers on places."""
    return [w for w in widgets if "{# anchor:" not in w.svg_template]


def test_generic_widgets_are_published_under_hud_widgets(robot):
    assert robot._instruments.synq_topic == "test/robot_instruments"
    assert [w.synq_topic for w in instrument_widgets(robot._hud)] == [
        f"hud/widgets/{name}" for name in hud.ROBOT_WIDGETS
    ]
    for widget in instrument_widgets(robot._hud):
        assert (
            f"{{# object: i = {robot._instruments.synq_absolute_path} #}}"
            in widget.svg_template
        )


def test_widget_is_spiricameras_type():
    # Cameras and UIs find widgets by the type's name and mirror its fields.
    assert HudWidget.__name__ == "HudWidget"
    assert [f.name for f in fields(HudWidget)][-5:] == [
        "svg_template",
        "anchor",
        "custom_anchor_x",
        "custom_anchor_y",
        "z_index",
    ]


# ── Rendering, as a camera does: MiniJinja, strict about what's undefined ─────

FLYING = dict(  # noqa: C408 (as the states below, which extend it)
    connected=True,
    armed=True,
    mode="GUIDED",
    failsafe=None,
    roll=-14.3,
    pitch=6.2,
    heading=352.4,
    altitude=23.4,
    climb=-0.6,
    groundspeed=4.2,
    gps_fix="3d",
    satellites=14,
    batteries=({"id": 0, "remaining": 0.76, "voltage": 15.8, "state": "ok"},),
    gimbals=(
        {
            "id": 1,
            "pitch": -32.0,
            "yaw": 12.4,
            "yaw_lock": False,
            "range": {
                "status": "ok",
                "distance": 123.4,
                "min": 0.2,
                "max": 650.0,
                "accuracy": 1.3,
                "place": "44.64700 -63.58000",
            },
        },
    ),
)
UNKNOWN = dict(
    FLYING,
    armed=False,
    mode="",
    roll=None,
    pitch=None,
    heading=None,
    altitude=None,
    climb=None,
    groundspeed=None,
    gps_fix=None,
    satellites=None,
    batteries=(),
    gimbals=(),
)
TROUBLE = dict(
    FLYING,
    failsafe="Battery 1 is critical",
    roll=38.0,
    pitch=-89.0,
    heading=2.0,
    gps_fix="none",
    batteries=(
        {"id": 0, "remaining": 0.12, "voltage": 13.9, "state": "critical"},
        {"id": 1, "remaining": None, "voltage": None, "state": "low"},
    ),
    gimbals=(
        {
            "id": 1,
            "pitch": None,
            "yaw": None,
            "yaw_lock": True,
            "range": {
                "status": "far",
                "distance": None,
                "min": 0.2,
                "max": 650.0,
                "accuracy": None,
                "place": None,
            },
        },
    ),
)


def render(template: str, instruments: dict) -> ET.Element:
    env = minijinja.Environment()
    env.undefined_behavior = "strict"
    return ET.fromstring(env.render_str(template, objects={"i": instruments}).strip())


def texts(svg: ET.Element) -> list[str]:
    return [t.text for t in svg.iter("{http://www.w3.org/2000/svg}text")]


@pytest.fixture
def widgets(robot):
    robot._gimbal_rangefinders = {1: 0}
    robot.handle(gimbal_attitude(Orientation(), device_id=1))
    robot.handle(distance_sensor(1234))
    return {
        w.synq_topic.removeprefix("hud/widgets/").removeprefix("test/robot_"): w
        for w in instrument_widgets(robot._hud)
    }


@pytest.mark.parametrize(
    "state", [FLYING, UNKNOWN, TROUBLE], ids=["flying", "unknown", "trouble"]
)
def test_every_widget_renders_every_state(widgets, state):
    assert sorted(state) == sorted(INSTRUMENT_FIELDS)
    for name, widget in widgets.items():
        svg = render(widget.svg_template, state)
        assert float(svg.get("width")) > 0 and float(svg.get("height")) > 0, name


def test_widgets_read_fields_a_viewer_can_follow(widgets):
    # The debug map follows the fields its static analysis finds; read
    # through a variable, it would find none and never update.
    env = minijinja.Environment()
    for name, widget in widgets.items():
        reads = {
            r
            for r in env.undeclared_variables_in_str(widget.svg_template, nested=True)
            if r.startswith("objects")
        }
        assert reads, name
        assert all(r.startswith("objects.i.") for r in reads), (name, reads)


def test_widgets_show_what_they_read(widgets):
    assert texts(render(widgets["horizon"].svg_template, FLYING))[-1] == "P +6  R -14"
    assert texts(render(widgets["heading"].svg_template, FLYING))[0] == "352"
    assert texts(render(widgets["status"].svg_template, FLYING)) == [
        "ARMED",
        "GUIDED",
        "GPS 3D 14 sats",
    ]
    assert texts(render(widgets["flight"].svg_template, FLYING)) == [
        "ALT (REL) 23.4 m",
        "CLIMB     -0.6 m/s",
        "GND SPEED 4.2 m/s",
    ]
    assert texts(render(widgets["battery"].svg_template, FLYING)) == ["BAT 76% 15.8 V"]
    assert texts(render(widgets["gimbal1"].svg_template, FLYING)) == [
        "GIMBAL 1",
        "PITCH -32°",
        "YAW   +12°",
    ]


def test_rangefinder_widget_shows_the_reading(widgets):
    range_hud = widgets["gimbal1_range_hud"].svg_template
    assert texts(render(range_hud, FLYING)) == ["123.4 m ±1.3", "44.64700 -63.58000"]
    assert texts(render(range_hud, TROUBLE)) == ["> 650 m"]
    assert texts(render(range_hud, UNKNOWN)) == ["no reading"]


def test_widgets_say_what_they_dont_know(widgets):
    assert "NO ATTITUDE" in texts(render(widgets["horizon"].svg_template, UNKNOWN))
    assert texts(render(widgets["heading"].svg_template, UNKNOWN)) == ["---"]
    assert texts(render(widgets["battery"].svg_template, UNKNOWN)) == ["BAT --"]
    assert texts(render(widgets["gimbal1"].svg_template, UNKNOWN)) == ["GIMBAL 1 --"]
    assert texts(
        render(widgets["status"].svg_template, dict(UNKNOWN, connected=False))
    ) == ["NO AUTOPILOT LINK"]


def test_trouble_is_in_colour(widgets):
    status = render(widgets["status"].svg_template, TROUBLE)
    colours = {
        t.text: t.get("fill") for t in status.iter("{http://www.w3.org/2000/svg}text")
    }
    assert colours["FAILSAFE Battery 1 is critical"] == "#ff5555"
    assert colours["GPS NONE 14 sats"] == "#ffcc00"
    battery = render(widgets["battery"].svg_template, TROUBLE)
    assert [
        (t.text, t.get("fill"))
        for t in battery.iter("{http://www.w3.org/2000/svg}text")
    ] == [
        ("BAT1 12% 13.9 V", "#ff5555"),
        ("BAT2", "#ffcc00"),
    ]


# ── Markers on places, which the camera puts where they are in the picture ───


def markers(robot):
    return {
        w.synq_topic.removeprefix("test/robot_"): w
        for w in robot._hud
        if "{# anchor:" in w.svg_template
    }


def test_markers_are_on_home_and_the_roi(robot):
    found = markers(robot)
    assert sorted(found) == ["home_hud", "roi_hud"]
    for name, place in (("home_hud", robot._home), ("roi_hud", robot._roi)):
        template = found[name].svg_template
        assert f"{{# object: place = {place.synq_absolute_path} #}}" in template
        assert "{# anchor: place #}" in template


def test_marker_centres_its_mark_and_says_how_far(robot):
    env = minijinja.Environment()
    env.undefined_behavior = "strict"
    view = {"x": 0, "y": 0, "r": None, "range": 123.4, "edge": False}
    svg = ET.fromstring(
        env.render_str(
            markers(robot)["home_hud"].svg_template, objects={}, view=view
        ).strip()
    )
    assert texts(svg) == ["HOME 123 m"]
    width, height = float(svg.get("width")), float(svg.get("height"))
    # The mark is drawn at the widget's centre, which the camera places.
    marks = [p.get("transform") for p in svg.iter("{http://www.w3.org/2000/svg}path")]
    assert marks == [f"translate({width / 2} {height / 2:g})"] * 2
