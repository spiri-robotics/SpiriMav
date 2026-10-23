"""Tests for SpiriMav CLI."""

from __future__ import annotations

from spiri_mav.cli import app


def test_cli_version() -> None:
    """Test CLI version command."""

    from typer.testing import CliRunner

    runner = CliRunner()
    result = runner.invoke(app, ["version"])
    assert result.exit_code == 0


def test_cli_help() -> None:
    """Test CLI help command."""
    from typer.testing import CliRunner

    runner = CliRunner()
    result = runner.invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "SpiriMav" in result.stdout


def test_cli_run_overrides_settings(monkeypatch) -> None:
    """Options on `run` override SPIRI_MAV_* settings; unset ones keep them."""
    from typer.testing import CliRunner

    import spiri_mav.main

    served = []
    monkeypatch.setenv("SPIRI_MAV_TARGET_SYSID", "3")
    monkeypatch.setattr(
        spiri_mav.main, "serve", lambda settings: served.append(settings)
    )
    result = CliRunner().invoke(
        app, ["run", "--url", "udpin:0.0.0.0:14550", "--sysid", "200"]
    )
    assert result.exit_code == 0, result.output
    [settings] = served
    assert (settings.mavlink_url, settings.sysid, settings.target_sysid) == (
        "udpin:0.0.0.0:14550",
        200,
        3,
    )


def test_serve_closes_robot_on_interrupt(monkeypatch) -> None:
    from spiri_mav.main import Settings, serve
    from spiri_mav.mav_robot import MavRobot

    calls = []

    def fake_run(self, url, stop=None):
        calls.append((self.synq_topic, self.sysid, url))
        raise KeyboardInterrupt

    monkeypatch.setattr(MavRobot, "run", fake_run)
    monkeypatch.setattr(MavRobot, "close", lambda self: calls.append("closed"))
    serve(
        Settings(topic="test/serve_robot", sysid=200, mavlink_url="tcp:127.0.0.1:5760")
    )
    assert calls == [("test/serve_robot", 200, "tcp:127.0.0.1:5760"), "closed"]


def test_gimbal_cameras_from_environment(monkeypatch) -> None:
    from spiri_mav.main import Settings

    monkeypatch.setenv("SPIRI_MAV_GIMBAL_CAMERAS", '{"1": ["mu-1/camera_eo"]}')
    assert Settings().gimbal_cameras == {1: ["mu-1/camera_eo"]}
