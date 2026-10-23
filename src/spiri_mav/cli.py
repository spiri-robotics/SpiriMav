"""CLI interface for SpiriMav."""

from __future__ import annotations

import sys
from typing import Annotated

import typer
from loguru import logger

from spiri_mav import __version__

app = typer.Typer(
    name="SpiriMav",
    help="MAVLink vehicle bridge for SpiriSynq: a companion-computer GCS that mirrors autopilot state",
    no_args_is_help=True,
    pretty_exceptions_short=True,
)


@app.callback(invoke_without_command=True)
def main_callback(
    ctx: typer.Context,
    verbose: Annotated[
        bool, typer.Option("--verbose", "-v", help="Enable verbose logging")
    ] = False,
):
    """Enter callback for SpiriMav."""
    if ctx.invoked_subcommand is None:
        typer.echo(ctx.get_help())
        ctx.exit()

    log_level = "DEBUG" if verbose else "INFO"
    logger.remove()
    logger.add(sys.stderr, level=log_level)


@app.command()
def version() -> None:
    """Show version information."""
    typer.echo(f"SpiriMav v{__version__}")
    typer.echo(f"Python {sys.version.split()[0]}")


@app.command()
def run(
    url: Annotated[
        str | None,
        typer.Option(
            "--url", "-u", help="MAVLink connection string, e.g. /dev/ttyAMA0,921600"
        ),
    ] = None,
    sysid: Annotated[
        int | None, typer.Option(help="Our MAVLink sysid; never the FCU's GCS sysid")
    ] = None,
    target_sysid: Annotated[
        int | None, typer.Option(help="The autopilot's sysid")
    ] = None,
    topic: Annotated[
        str | None, typer.Option(help="SpiriSynq topic for the robot")
    ] = None,
    zenoh_connect: Annotated[
        str | None,
        typer.Option(help="Zenoh router to join as a client, e.g. tcp/127.0.0.1:7447"),
    ] = None,
) -> None:
    """Bridge the autopilot to SpiriSynq. Options override SPIRI_MAV_* settings."""
    from spiri_mav.main import get_settings, serve

    overrides = {
        "mavlink_url": url,
        "sysid": sysid,
        "target_sysid": target_sysid,
        "topic": topic,
        "zenoh_connect": zenoh_connect,
    }
    settings = get_settings().model_copy(
        update={k: v for k, v in overrides.items() if v is not None}
    )
    serve(settings)


if __name__ == "__main__":
    app()
