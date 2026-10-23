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
def run() -> None:
    """Run SpiriMav."""
    logger.info("Running SpiriMav...")


if __name__ == "__main__":
    app()
