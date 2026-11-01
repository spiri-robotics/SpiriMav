"""Main entry point for SpiriMav."""

from __future__ import annotations

import json
import sys
import threading

import zenoh
from loguru import logger
from pydantic_settings import BaseSettings, SettingsConfigDict
from SpiriSynq.link import Link
from SpiriSynq.session import Session, current_session

from spiri_mav import __version__


class Settings(BaseSettings):
    """Settings for SpiriMav, loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_prefix="SPIRI_MAV_",
        env_file=".env",
        extra="ignore",
    )

    app_name: str = "SpiriMav"
    version: str = __version__
    environment: str = "development"
    debug: bool = False

    mavlink_url: str = (
        "tcp:127.0.0.1:5760"  # pymavlink connection string; ArduPilot SITL's default
    )
    sysid: int = 254  # ours: must not be the FCU's GCS sysid (docs/design.md)
    target_sysid: int = 1  # the autopilot's
    topic: str = "robot"  # SpiriSynq topic, under this host's base topic
    zenoh_connect: str | None = (
        None  # e.g. tcp/127.0.0.1:7447: join via that router as a client
    )
    # Which SpiriCamera cameras each gimbal carries, by gimbal device id: MAVLink
    # doesn't say. JSON in the environment, e.g. {"1": ["mu-1/camera_eo"]}.
    gimbal_cameras: dict[int, list[str]] = {}
    # Which DISTANCE_SENSOR id is each gimbal's rangefinder, by gimbal device
    # id: ArduPilot reports one as fixed to the airframe. E.g. {"1": 0}.
    gimbal_rangefinders: dict[int, int] = {}
    # What MAVLink doesn't report, as 90% bounds; estimates until a datasheet
    # says otherwise. Pointing: where the gimbal thinks it points, off its
    # axis, mostly the vehicle's compass heading (~2° across). Rangefinder: a
    # ZT30-class laser's ±1 m, plus 0.25% of the distance past 100 m, taken
    # here as from the start.
    gimbal_pointing_accuracy: float = 1.0  # ± deg
    rangefinder_accuracy: float = 1.0  # ± m
    rangefinder_accuracy_per_m: float = 0.0025  # ± m per m of distance


def setup_logging(level: str = "INFO") -> None:
    """Configure loguru logging."""
    logger.remove()
    logger.add(
        sys.stderr,
        level=level,
        format="{time:YYYY-MM-DD HH:mm:ss.SSS} | {level: <8} | {name}:{function}:{line} - {message}",
    )


def get_settings() -> Settings:
    """Load settings from environment."""
    return Settings()


def serve(settings: Settings, stop: threading.Event | None = None) -> None:
    """Bridge the autopilot at ``settings.mavlink_url`` to SpiriSynq until *stop* or Ctrl-C."""
    from spiri_mav.mav_robot import MavRobot

    logger.info(
        "Starting {} v{} [environment: {}]",
        settings.app_name,
        settings.version,
        settings.environment,
    )
    if settings.zenoh_connect:
        config = zenoh.Config()
        config.insert_json5("mode", '"client"')
        config.insert_json5("connect/endpoints", json.dumps([settings.zenoh_connect]))
        current_session.set(Session(config=config))
    robot = MavRobot(
        settings.topic,
        synq_authoritive=True,
        sysid=settings.sysid,
        target_sysid=settings.target_sysid,
        _gimbal_cameras={
            device_id: tuple(Link(topic) for topic in topics)
            for device_id, topics in settings.gimbal_cameras.items()
        },
        _gimbal_rangefinders=settings.gimbal_rangefinders,
        _pointing_accuracy=settings.gimbal_pointing_accuracy,
        _range_accuracy=(
            settings.rangefinder_accuracy,
            settings.rangefinder_accuracy_per_m,
        ),
    )
    logger.info("Publishing robot at {}", robot.synq_absolute_path)
    try:
        robot.run(settings.mavlink_url, stop=stop)
    except KeyboardInterrupt:
        logger.info("Stopping")
    finally:
        robot.close()


def main() -> None:
    """Main entry point."""
    settings = get_settings()
    setup_logging(level="DEBUG" if settings.debug else "INFO")
    serve(settings)


if __name__ == "__main__":
    main()
