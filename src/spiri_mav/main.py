"""Main entry point for SpiriMav."""

from __future__ import annotations

import sys
from pathlib import Path

from loguru import logger
from pydantic_settings import BaseSettings, SettingsConfigDict

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


def main() -> None:
    """Main entry point."""
    settings = get_settings()
    setup_logging(level="DEBUG" if settings.debug else "INFO")

    logger.info(
        "Starting {} v{} [environment: {}]",
        settings.app_name,
        settings.version,
        settings.environment,
    )

    # Add your main application logic here
    logger.info("{} is running", settings.app_name)


if __name__ == "__main__":
    main()
