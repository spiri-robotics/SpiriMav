# Getting Started

This guide walks you through installing, configuring, and running SpiriMav.

## Installation

Add SpiriMav to your project:

```console
uv add SpiriMav
```

Or install from source:

```console
uv sync
uv lock
```

## Configuration

SpiriMav runs with sensible defaults. If you need to customize, set environment variables with the `SPIRI_MAV_` prefix:

```console
export SPIRI_MAV_ENVIRONMENT=production
export SPIRI_MAV_LOG_LEVEL=DEBUG
```

Or create a `.env` file in your project root. See the {ref}`api-reference` for all available options.

## Quick Start

Run the CLI:

```console
SpiriMav --help
```

Or run as a module:

```console
python -m spiri_mav
```

For verbose output:

```console
SpiriMav run --verbose
```

## Next Steps

- Read the {ref}`api-reference` for programmatic details
- Check the [pyproject.toml](https://github.com/spiri-robotics/spiri-app-template/blob/main/pyproject.toml) for build and dependency configuration
- See the [Docker compose files](https://github.com/spiri-robotics/spiri-app-template/blob/main/docker/) for deployment options

