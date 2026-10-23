# SpiriMav

MAVLink vehicle bridge for SpiriSynq: a companion-computer GCS that mirrors autopilot state

## Getting Started

### Installation

```console
# Install dependencies
uv sync

# Generate lock file
uv lock
```

### Run

```console
SpiriMav --help
```

### Run as module

```console
python -m spiri_mav
```

## Development

```console
# Run tests
python -m pytest
python -m pytest --cov=spiri_mav

# Run type checking
python -m mypy src/ 2>/dev/null
```
## Documentation

```console
# Install docs dependencies
uv sync
python -m pip install -r docs/requirements.txt

# Build HTML docs
sphinx-build -b html docs docs/_build

# View docs locally
python -m http.server --directory docs/_build 8888
```


## License
Licensed under the MIT License.

