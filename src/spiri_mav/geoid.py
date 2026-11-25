"""The geoid's height above the WGS84 ellipsoid (EGM96), for a GPS that doesn't say.

MAVLink altitudes are above mean sea level; the pose tree is ECEF, so its
heights are above the ellipsoid. The two differ by the geoid separation, about
-21 m at Halifax. A real GPS reports it (``GPS_RAW_INT.alt_ellipsoid``);
ArduPilot SITL doesn't, and without this its robots' heights would be sea-level
heights posing as ellipsoid heights: right against a sea-level map only by
accident, and wrong against everything else (an ATAK phone's ``hae``).

The grid is EGM96 at 15', 721 rows from 90°N down to 90°S by 1440 columns
eastward from 0°, as big-endian int16 centimetres: the same data, and the same
bilinear interpolation, as the debug map's ``egm96-universal``, so the two
agree to the centimetre. It ships in the package: nothing is fetched.
"""

from __future__ import annotations

import math
from array import array
from functools import cache
from importlib.resources import files

ROWS, COLS = 721, 1440
STEP = 0.25  # degrees


@cache
def _grid() -> array:
    data = array("h")
    data.frombytes(files(__package__).joinpath("egm96-15.bin").read_bytes())
    if len(data) != ROWS * COLS:
        raise ValueError(f"egm96-15.bin holds {len(data)} values, not {ROWS * COLS}")
    if array("h", b"\x00\x01")[0] != 1:  # the file is big-endian; this machine isn't
        data.byteswap()
    return data


def geoid_height(lat: float, lon: float) -> float:
    """The geoid's height above the WGS84 ellipsoid at *lat*/*lon* (degrees), in metres.

    An ellipsoid height is a sea-level height plus this.
    """
    if not (math.isfinite(lat) and math.isfinite(lon)) or abs(lat) > 90:
        raise ValueError(f"no geoid height at {lat}, {lon}")
    grid = _grid()
    y = (90.0 - lat) / STEP
    top = min(int(y), ROWS - 2)
    x = (lon % 360.0) / STEP
    left = int(x) % COLS
    right = (left + 1) % COLS
    fy, fx = y - top, x - int(x)

    def at(row: int, col: int) -> float:
        return grid[row * COLS + col] / 100

    upper = at(top, left) + (at(top, right) - at(top, left)) * fx
    lower = at(top + 1, left) + (at(top + 1, right) - at(top + 1, left)) * fx
    return upper + (lower - upper) * fy
