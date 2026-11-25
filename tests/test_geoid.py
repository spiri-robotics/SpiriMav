"""EGM96 from the shipped grid: NGA's check points, and the debug map's agreement."""

import pytest

from spiri_mav.geoid import geoid_height


@pytest.mark.parametrize(
    ("lat", "lon", "n"),
    [
        # NGA's EGM96 test values, from the spherical harmonics: bilinear on the
        # 15' grid is within a few centimetres of them, up to ~6 cm.
        (38.6281550, 269.7791550, -31.628),
        (-14.6212170, 305.0211140, -2.969),
        (46.8743190, 102.4487290, -43.575),
        (-23.6174460, 133.8747120, 15.871),
        (38.6254730, 359.9995000, 50.066),
        (-0.4667440, 0.0023000, 17.329),
    ],
)
def test_nga_check_points(lat, lon, n):
    assert geoid_height(lat, lon) == pytest.approx(n, abs=0.1)


def test_longitude_wraps():
    assert geoid_height(44.66, -63.56) == pytest.approx(
        geoid_height(44.66, 296.44), abs=1e-9
    )


def test_halifax_is_about_21_m_below_the_ellipsoid():
    assert -22.5 < geoid_height(44.66, -63.56) < -19.5


def test_the_poles():
    assert geoid_height(90.0, 0.0) == pytest.approx(13.61, abs=0.05)
    assert geoid_height(-90.0, 0.0) == pytest.approx(-29.53, abs=0.05)


def test_nonsense_is_refused():
    with pytest.raises(ValueError):
        geoid_height(91.0, 0.0)
