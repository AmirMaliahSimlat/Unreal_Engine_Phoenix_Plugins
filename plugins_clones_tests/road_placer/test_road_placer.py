"""Unit tests for Road Placer logic clone (no Unreal)."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import shapefile
from shapely.geometry import Polygon

from plugins_clones_tests.road_placer.clone import (
    effective_max_edge_meters,
    format_report,
    resample_altitude_along_chainage,
    resample_altitude_along_neighbor_chains,
    simulate_road_tin,
    validate_epsg4326_prj,
)
from plugins_clones_tests.road_placer.constants import (
    DEFAULT_ALTITUDE_SAMPLE_METERS,
    DEFAULT_MAX_EDGE_OLD_SHRED,
    MAX_EDGE_IGNORE_BELOW_METERS,
    METERS_PER_LAT_DEG,
)


def _lonlat_offset(lon0: float, lat0: float, east_m: float, north_m: float) -> tuple[float, float]:
    mlon = 111320.0 * max(math.cos(math.radians(lat0)), 0.05)
    return lon0 + east_m / mlon, lat0 + north_m / METERS_PER_LAT_DEG


def _write_polygon_shp(path: Path, ring_lonlat: list[tuple[float, float]], prj: str) -> None:
    w = shapefile.Writer(str(path), shapeType=shapefile.POLYGON)
    w.field("id", "N")
    w.poly([ring_lonlat])
    w.record(1)
    w.close()
    path.with_suffix(".prj").write_text(prj, encoding="utf-8")


def _write_pointz_shp(path: Path, lonlat: np.ndarray, z: np.ndarray, prj: str) -> None:
    w = shapefile.Writer(str(path), shapeType=shapefile.POINTZ)
    w.field("id", "N")
    for i, ((x, y), zz) in enumerate(zip(lonlat, z)):
        w.pointz(float(x), float(y), float(zz))
        w.record(i)
    w.close()
    path.with_suffix(".prj").write_text(prj, encoding="utf-8")


WGS84 = 'GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137.0,298.257223563]],PRIMEM["Greenwich",0.0],UNIT["Degree",0.0174532925199433]]'
UTM14 = 'PROJCS["NAD_1983_UTM_Zone_14N",GEOGCS["GCS_North_American_1983",DATUM["D_North_American_1983",SPHEROID["GRS_1980",6378137.0,298.257222101]],PRIMEM["Greenwich",0.0],UNIT["Degree",0.0174532925199433]],PROJECTION["Transverse_Mercator"],PARAMETER["False_Easting",500000.0],PARAMETER["False_Northing",0.0],PARAMETER["Central_Meridian",-99.0],PARAMETER["Scale_Factor",0.9996],PARAMETER["Latitude_Of_Origin",0.0],UNIT["Meter",1.0]]'


def _stub_road(tmp_path: Path) -> tuple[Path, Path]:
    """8 m wide, 40 m long rectangle with 0.3 m outline PointZ."""
    lon0, lat0 = -96.77, 39.075
    length, width, step = 40.0, 8.0, 0.3
    corners = [
        _lonlat_offset(lon0, lat0, 0.0, 0.0),
        _lonlat_offset(lon0, lat0, length, 0.0),
        _lonlat_offset(lon0, lat0, length, width),
        _lonlat_offset(lon0, lat0, 0.0, width),
        _lonlat_offset(lon0, lat0, 0.0, 0.0),
    ]
    mask = tmp_path / "mask.shp"
    _write_polygon_shp(mask, corners, WGS84)

    pts = []
    zs = []
    # Walk the four edges in meters.
    edges = [
        ((0.0, 0.0), (length, 0.0)),
        ((length, 0.0), (length, width)),
        ((length, width), (0.0, width)),
        ((0.0, width), (0.0, 0.0)),
    ]
    for (x0, y0), (x1, y1) in edges:
        dist = math.hypot(x1 - x0, y1 - y0)
        n = max(int(dist / step), 1)
        for i in range(n):
            t = i / n
            e = x0 + t * (x1 - x0)
            nrt = y0 + t * (y1 - y0)
            lon, lat = _lonlat_offset(lon0, lat0, e, nrt)
            pts.append((lon, lat))
            zs.append(350.0 + 0.02 * e)
    points = tmp_path / "points.shp"
    _write_pointz_shp(points, np.asarray(pts), np.asarray(zs), WGS84)
    return mask, points


def test_utm_prj_is_rejected():
    ok, msg = validate_epsg4326_prj(UTM14)
    assert ok is False
    assert "projected" in msg.lower()


def test_wgs84_prj_is_accepted():
    ok, _ = validate_epsg4326_prj(WGS84)
    assert ok is True


def test_plugin_ignores_short_max_edge():
    assert effective_max_edge_meters(DEFAULT_MAX_EDGE_OLD_SHRED) == 0.0
    assert effective_max_edge_meters(40.0) == 0.0
    assert effective_max_edge_meters(0.0) == 0.0
    assert effective_max_edge_meters(MAX_EDGE_IGNORE_BELOW_METERS) == MAX_EDGE_IGNORE_BELOW_METERS
    assert effective_max_edge_meters(150.0) == 150.0


def test_raw_3_5m_cap_shreds_8m_road(tmp_path: Path):
    mask, points = _stub_road(tmp_path)
    shredded = simulate_road_tin(
        mask,
        points,
        max_edge_meters=3.5,
        include_mask_vertices=False,
        apply_plugin_max_edge_clamp=False,
    )
    filled = simulate_road_tin(
        mask,
        points,
        max_edge_meters=0.0,
        include_mask_vertices=False,
        apply_plugin_max_edge_clamp=True,
    )
    assert shredded.coverage_of_mask < 0.2
    assert filled.coverage_of_mask > 0.85
    assert filled.ok


def test_plugin_clamp_saves_8m_road(tmp_path: Path):
    mask, points = _stub_road(tmp_path)
    report = simulate_road_tin(
        mask,
        points,
        max_edge_meters=3.5,
        include_mask_vertices=False,
        apply_plugin_max_edge_clamp=True,
    )
    assert report.effective_max_edge_m == 0.0
    assert report.coverage_of_mask > 0.85
    assert "shredded" not in format_report(report).lower() or report.ok


def test_zero_altitude_spacing_keeps_raw_z():
    s = np.arange(0.0, 21.0, 1.0)
    z = 100.0 + np.sin(s)
    out = resample_altitude_along_chainage(s, z, DEFAULT_ALTITUDE_SAMPLE_METERS)
    np.testing.assert_allclose(out, z)


def test_altitude_resample_smooths_dtm_noise_keeps_slope():
    s = np.arange(0.0, 101.0, 1.0)
    z_true = 100.0 + 0.05 * s
    z_noisy = z_true.copy()
    z_noisy[np.mod(s, 10.0) > 0.5] += 5.0
    out = resample_altitude_along_chainage(s, z_noisy, 10.0, closed=False, ring_length=100.0)
    np.testing.assert_allclose(out[0::10], z_true[0::10], atol=1e-6)
    assert abs(out[5] - 100.25) < 0.15
    assert abs(out[5] - z_noisy[5]) > 4.0
    assert abs(out[55] - 102.75) < 0.15


def test_altitude_resample_does_not_blend_opposite_curbs():
    lon0, lat0 = -96.77, 39.075
    length, width, step = 40.0, 8.0, 1.0
    pts = []
    zs = []
    south = []
    north = []
    for y, z_edge, bucket in ((0.0, 100.0, south), (width, 200.0, north)):
        n = max(int(round(length / step)), 1)
        for i in range(n + 1):
            e = i * step
            lon, lat = _lonlat_offset(lon0, lat0, e, y)
            pts.append((lon, lat))
            spike = 5.0 if i % 10 else 0.0
            zs.append(z_edge + spike)
            bucket.append(len(pts) - 1)
    lonlat = np.asarray(pts)
    z = np.asarray(zs)
    out = resample_altitude_along_neighbor_chains(lonlat, z, 10.0)
    assert np.mean(out[south]) < 115.0
    assert np.mean(out[north]) > 185.0
    assert abs(np.mean(out[south]) - np.mean(out[north])) > 70.0


def test_altitude_resample_keeps_along_curb_slope():
    """10 m Z controls on a 0.3 m walk must not collapse a 40 m grade to one height."""
    lon0, lat0 = -96.77, 39.075
    length, step = 400.0, 0.3
    pts = []
    zs = []
    n = int(round(length / step))
    for i in range(n + 1):
        e = i * step
        lon, lat = _lonlat_offset(lon0, lat0, e, 0.0)
        pts.append((lon, lat))
        zs.append(320.0 + 0.1 * e)  # 40 m rise over 400 m
    out = resample_altitude_along_neighbor_chains(np.asarray(pts), np.asarray(zs), 10.0)
    assert out.max() - out.min() > 35.0
    assert abs(out[0] - 320.0) < 0.2
    assert abs(out[-1] - 360.0) < 0.2
    assert abs(float(np.mean(out)) - 340.0) < 2.0
