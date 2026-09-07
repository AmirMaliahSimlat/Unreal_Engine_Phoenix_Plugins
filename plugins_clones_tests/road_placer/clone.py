"""
Road Placer geometry clone (no Unreal).

Mirrors CRS checks, outline-point keep rules, sagging-interior drop,
max-edge ignore, and centroid-in-mask TIN filtering from Road Placer 1.5.0.

Delaunay uses scipy/Qhull, not the plugin Bowyer-Watson. Use this to
validate thresholds and keep/drop counts, not triangle identity.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from shapely import contains_xy
from shapely.geometry import Polygon, shape
from shapely.ops import unary_union

from .constants import (
    INTERIOR_PROUD_METERS,
    MAX_EDGE_IGNORE_BELOW_METERS,
    METERS_PER_LAT_DEG,
    OUTLINE_BAND_METERS,
    OUTLINE_SNAP_METERS,
    QUANTIZE_DEG,
    ZERO_Z_EPS,
)


def meters_per_lon_deg(lat_deg: float) -> float:
    return 111320.0 * max(math.cos(math.radians(lat_deg)), 0.05)


def keep_proud_interior(
    outline_lonlat: np.ndarray,
    outline_z: np.ndarray,
    interior_lonlat: np.ndarray,
    interior_z: np.ndarray,
    proud_m: float = INTERIOR_PROUD_METERS,
) -> tuple[np.ndarray, np.ndarray, int, int]:
    """Keep interior samples at or above the interpolated curb plane + proud_m.

    Mirrors DropSaggingInteriorSamples: inverse-distance of the 6 nearest
    outline heights. Drops width-wise bowls (high-low-high); keeps crowns.
    """
    if interior_lonlat.shape[0] == 0:
        empty_ll = interior_lonlat.reshape(0, 2) if interior_lonlat.ndim == 1 else interior_lonlat
        empty_z = interior_z.reshape(0) if interior_z.ndim == 0 else interior_z
        return empty_ll, empty_z, 0, 0
    if outline_lonlat.shape[0] == 0:
        n = int(interior_lonlat.shape[0])
        return interior_lonlat, interior_z, n, 0

    lat0 = float(np.mean(outline_lonlat[:, 1]))
    mlon = meters_per_lon_deg(lat0)
    origin = outline_lonlat[0]

    def to_xy(ll: np.ndarray) -> np.ndarray:
        xy = np.empty_like(ll)
        xy[:, 0] = (ll[:, 0] - origin[0]) * mlon
        xy[:, 1] = (ll[:, 1] - origin[1]) * METERS_PER_LAT_DEG
        return xy

    from scipy.spatial import cKDTree

    tree = cKDTree(to_xy(outline_lonlat))
    k = min(6, int(outline_lonlat.shape[0]))
    dists, idx = tree.query(to_xy(interior_lonlat), k=k)
    if k == 1:
        dists = np.asarray(dists)[:, None]
        idx = np.asarray(idx)[:, None]
    weights = 1.0 / (np.square(dists) + 0.01)
    curb_z = np.sum(weights * outline_z[idx], axis=1) / np.sum(weights, axis=1)
    keep = interior_z + 1.0e-6 >= curb_z + proud_m
    nkeep = int(np.count_nonzero(keep))
    nskip = int(np.count_nonzero(~keep))
    return interior_lonlat[keep], interior_z[keep], nkeep, nskip


def effective_max_edge_meters(max_edge_meters: float) -> float:
    """Mirrors PlaceRoadsFromShapefiles max-edge clamp in 1.3.0."""
    if max_edge_meters < 0.0:
        return 0.0
    if 0.0 < max_edge_meters < MAX_EDGE_IGNORE_BELOW_METERS:
        return 0.0
    return float(max_edge_meters)


def validate_epsg4326_prj(prj_text: str | None, missing_ok: bool = True) -> tuple[bool, str]:
    """Mirrors RoadShapefileReader::ValidateEpsg4326Prj."""
    if prj_text is None or not prj_text.strip():
        if missing_ok:
            return True, "No .prj; assuming EPSG:4326."
        return False, "Missing .prj."
    upper = prj_text.upper()
    looks_4326 = (
        'EPSG","4326' in upper
        or 'AUTHORITY["EPSG","4326"]' in upper
        or "WGS_1984" in upper
        or "WGS 84" in upper
        or "GCS_WGS_1984" in upper
        or "WGS84" in upper
    )
    if looks_4326:
        return True, "EPSG:4326 / WGS84."
    if "PROJCS[" in upper or 'PROJCS "' in upper:
        return False, "Shapefile .prj is a projected CRS, not EPSG:4326."
    return True, "Unrecognized .prj; treating as lon/lat degrees."


def validate_shapefile_prj(shp_path: Path) -> tuple[bool, str]:
    prj = Path(shp_path).with_suffix(".prj")
    text = prj.read_text(encoding="utf-8", errors="replace") if prj.exists() else None
    return validate_epsg4326_prj(text)


@dataclass
class RoadLogicReport:
    mask_polygons: int = 0
    mask_holes: int = 0
    mask_ring_vertices: int = 0
    mask_area_m2: float = 0.0
    points_read: int = 0
    points_unique: int = 0
    points_kept: int = 0
    points_strict_inside: int = 0
    interior_kept: int = 0
    interior_skipped: int = 0
    zero_z: int = 0
    z_min: float = 0.0
    z_max: float = 0.0
    z_median: float = 0.0
    nn_spacing_p50_m: float = 0.0
    delaunay_triangles: int = 0
    in_mask_triangles: int = 0
    kept_triangles: int = 0
    in_mask_area_m2: float = 0.0
    kept_area_m2: float = 0.0
    longest_edge_p50_m: float = 0.0
    longest_edge_p90_m: float = 0.0
    longest_edge_max_m: float = 0.0
    effective_max_edge_m: float = 0.0
    raw_max_edge_m: float = 0.0
    coverage_of_mask: float = 0.0
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors and self.kept_triangles >= 1


def load_mask_polygons(shp_path: Path) -> list[Polygon]:
    import shapefile

    polys: list[Polygon] = []
    with shapefile.Reader(str(shp_path), encoding="utf-8") as reader:
        for sr in reader.iterShapeRecords():
            geom = shape(sr.shape.__geo_interface__)
            if geom.is_empty:
                continue
            if geom.geom_type == "Polygon":
                polys.append(geom)
            elif geom.geom_type == "MultiPolygon":
                polys.extend(g for g in geom.geoms if not g.is_empty)
    return [p for p in polys if p.exterior is not None and len(p.exterior.coords) >= 4]


def load_pointz(shp_path: Path) -> tuple[np.ndarray, np.ndarray]:
    import shapefile

    lonlat = []
    heights = []
    with shapefile.Reader(str(shp_path)) as reader:
        for s in reader.iterShapes():
            if not s.points:
                continue
            x, y = s.points[0]
            z = s.z[0] if getattr(s, "z", None) else 0.0
            lonlat.append((x, y))
            heights.append(z)
    return np.asarray(lonlat, dtype=np.float64), np.asarray(heights, dtype=np.float64)


def _mask_union(polys: list[Polygon]):
    union = unary_union(polys)
    return union


def _ring_vertex_count(polys: list[Polygon]) -> tuple[int, int]:
    verts = 0
    holes = 0
    for p in polys:
        verts += max(len(p.exterior.coords) - 1, 0)
        holes += len(p.interiors)
        for hole in p.interiors:
            verts += max(len(hole.coords) - 1, 0)
    return verts, holes


def _mask_vertices_lonlat(polys: list[Polygon]) -> np.ndarray:
    rows = []
    for p in polys:
        for ring in (p.exterior, *p.interiors):
            xy = np.asarray(ring.coords)[:-1]
            if len(xy):
                rows.append(xy)
    if not rows:
        return np.zeros((0, 2), dtype=np.float64)
    return np.vstack(rows)


def _unique_quant(lonlat: np.ndarray, z: np.ndarray) -> tuple[np.ndarray, np.ndarray, int]:
    keys = np.round(lonlat * QUANTIZE_DEG).astype(np.int64)
    packed = keys[:, 0] * 10_000_000_000 + keys[:, 1]
    _, idx = np.unique(packed, return_index=True)
    idx.sort()
    return lonlat[idx], z[idx], int(lonlat.shape[0] - idx.size)


def _to_xy(lonlat: np.ndarray) -> tuple[np.ndarray, float]:
    lat0 = float(np.mean(lonlat[:, 1]))
    mlon = meters_per_lon_deg(lat0)
    origin = lonlat.mean(axis=0)
    xy = np.empty_like(lonlat)
    xy[:, 0] = (lonlat[:, 0] - origin[0]) * mlon
    xy[:, 1] = (lonlat[:, 1] - origin[1]) * METERS_PER_LAT_DEG
    return xy, lat0


def _edge_len_m(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    mid_lat = 0.5 * (a[:, 1] + b[:, 1])
    mlon = 111320.0 * np.clip(np.cos(np.deg2rad(mid_lat)), 0.05, None)
    dx = (a[:, 0] - b[:, 0]) * mlon
    dy = (a[:, 1] - b[:, 1]) * METERS_PER_LAT_DEG
    return np.hypot(dx, dy)


def _area_m2(polys: list[Polygon], lat0: float) -> float:
    mlon = meters_per_lon_deg(lat0)
    return float(sum(p.area for p in polys) * mlon * METERS_PER_LAT_DEG)


def simulate_road_tin(
    mask_shp: Path,
    points_shp: Path,
    max_edge_meters: float = 0.0,
    outline_snap_meters: float = OUTLINE_SNAP_METERS,
    include_mask_vertices: bool = True,
    apply_plugin_max_edge_clamp: bool = True,
    interior_proud_meters: float = INTERIOR_PROUD_METERS,
) -> RoadLogicReport:
    """
    Replay keep-rules + TIN filters. Does not spawn meshes.

    apply_plugin_max_edge_clamp=True uses the 1.3.0 ignore-below-100 rule.
    Set False to see what a raw cap (e.g. 3.5 m) would do to the pavement.
    interior_proud_meters mirrors InteriorProudMeters (0 = drop any sag).
    """
    report = RoadLogicReport(raw_max_edge_m=float(max_edge_meters))
    ok_mask, msg_mask = validate_shapefile_prj(mask_shp)
    ok_pts, msg_pts = validate_shapefile_prj(points_shp)
    if not ok_mask:
        report.errors.append(f"Mask CRS: {msg_mask}")
        return report
    if not ok_pts:
        report.errors.append(f"Points CRS: {msg_pts}")
        return report

    polys = load_mask_polygons(mask_shp)
    if not polys:
        report.errors.append("No mask polygons.")
        return report
    lonlat, z = load_pointz(points_shp)
    if len(lonlat) < 3:
        report.errors.append("Fewer than 3 elevation points.")
        return report

    report.mask_polygons = len(polys)
    report.mask_ring_vertices, report.mask_holes = _ring_vertex_count(polys)
    report.points_read = int(len(lonlat))
    union = _mask_union(polys)
    lat0 = float(np.mean(lonlat[:, 1]))
    report.mask_area_m2 = _area_m2(polys, lat0)

    lonlat_u, z_u, n_dup = _unique_quant(lonlat, z)
    report.points_unique = int(len(lonlat_u))
    if n_dup:
        report.warnings.append(f"Dropped {n_dup} duplicate PointZ at 1e-7 deg.")

    report.z_min = float(z_u.min())
    report.z_max = float(z_u.max())
    report.z_median = float(np.median(z_u))
    report.zero_z = int(np.sum(np.abs(z_u) < ZERO_Z_EPS))
    if report.zero_z == report.points_unique:
        report.warnings.append("Every PointZ height is 0.")

    inside = contains_xy(union, lonlat_u[:, 0], lonlat_u[:, 1])
    report.points_strict_inside = int(inside.sum())
    mlon = meters_per_lon_deg(lat0)
    buf_deg = outline_snap_meters / min(mlon, METERS_PER_LAT_DEG)
    near = contains_xy(union.boundary.buffer(buf_deg), lonlat_u[:, 0], lonlat_u[:, 1])
    keep = inside | near
    if int(keep.sum()) < 3:
        report.points_kept = int(keep.sum())
        report.errors.append("Fewer than 3 outline samples on or near the mask.")
        return report
    if report.points_strict_inside / max(report.points_unique, 1) < 0.5:
        report.warnings.append(
            f"Only {100.0 * report.points_strict_inside / report.points_unique:.0f}% of points "
            "are strictly inside the mask (expected for ring samples)."
        )

    kept_ll = lonlat_u[keep]
    kept_z = z_u[keep]
    band_deg = OUTLINE_BAND_METERS / min(mlon, METERS_PER_LAT_DEG)
    on_curb = contains_xy(union.boundary.buffer(band_deg), kept_ll[:, 0], kept_ll[:, 1])
    inside_kept = contains_xy(union, kept_ll[:, 0], kept_ll[:, 1])
    interior = inside_kept & ~on_curb
    outline = ~interior
    if np.any(interior) and np.any(outline):
        i_ll, i_z, nkeep, nskip = keep_proud_interior(
            kept_ll[outline],
            kept_z[outline],
            kept_ll[interior],
            kept_z[interior],
            proud_m=interior_proud_meters,
        )
        report.interior_kept = nkeep
        report.interior_skipped = nskip
        kept_ll = np.vstack([kept_ll[outline], i_ll]) if nkeep else kept_ll[outline]
        kept_z = np.concatenate([kept_z[outline], i_z]) if nkeep else kept_z[outline]
    elif np.any(interior):
        report.interior_kept = int(np.count_nonzero(interior))
    report.points_kept = int(len(kept_ll))
    if include_mask_vertices:
        mv = _mask_vertices_lonlat(polys)
        if len(mv):
            from scipy.spatial import cKDTree

            combined = np.vstack([kept_ll, mv])
            xy_all, _ = _to_xy(combined)
            tree = cKDTree(xy_all[: len(kept_ll)])
            _, nn = tree.query(xy_all[len(kept_ll) :], k=1)
            mv_z = kept_z[nn]
            kept_ll = np.vstack([kept_ll, mv])
            kept_z = np.concatenate([kept_z, mv_z])
            kept_ll, kept_z, _ = _unique_quant(kept_ll, kept_z)

    xy, _ = _to_xy(kept_ll)
    from scipy.spatial import cKDTree, Delaunay

    if len(xy) >= 8:
        sample = min(8000, len(xy))
        rng = np.random.default_rng(0)
        take = rng.choice(len(xy), size=sample, replace=False)
        nn = cKDTree(xy).query(xy[take], k=2)[0][:, 1]
        report.nn_spacing_p50_m = float(np.percentile(nn, 50))

    tri = Delaunay(xy, qhull_options="QJ")
    simplices = tri.simplices
    report.delaunay_triangles = int(len(simplices))

    a, b, c = kept_ll[simplices[:, 0]], kept_ll[simplices[:, 1]], kept_ll[simplices[:, 2]]
    cent = (a + b + c) / 3.0
    longest = np.maximum(
        np.maximum(_edge_len_m(a, b), _edge_len_m(b, c)),
        _edge_len_m(c, a),
    )
    pa, pb, pc = xy[simplices[:, 0]], xy[simplices[:, 1]], xy[simplices[:, 2]]
    area = 0.5 * np.abs(
        (pb[:, 0] - pa[:, 0]) * (pc[:, 1] - pa[:, 1])
        - (pc[:, 0] - pa[:, 0]) * (pb[:, 1] - pa[:, 1])
    )
    in_mask = contains_xy(union, cent[:, 0], cent[:, 1])
    report.in_mask_triangles = int(in_mask.sum())
    report.in_mask_area_m2 = float(area[in_mask].sum())
    if report.in_mask_triangles:
        report.longest_edge_p50_m = float(np.percentile(longest[in_mask], 50))
        report.longest_edge_p90_m = float(np.percentile(longest[in_mask], 90))
        report.longest_edge_max_m = float(longest[in_mask].max())

    cap = effective_max_edge_meters(max_edge_meters) if apply_plugin_max_edge_clamp else float(max_edge_meters)
    report.effective_max_edge_m = cap
    if cap > 0.0:
        keep_t = in_mask & (longest <= cap)
    else:
        keep_t = in_mask
    report.kept_triangles = int(keep_t.sum())
    report.kept_area_m2 = float(area[keep_t].sum())
    report.coverage_of_mask = report.kept_area_m2 / max(report.mask_area_m2, 1.0)

    if report.kept_triangles < 1:
        report.errors.append("No triangles remained inside the road mask.")
    if report.coverage_of_mask < 0.5 and cap > 0.0:
        report.errors.append(
            f"Max-edge {cap:.1f} m keeps only {100.0 * report.coverage_of_mask:.1f}% of the mask "
            "(pavement shredded)."
        )
    elif report.coverage_of_mask < 0.85:
        report.warnings.append(
            f"Kept triangles cover only {100.0 * report.coverage_of_mask:.1f}% of the mask."
        )
    return report


def format_report(report: RoadLogicReport) -> str:
    lines = [
        f"mask polygons={report.mask_polygons} holes={report.mask_holes} "
        f"ring_verts={report.mask_ring_vertices} area={report.mask_area_m2/1e4:.2f} ha",
        f"points read={report.points_read:,} unique={report.points_unique:,} "
        f"kept={report.points_kept:,} strict_inside={report.points_strict_inside:,} "
        f"interior kept/skipped={report.interior_kept}/{report.interior_skipped}",
        f"Z min/median/max={report.z_min:.2f}/{report.z_median:.2f}/{report.z_max:.2f} "
        f"zero={report.zero_z}  nn_p50={report.nn_spacing_p50_m:.3f} m",
        f"TIN delaunay={report.delaunay_triangles:,} in_mask={report.in_mask_triangles:,} "
        f"kept={report.kept_triangles:,}",
        f"longest in-mask edge p50/p90/max="
        f"{report.longest_edge_p50_m:.2f}/{report.longest_edge_p90_m:.2f}/{report.longest_edge_max_m:.1f} m",
        f"raw_max_edge={report.raw_max_edge_m:.1f} effective={report.effective_max_edge_m:.1f} "
        f"coverage={100.0 * report.coverage_of_mask:.1f}% "
        f"({report.kept_area_m2/1e4:.2f} ha of {report.mask_area_m2/1e4:.2f} ha)",
    ]
    for w in report.warnings:
        lines.append(f"WARN: {w}")
    for e in report.errors:
        lines.append(f"ERROR: {e}")
    return "\n".join(lines)
