"""
Road Placer geometry clone (no Unreal).

Mirrors CRS checks, outline-point keep rules, max-edge ignore,
along-curb altitude resampling, and centroid-in-mask TIN filtering
from Road Placer 1.9.0.

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
    ALTITUDE_SNAP_METERS,
    MAX_EDGE_IGNORE_BELOW_METERS,
    METERS_PER_LAT_DEG,
    OUTLINE_SNAP_METERS,
    QUANTIZE_DEG,
    ZERO_Z_EPS,
)


def meters_per_lon_deg(lat_deg: float) -> float:
    return 111320.0 * max(math.cos(math.radians(lat_deg)), 0.05)


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


def catmull_rom(t: float, p0: float, p1: float, p2: float, p3: float) -> float:
    t2 = t * t
    t3 = t2 * t
    return 0.5 * (
        (2.0 * p1)
        + (-p0 + p2) * t
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
    )


def pick_chainage_anchors(s: np.ndarray, spacing_m: float, *, closed: bool, ring_length: float) -> np.ndarray:
    """Indices of Z control samples ~every spacing_m along sorted chainage."""
    n = int(len(s))
    if n == 0:
        return np.zeros(0, dtype=np.int64)
    anchors = [0]
    last_s = float(s[0])
    for i in range(1, n):
        if float(s[i]) - last_s >= spacing_m - 1.0e-6:
            anchors.append(i)
            last_s = float(s[i])
    if anchors[-1] != n - 1:
        wrap = (float(s[0]) + ring_length - float(s[-1])) if closed else math.inf
        if (not closed) or wrap >= spacing_m * 0.5:
            anchors.append(n - 1)
    return np.asarray(anchors, dtype=np.int64)


def eval_altitude_at_s(
    s_query: float,
    anchor_s: np.ndarray,
    anchor_z: np.ndarray,
    *,
    closed: bool,
    ring_length: float,
) -> float:
    n = int(len(anchor_z))
    if n == 0:
        return 0.0
    if n == 1:
        return float(anchor_z[0])
    if n == 2:
        if not closed:
            den = max(float(anchor_s[1] - anchor_s[0]), 1.0e-9)
            t = min(max((s_query - float(anchor_s[0])) / den, 0.0), 1.0)
            return float(anchor_z[0] + t * (anchor_z[1] - anchor_z[0]))
        fwd = max(float(anchor_s[1] - anchor_s[0]), 1.0e-9)
        if s_query + 1.0e-9 >= float(anchor_s[0]) and s_query <= float(anchor_s[1]) + 1.0e-9:
            t = min(max((s_query - float(anchor_s[0])) / fwd, 0.0), 1.0)
            return float(anchor_z[0] + t * (anchor_z[1] - anchor_z[0]))
        back = max(ring_length - float(anchor_s[1] - anchor_s[0]), 1.0e-9)
        q = s_query + ring_length if s_query + 1.0e-9 < float(anchor_s[0]) else s_query
        t = min(max((q - float(anchor_s[1])) / back, 0.0), 1.0)
        return float(anchor_z[1] + t * (anchor_z[0] - anchor_z[1]))

    seg = 0
    t = 0.0
    found = False
    for i in range(n):
        a = float(anchor_s[i])
        b = float(anchor_s[i + 1]) if i + 1 < n else (float(anchor_s[0]) + ring_length if closed else float(anchor_s[-1]))
        if (not closed) and i + 1 >= n:
            break
        q_adj = s_query
        if closed and i + 1 >= n and s_query + 1.0e-9 < float(anchor_s[0]):
            q_adj = s_query + ring_length
        if q_adj + 1.0e-9 >= a and q_adj <= b + 1.0e-9:
            seg = i
            t = min(max((q_adj - a) / max(b - a, 1.0e-9), 0.0), 1.0)
            found = True
            break
    if not found:
        if not closed:
            return float(anchor_z[0] if s_query <= float(anchor_s[0]) else anchor_z[-1])
        return float(anchor_z[0])

    if closed:
        p0 = float(anchor_z[(seg - 1) % n])
        p1 = float(anchor_z[seg])
        p2 = float(anchor_z[(seg + 1) % n])
        p3 = float(anchor_z[(seg + 2) % n])
    else:
        p1 = float(anchor_z[seg])
        p2 = float(anchor_z[min(seg + 1, n - 1)])
        p0 = float(anchor_z[seg - 1]) if seg > 0 else p1
        p3 = float(anchor_z[seg + 2]) if seg + 2 < n else p2
    return catmull_rom(t, p0, p1, p2, p3)


def resample_altitude_along_chainage(
    s: np.ndarray,
    z: np.ndarray,
    spacing_m: float,
    *,
    closed: bool = False,
    ring_length: float | None = None,
) -> np.ndarray:
    """Keep XY chainage; take Z every spacing_m and cubically fill the rest."""
    if spacing_m <= 1.0e-6 or len(s) == 0:
        return np.asarray(z, dtype=np.float64).copy()
    order = np.argsort(s, kind="stable")
    s_sorted = np.asarray(s, dtype=np.float64)[order]
    z_sorted = np.asarray(z, dtype=np.float64)[order]
    unique = [0]
    for i in range(1, len(s_sorted)):
        if s_sorted[i] - s_sorted[unique[-1]] > 0.05:
            unique.append(i)
    su = s_sorted[unique]
    zu = z_sorted[unique]
    length = float(ring_length) if ring_length is not None else (float(su[-1] - su[0]) if len(su) else 0.0)
    if closed and length <= 1.0:
        length = float(su[-1] - su[0]) + 1.0
    anchors = pick_chainage_anchors(su, spacing_m, closed=closed, ring_length=length)
    out = np.asarray(z, dtype=np.float64).copy()
    for i, si in enumerate(s_sorted):
        out[order[i]] = eval_altitude_at_s(
            float(si),
            su[anchors],
            zu[anchors],
            closed=closed,
            ring_length=length,
        )
    return out


def _project_point_to_ring(
    lon: float,
    lat: float,
    ring_lonlat: np.ndarray,
) -> tuple[float, float, float]:
    """Return (dist_m, chainage_s, ring_length)."""
    pts = np.asarray(ring_lonlat, dtype=np.float64)
    if len(pts) >= 2 and np.allclose(pts[0], pts[-1], atol=1.0e-12):
        pts = pts[:-1]
    n = len(pts)
    if n < 2:
        return math.inf, 0.0, 0.0
    best_d = math.inf
    best_s = 0.0
    s_acc = 0.0
    for i in range(n):
        a = pts[i]
        b = pts[(i + 1) % n]
        mlon = meters_per_lon_deg(0.5 * (a[1] + b[1]))
        bx = (b[0] - a[0]) * mlon
        by = (b[1] - a[1]) * METERS_PER_LAT_DEG
        len_m = math.hypot(bx, by)
        if len_m < 1.0e-4:
            continue
        px = (lon - a[0]) * mlon
        py = (lat - a[1]) * METERS_PER_LAT_DEG
        t = (px * bx + py * by) / (len_m * len_m)
        t = min(max(t, 0.0), 1.0)
        dx = px - t * bx
        dy = py - t * by
        dist = math.hypot(dx, dy)
        if dist < best_d:
            best_d = dist
            best_s = s_acc + t * len_m
        s_acc += len_m
    return best_d, best_s, s_acc


def resample_altitude_along_rings(
    lonlat: np.ndarray,
    heights: np.ndarray,
    rings: list[np.ndarray],
    spacing_m: float,
    snap_max_m: float = ALTITUDE_SNAP_METERS,
) -> np.ndarray:
    """Snap samples to mask rings, then resample Z along each curb separately."""
    out = np.asarray(heights, dtype=np.float64).copy()
    if spacing_m <= 1.0e-6 or len(lonlat) == 0 or not rings:
        return out
    grouped: dict[int, list[int]] = {}
    chainage: dict[int, float] = {}
    lengths: dict[int, float] = {}
    for i, (lon, lat) in enumerate(np.asarray(lonlat, dtype=np.float64)):
        best_d = math.inf
        best_ring = -1
        best_s = 0.0
        best_len = 0.0
        for ri, ring in enumerate(rings):
            dist, s, length = _project_point_to_ring(float(lon), float(lat), ring)
            if dist < best_d:
                best_d = dist
                best_ring = ri
                best_s = s
                best_len = length
        if best_ring < 0 or best_d > snap_max_m:
            continue
        grouped.setdefault(best_ring, []).append(i)
        chainage[i] = best_s
        lengths[best_ring] = best_len
    for ri, idxs in grouped.items():
        s = np.asarray([chainage[i] for i in idxs], dtype=np.float64)
        z = out[np.asarray(idxs, dtype=np.int64)]
        closed = lengths.get(ri, 0.0) > 1.0
        z_new = resample_altitude_along_chainage(
            s, z, spacing_m, closed=closed, ring_length=lengths.get(ri, 0.0)
        )
        out[np.asarray(idxs, dtype=np.int64)] = z_new
    return out


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
) -> RoadLogicReport:
    """
    Replay keep-rules + TIN filters. Does not spawn meshes.

    apply_plugin_max_edge_clamp=True uses the 1.3.0 ignore-below-100 rule.
    Set False to see what a raw cap (e.g. 3.5 m) would do to the pavement.
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
    report.points_kept = int(keep.sum())
    if report.points_kept < 3:
        report.errors.append("Fewer than 3 outline samples on or near the mask.")
        return report
    if report.points_strict_inside / max(report.points_unique, 1) < 0.5:
        report.warnings.append(
            f"Only {100.0 * report.points_strict_inside / report.points_unique:.0f}% of points "
            "are strictly inside the mask (expected for ring samples)."
        )

    kept_ll = lonlat_u[keep]
    kept_z = z_u[keep]
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
        f"kept={report.points_kept:,} strict_inside={report.points_strict_inside:,}",
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
