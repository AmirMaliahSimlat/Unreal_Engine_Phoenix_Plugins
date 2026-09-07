"""Run the Road Placer logic clone on the Fort Riley shapefiles (no Unreal)."""

from __future__ import annotations

import argparse
from pathlib import Path

from plugins_clones_tests.road_placer.clone import format_report, simulate_road_tin, validate_shapefile_prj

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MASK = ROOT / "Data" / "FortRiley" / "Roads" / "Mask" / "FortRiley_r61440_c81920_mask_4326.shp"
DEFAULT_POINTS = ROOT / "Data" / "FortRiley" / "Roads" / "Outline Points" / "Roads_Elevation_Points.shp"
UTM_MASK = ROOT / "Data" / "FortRiley" / "Roads" / "Mask" / "FortRiley_r61440_c81920_union_v3.shp"


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay Road Placer filters on shapefiles.")
    parser.add_argument("--mask", type=Path, default=DEFAULT_MASK)
    parser.add_argument("--points", type=Path, default=DEFAULT_POINTS)
    args = parser.parse_args()

    print("=== Road Placer clone (Fort Riley inputs) ===")
    print(f"mask:   {args.mask}")
    print(f"points: {args.points}")

    if UTM_MASK.exists():
        ok, msg = validate_shapefile_prj(UTM_MASK)
        print(f"\nUTM mask sibling rejected={not ok}: {msg}")

    print("\n--- 1.3.0 plugin clamp, MaxEdge=3.5 (what Unreal 1.3.0 does) ---")
    plugin = simulate_road_tin(args.mask, args.points, max_edge_meters=3.5, apply_plugin_max_edge_clamp=True)
    print(format_report(plugin))

    print("\n--- Raw MaxEdge=3.5 with clamp OFF (the bug you saw) ---")
    raw = simulate_road_tin(
        args.mask,
        args.points,
        max_edge_meters=3.5,
        include_mask_vertices=False,
        apply_plugin_max_edge_clamp=False,
    )
    print(format_report(raw))

    print("\n--- 1.3.0 unlimited, points only (no extra mask vertices) ---")
    pts_only = simulate_road_tin(
        args.mask,
        args.points,
        max_edge_meters=0.0,
        include_mask_vertices=False,
        apply_plugin_max_edge_clamp=True,
    )
    print(format_report(pts_only))

    issues = []
    if plugin.errors:
        issues.extend(plugin.errors)
    if raw.coverage_of_mask < 0.05:
        issues.append(
            f"CONFIRMED: raw 3.5 m cap covers {100*raw.coverage_of_mask:.1f}% of the mask "
            "(matches the tiny floating scraps)."
        )
    if plugin.coverage_of_mask < 0.85:
        issues.append(f"1.3.0 still only covers {100*plugin.coverage_of_mask:.1f}% of the mask.")
    if plugin.longest_edge_max_m > 40.0:
        issues.append(
            f"Some in-mask triangles span {plugin.longest_edge_max_m:.1f} m "
            "(wide lots). A 40 m cap would still punch holes."
        )
    if plugin.points_strict_inside < 0.5 * plugin.points_unique:
        issues.append(
            "Most PointZ sit on the ring, not inside. Strict PIP-only keep would drop them; "
            "15 m snap is required."
        )
    if plugin.zero_z:
        issues.append(f"{plugin.zero_z} points have Z≈0.")
    if plugin.z_min < 200 or plugin.z_max > 800:
        issues.append(
            f"Z range {plugin.z_min:.1f}–{plugin.z_max:.1f} m looks unusual for Fort Riley ellipsoid height."
        )

    print("\n=== Issues ===")
    if issues:
        for i, line in enumerate(issues, 1):
            print(f"{i}. {line}")
    else:
        print("None beyond expected outline-PIP behavior.")
    return 0 if plugin.ok and plugin.coverage_of_mask >= 0.85 else 1


if __name__ == "__main__":
    raise SystemExit(main())
