#!/usr/bin/env python3
"""Check the explicitly synthetic drive geometry and USD obstacle clearances.

This helper runs in its own process because ovstage/ovfmi and usd-core may
load different USD libraries. None of these values describes a measured room.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes/demo_movimiento_4ruedas.usda"
HOME = np.array([0.0, 0.0])
HOME_YAW = 0.0
AISLE_YAW = -1.5536882812104804
TRAVEL_M = 2.2
TRACK_M = 0.17225000262260437
WHEEL_RADIUS_M = 0.04867133125373781
FOOTPRINT_RADIUS_M = 0.25


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, help="Check all x/y poses in this CSV")
    args = parser.parse_args()
    stage = Usd.Stage.Open(str(SCENE))
    if not stage:
        raise RuntimeError(f"Could not open {SCENE}")
    for path in ("/World/jetauto", "/World/structure/Floor", "/World/gemelo_semantico"):
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"Missing synthetic scene prim: {path}")
    direction = np.array([math.cos(AISLE_YAW), math.sin(AISLE_YAW)])
    if args.trace:
        with args.trace.open(newline="", encoding="utf-8") as stream:
            path_xy = np.array([
                [float(row["x_m"]), float(row["y_m"])]
                for row in csv.DictReader(stream)
            ])
        if len(path_xy) < 2:
            raise RuntimeError("The trajectory has too few poses")
    else:
        fraction = np.linspace(0, 1, 100)
        path_xy = HOME[None, :] + fraction[:, None] * (TRAVEL_M * direction)[None, :]

    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    obstacle_distances = []
    for prim in stage.GetPrimAtPath("/World/gemelo_semantico").GetChildren():
        bounds = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
        minimum = np.asarray(bounds.GetMin(), dtype=float)[:2]
        maximum = np.asarray(bounds.GetMax(), dtype=float)[:2]
        delta = np.maximum(0, np.maximum(minimum - path_xy, path_xy - maximum))
        obstacle_distances.append((float(np.min(np.linalg.norm(delta, axis=1))), prim.GetName()))
    obstacle_distances.sort()
    nearest_distance, nearest_name = obstacle_distances[0]
    floor = bbox.ComputeWorldBound(stage.GetPrimAtPath("/World/structure/Floor")).ComputeAlignedRange()
    lo = np.asarray(floor.GetMin(), dtype=float)[:2]
    hi = np.asarray(floor.GetMax(), dtype=float)[:2]
    wall_clearance = float(np.min(np.c_[path_xy - lo, hi - path_xy]))
    if nearest_distance - FOOTPRINT_RADIUS_M < 0.10 or wall_clearance < 0.50:
        raise RuntimeError("The synthetic path is too close to an obstacle or wall")
    info = {
        "scene": str(SCENE.relative_to(ROOT)),
        "provenance": "original synthetic geometry; not a measured Molonbot room",
        "home_x_m": 0.0,
        "home_y_m": 0.0,
        "home_yaw_rad": HOME_YAW,
        "aisle_heading_rad": AISLE_YAW,
        "required_yaw_change_rad": AISLE_YAW - HOME_YAW,
        "aisle_unit_x": float(direction[0]),
        "aisle_unit_y": float(direction[1]),
        "nominal_travel_m": TRAVEL_M,
        "checked_path": "trajectory.csv" if args.trace else "nominal_centerline",
        "checked_path_samples": len(path_xy),
        "wheel_track_m": TRACK_M,
        "wheel_radius_visual_estimate_m": WHEEL_RADIUS_M,
        "wheel_footprint_radius_visual_estimate_m": FOOTPRINT_RADIUS_M,
        "semantic_aabb_clearance_after_wheel_footprint_m": nearest_distance - FOOTPRINT_RADIUS_M,
        "semantic_aabb_center_clearance_m": nearest_distance,
        "nearest_semantic_object": nearest_name,
        "wall_center_clearance_m": wall_clearance,
        "caveat": "Synthetic model parameters and AABB clearance are for software verification, not physical robot safety.",
    }
    print(json.dumps(info, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
