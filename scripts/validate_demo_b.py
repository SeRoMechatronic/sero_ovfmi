#!/usr/bin/env python3
"""Validate Demo B's composed USD replay against the four-FMU trace.

This is an offline numerical/geometry consistency check for synthetic assets.
It does not run Isaac physics, estimate a real room, or prove robot safety.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from pxr import Usd, UsdGeom, UsdUtils


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes/demo_B_synthetic_room.usda"
DRIVE_SCENE = ROOT / "scenes/demo_movimiento_4ruedas.usda"
MOTION = ROOT / "scenes/robot_motion.usda"
TRACE = ROOT / "results/drive/trajectory.csv"
DRIVE_REPORT = ROOT / "results/drive/report.json"
FMU = ROOT / "fmus/MolonbotWheelPlant.fmu"
RENDER = ROOT / "results/demo_B/render_manifest.json"
VIDEO_META = ROOT / "results/demo_B/video_metadata.json"
VIDEO = ROOT / "results/demo_B/demo_B_synthetic_room.mp4"
OUTPUT = ROOT / "results/demo_B/validation/report.json"
NAMES = ("FL", "FR", "BL", "BR")
MAPPINGS = {
    "voltage_cmd_V": "input",
    "voltage_applied_V": "output",
    "omega_rad_s": "output",
    "theta_rad": "output",
}
POSITION_TOL_M = 1e-9
HEADING_TOL_RAD = 1e-9
MIN_SYNTHETIC_MARGIN_M = 0.10


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def rectangle_distance(x: float, y: float, bounds) -> float:
    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    dx = max(float(minimum[0]) - x, 0.0, x - float(maximum[0]))
    dy = max(float(minimum[1]) - y, 0.0, y - float(maximum[1]))
    return math.hypot(dx, dy)


def check_mappings(stage: Usd.Stage) -> None:
    plants = stage.GetPrimAtPath("/World/DrivePlants")
    require(plants.IsValid(), "Demo B has no composed FMU plant prims")
    require({prim.GetName() for prim in plants.GetChildren()} == set(NAMES),
            "Demo B does not contain exactly FL/FR/BL/BR FMU instances")
    for name in NAMES:
        plant = stage.GetPrimAtPath(f"/World/DrivePlants/{name}")
        require(plant.GetTypeName() == "FmuInstance", f"{name}: wrong prim type")
        require(plant.GetAttribute("fmi:enabled").Get() is True, f"{name}: disabled FMU")
        asset = plant.GetAttribute("fmi:fmu").Get()
        require(asset is not None and (DRIVE_SCENE.parent / asset.path).resolve() == FMU,
                f"{name}: wrong FMU asset")
        connection = plant.GetPrimAtPath(f"/World/DrivePlants/{name}/Signals")
        require(connection.GetTypeName() == "FmuConnection", f"{name}: missing connection")
        require([str(path) for path in connection.GetRelationship("fmi:targets").GetTargets()]
                == [f"/World/DriveStates/{name}"], f"{name}: wrong state target")
        mappings = {}
        for mapping in connection.GetChildren():
            require(mapping.GetTypeName() == "FmuMapping", f"{name}: wrong mapping prim")
            variable = mapping.GetAttribute("fmi:fmuAttribute").Get()
            usd_attribute = mapping.GetAttribute("fmi:usdAttribute").Get()
            direction = mapping.GetAttribute("fmi:direction").Get()
            require(variable in MAPPINGS, f"{name}: unknown FMU variable {variable}")
            require(variable not in mappings, f"{name}: duplicate mapping for {variable}")
            require(usd_attribute == f"molonbot:{variable}" and direction == MAPPINGS[variable],
                    f"{name}: wrong mapping for {variable}")
            require(tuple(mapping.GetAttribute("fmi:usdMapping").Get()) == (0, 1),
                    f"{name}: wrong USD mapping range for {variable}")
            mappings[variable] = direction
        require(mappings == MAPPINGS, f"{name}: incomplete mapping set")
        state = stage.GetPrimAtPath(f"/World/DriveStates/{name}")
        require(state.IsValid(), f"{name}: missing state prim")
        require(all(state.GetAttribute(f"molonbot:{variable}").IsValid()
                    for variable in MAPPINGS), f"{name}: missing state attribute")


def validate(scene_path: Path = SCENE, trace_path: Path = TRACE) -> dict:
    drive = json.loads(DRIVE_REPORT.read_text(encoding="utf-8"))
    render = json.loads(RENDER.read_text(encoding="utf-8"))
    video_meta = json.loads(VIDEO_META.read_text(encoding="utf-8"))
    scene_hash = sha256(scene_path)
    trace_hash = sha256(trace_path)
    fmu_hash = sha256(FMU)
    motion_hash = sha256(MOTION)
    require(drive["status"] == "passed", "Shared four-FMU run did not pass")
    require(drive["max_abs_ovfmi_vs_fmpy"] <= 5e-6,
            "Shared four-FMU/FMPy error exceeds its declared tolerance")
    require(drive["trajectory_sha256"] == trace_hash, "Four-FMU trace hash mismatch")
    require(drive["fmu_sha256"] == fmu_hash, "Four-FMU plant hash mismatch")
    require(drive["scene_sha256"] == sha256(DRIVE_SCENE),
            "Four-FMU USD stage hash mismatch")
    require(render["demo"] == "B" and render["status"] == "complete"
            and render["scene"] == str(SCENE.relative_to(ROOT))
            and render["scene_sha256"] == scene_hash
            and render["motion_layer_sha256"] == motion_hash
            and render["trace_sha256"] == trace_hash, "Demo B render provenance mismatch")
    require(video_meta["demo"] == "B" and video_meta["video"] == VIDEO.name
            and video_meta["scene"] == str(SCENE.relative_to(ROOT))
            and video_meta["scene_sha256"] == scene_hash
            and video_meta["trace_sha256"] == trace_hash
            and video_meta["sha256"] == sha256(VIDEO)
            and video_meta["frames"] == render["frames"],
            "Demo B video provenance mismatch")

    with trace_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    require(len(rows) == drive["steps"] == 789, "Wrong four-FMU trajectory length")
    require(all(row["step_status"] == "ok" for row in rows), "Failed FMU step in trajectory")

    stage = Usd.Stage.Open(str(scene_path))
    require(stage is not None, "Could not open Demo B USD stage")
    require(stage.GetStartTimeCode() == 0 and stage.GetEndTimeCode() == len(rows)
            and stage.GetTimeCodesPerSecond() == 100, "Wrong Demo B timeline metadata")
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(scene_path))
    require(not unresolved, f"Unresolved Demo B USD dependencies: {unresolved}")
    require(all(Path(str(getattr(item, "identifier", item))).resolve().is_relative_to(ROOT)
                for item in (*layers, *assets)), "Demo B references assets outside this repository")
    check_mappings(stage)

    robot = stage.GetPrimAtPath("/World/jetauto")
    require(robot.IsValid(), "Demo B has no robot prim")
    position = robot.GetAttribute("xformOp:translate")
    orientation = robot.GetAttribute("xformOp:orient")
    expected_time_codes = list(range(len(rows) + 1))
    require(position.GetTimeSamples() == expected_time_codes
            and orientation.GetTimeSamples() == expected_time_codes,
            "Demo B pose samples are incomplete")
    samples = [(0.0, 0.0, 0.0)] + [
        (float(row["x_m"]), float(row["y_m"]), float(row["yaw_rad"]))
        for row in rows
    ]
    max_position_error = 0.0
    max_heading_error = 0.0
    path_xy = []
    for index, (x, y, yaw) in enumerate(samples):
        if index:
            require(math.isclose(float(rows[index - 1]["t_s"]), index / 100,
                                 abs_tol=1e-9), f"Wrong trajectory time at step {index}")
        pose = position.Get(Usd.TimeCode(index))
        quat = orientation.Get(Usd.TimeCode(index))
        require(pose is not None and quat is not None, f"Missing pose at frame {index}")
        require(all(math.isfinite(float(value)) for value in (x, y, yaw, *pose,
                    quat.GetReal(), *quat.GetImaginary())), f"Non-finite pose at frame {index}")
        max_position_error = max(max_position_error,
                                 math.dist((x, y, 0.0), tuple(float(v) for v in pose)))
        imaginary = quat.GetImaginary()
        require(abs(float(imaginary[0])) <= HEADING_TOL_RAD
                and abs(float(imaginary[1])) <= HEADING_TOL_RAD,
                f"Unexpected non-planar orientation at frame {index}")
        actual_yaw = 2 * math.atan2(float(imaginary[2]), float(quat.GetReal()))
        heading_error = (actual_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        max_heading_error = max(max_heading_error, abs(heading_error))
        path_xy.append((x, y))
    require(max_position_error <= POSITION_TOL_M and max_heading_error <= HEADING_TOL_RAD,
            "Demo B USD poses differ from the four-FMU trajectory")

    radius = float(drive["geometry"]["wheel_footprint_radius_visual_estimate_m"])
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    semantic = stage.GetPrimAtPath("/World/gemelo_semantico")
    structure = stage.GetPrimAtPath("/World/structure")
    require(semantic.IsValid() and structure.IsValid(), "Demo B room geometry is missing")
    obstacles = list(semantic.GetChildren())
    walls = [prim for prim in structure.GetChildren() if prim.GetName().endswith("Wall")]
    require(obstacles and len(walls) == 4, "Demo B obstacles or four walls are missing")
    object_bounds = [(prim.GetName(), bbox.ComputeWorldBound(prim).ComputeAlignedRange())
                     for prim in obstacles]
    wall_bounds = [(prim.GetName(), bbox.ComputeWorldBound(prim).ComputeAlignedRange())
                   for prim in walls]
    floor = stage.GetPrimAtPath("/World/structure/Floor")
    require(floor.IsValid(), "Demo B floor is missing")
    floor_range = bbox.ComputeWorldBound(floor).ComputeAlignedRange()
    floor_min = floor_range.GetMin()
    floor_max = floor_range.GetMax()
    nearest_object = (math.inf, "")
    nearest_wall = (math.inf, "")
    floor_margin = math.inf
    for x, y in path_xy:
        for name, bounds in object_bounds:
            nearest_object = min(nearest_object, (rectangle_distance(x, y, bounds), name))
        for name, bounds in wall_bounds:
            nearest_wall = min(nearest_wall, (rectangle_distance(x, y, bounds), name))
        floor_margin = min(floor_margin, x - float(floor_min[0]),
                           float(floor_max[0]) - x, y - float(floor_min[1]),
                           float(floor_max[1]) - y)
    object_margin = nearest_object[0] - radius
    wall_margin = nearest_wall[0] - radius
    floor_margin -= radius
    require(math.isclose(
        object_margin,
        float(drive["path_validation"]["semantic_aabb_clearance_after_wheel_footprint_m"]),
        abs_tol=1e-9,
    ), "Demo B semantic clearance differs from the shared four-FMU path check")
    require(min(object_margin, wall_margin, floor_margin) >= MIN_SYNTHETIC_MARGIN_M,
            "Demo B synthetic trajectory violates the declared coarse clearance margin")

    return {
        "status": "passed",
        "scope": "Offline numerical consistency of Demo B's synthetic composed scene; not physical safety or contact simulation",
        "scene": str(scene_path.relative_to(ROOT)),
        "scene_sha256": scene_hash,
        "motion_layer_sha256": motion_hash,
        "trajectory": str(trace_path.relative_to(ROOT)) if trace_path.is_relative_to(ROOT) else str(trace_path),
        "trajectory_sha256": trace_hash,
        "fmu_sha256": fmu_hash,
        "steps": len(rows),
        "checked_pose_samples": len(samples),
        "max_position_error_m": max_position_error,
        "max_heading_error_rad": max_heading_error,
        "fmu_instances": len(NAMES),
        "mapped_signals_per_instance": len(MAPPINGS),
        "shared_four_fmu_max_abs_ovfmi_vs_fmpy": drive["max_abs_ovfmi_vs_fmpy"],
        "synthetic_footprint_radius_m": radius,
        "minimum_semantic_aabb_margin_m": object_margin,
        "nearest_semantic_object": nearest_object[1],
        "minimum_wall_aabb_margin_m": wall_margin,
        "nearest_wall": nearest_wall[1],
        "minimum_floor_edge_margin_m": floor_margin,
        "video_sha256": sha256(VIDEO),
        "evidence_note": "The four-FMU/FMPy comparison and trajectory are shared with Demo A's visual replay; Demo B additionally checks its own USD pose and room geometry. No independent FMU run is claimed.",
        "limitations": [
            "Original synthetic room, not an observed or reconstructed physical room.",
            "Circular 2D footprint and axis-aligned boxes are coarse software checks, not collision simulation.",
            "USD timeline poses are offline kinematic replay; Isaac Sim rendering does not validate traction, Nav2, or real-world clearance.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    report = validate()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in (
        "status", "steps", "checked_pose_samples", "max_position_error_m",
        "max_heading_error_rad", "minimum_semantic_aabb_margin_m",
        "minimum_wall_aabb_margin_m", "minimum_floor_edge_margin_m",
    )}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
