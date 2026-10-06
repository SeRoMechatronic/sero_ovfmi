#!/usr/bin/env python3
"""Bake the verified four-FMU trajectory into reusable USD time samples.

The generated layer is shared by Demo A (robot only) and Demo B (room).
This is offline kinematic replay: it never connects to ROS or hardware.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom


ROOT = Path(__file__).resolve().parents[1]
TRACE = ROOT / "results/drive/trajectory.csv"
MOTION = ROOT / "scenes/robot_motion.usda"
SCENES = (
    ROOT / "scenes/demo_A_robot_only.usda",
    ROOT / "scenes/demo_B_synthetic_room.usda",
)
HOME = (0.0, 0.0, 0.0)
DT = 0.01


def pose_at_row(row: dict[str, str]) -> tuple[float, float, float]:
    return tuple(float(row[key]) for key in ("x_m", "y_m", "yaw_rad"))


def main() -> int:
    with TRACE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 789:
        raise ValueError(f"Expected 789 trajectory samples; got {len(rows)}")
    for index, row in enumerate(rows, 1):
        if not math.isclose(float(row["t_s"]), index * DT, abs_tol=1e-8):
            raise ValueError(f"Invalid time at sample {index}")
        if row["step_status"] != "ok":
            raise ValueError(f"FMU step {index} did not complete successfully")
    if any(abs(a - b) > 1e-6 for a, b in zip(pose_at_row(rows[0]), HOME)):
        raise ValueError("First FMU trajectory pose differs from the saved HOME pose")

    stage = Usd.Stage.CreateNew(str(MOTION))
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(len(rows))
    stage.SetTimeCodesPerSecond(round(1 / DT))
    stage.SetFramesPerSecond(round(1 / DT))
    stage.SetMetadata("customLayerData", {
        "purpose": "Baked four-FMU kinematic pose replay for Isaac Sim timeline playback",
        "source": "results/drive/trajectory.csv",
        "safety": "Offline animation; no physical robot motion",
    })
    robot = stage.OverridePrim("/World/jetauto")
    translate = robot.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3)
    orient = robot.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd)
    robot.CreateAttribute(
        "xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False,
        variability=Sdf.VariabilityUniform,
    ).Set(["xformOp:translate", "xformOp:orient"])
    for index, pose in enumerate([HOME, *(pose_at_row(row) for row in rows)]):
        x, y, yaw = pose
        time = Usd.TimeCode(index)
        translate.Set(Gf.Vec3d(x, y, 0.0), time)
        orient.Set(Gf.Quatd(math.cos(yaw / 2), Gf.Vec3d(0, 0, math.sin(yaw / 2))), time)
    stage.GetRootLayer().Save()
    MOTION.write_text(MOTION.read_text(encoding="utf-8").rstrip() + "\n", encoding="utf-8")

    for scene in SCENES:
        composed = Usd.Stage.Open(str(scene))
        if composed is None:
            raise RuntimeError(f"Could not open {scene}")
        prim = composed.GetPrimAtPath("/World/jetauto")
        if not prim.IsValid():
            raise RuntimeError(f"No robot in {scene}")
        attr = prim.GetAttribute("xformOp:translate")
        if attr.GetNumTimeSamples() != len(rows) + 1:
            raise RuntimeError(f"Time samples are missing from {scene}")
        if composed.GetEndTimeCode() != len(rows):
            raise RuntimeError(f"Incorrect timeline length in {scene}")
        for index in (0, 100, 400, 789):
            actual = attr.Get(Usd.TimeCode(index))
            expected = HOME if index == 0 else pose_at_row(rows[index - 1])
            if abs(actual[0] - expected[0]) > 1e-7 or abs(actual[1] - expected[1]) > 1e-7:
                raise RuntimeError(f"Pose mismatch at frame {index} in {scene}")
        print(f"OK: {scene.relative_to(ROOT)} — 790 animated poses, 7.89 s")
    print(f"Motion layer: {MOTION.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
