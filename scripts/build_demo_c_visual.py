#!/usr/bin/env python3
"""Bake Demo C's verified FMU outputs into a small timeline-playable USD dial.

The wheel angle and signal bars are offline data visualization, not contact
physics or a ROS/robot command path.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom


ROOT = Path(__file__).resolve().parents[1]
TRACE = ROOT / "results/demo_C/trace.csv"
REPORT = ROOT / "results/demo_C/report.json"
OUTPUT = ROOT / "scenes/demo_C_wheel_visual.usda"


def cube(stage: Usd.Stage, path: str, position: tuple, scale: tuple, color: tuple):
    prim = UsdGeom.Cube.Define(stage, path)
    prim.CreateSizeAttr(1.0)
    prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xform = UsdGeom.Xformable(prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))
    return prim


def main() -> int:
    with TRACE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    report = json.loads(REPORT.read_text(encoding="utf-8"))
    if len(rows) != 1001 or report["trace_sha256"] != hashlib.sha256(TRACE.read_bytes()).hexdigest():
        raise ValueError("Demo C trace is missing or does not match the validation report")
    stage = Usd.Stage.CreateNew(str(OUTPUT))
    stage.GetRootLayer().subLayerPaths.append("demo_C_closed_loop.usda")
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(1000)
    stage.SetTimeCodesPerSecond(100)
    stage.SetFramesPerSecond(100)
    stage.SetMetadata("metersPerUnit", 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    UsdGeom.Xform.Define(stage, "/World/Visual")
    cube(stage, "/World/Visual/Base", (0, 0, -0.06), (3.8, 2.0, 0.10),
         (0.055, 0.090, 0.15))
    cylinder = UsdGeom.Cylinder.Define(stage, "/World/Visual/WheelDisk")
    cylinder.CreateRadiusAttr(0.50)
    cylinder.CreateHeightAttr(0.08)
    cylinder.CreateDisplayColorAttr([Gf.Vec3f(0.28, 0.35, 0.43)])
    UsdGeom.Xformable(cylinder).AddTranslateOp().Set(Gf.Vec3d(-0.92, 0, 0.06))
    wheel = UsdGeom.Xform.Define(stage, "/World/Visual/WheelAngle")
    wheel_xform = UsdGeom.Xformable(wheel)
    wheel_xform.AddTranslateOp().Set(Gf.Vec3d(-0.92, 0, 0.11))
    rotate = wheel_xform.AddRotateZOp()
    cube(stage, "/World/Visual/WheelAngle/Pointer", (0.26, 0, 0.02),
         (0.43, 0.055, 0.025), (1.0, 0.52, 0.12))
    cube(stage, "/World/Visual/WheelAngle/Hub", (0, 0, 0.04),
         (0.11, 0.11, 0.08), (0.85, 0.92, 0.98))
    cube(stage, "/World/Visual/ReferenceTrack", (0.66, 0.44, 0.03),
         (1.42, 0.12, 0.04), (0.20, 0.26, 0.34))
    cube(stage, "/World/Visual/MeasuredTrack", (0.66, -0.44, 0.03),
         (1.42, 0.12, 0.04), (0.20, 0.26, 0.34))
    ref_bar = cube(stage, "/World/Visual/ReferenceBar", (0, 0, 0), (1, 1, 1),
                   (1.0, 0.53, 0.15))
    speed_bar = cube(stage, "/World/Visual/MeasuredBar", (0, 0, 0), (1, 1, 1),
                     (0.16, 0.60, 0.94))
    ref_ops = UsdGeom.Xformable(ref_bar).GetOrderedXformOps()
    speed_ops = UsdGeom.Xformable(speed_bar).GetOrderedXformOps()
    for row in rows:
        frame = int(row["step"])
        angle = float(row["theta_rad_ovstage"])
        ref = float(row["reference_rad_s"])
        measured = float(row["omega_rad_s_ovstage"])
        rotate.Set(math.degrees(angle), Usd.TimeCode(frame))
        for value, y, ops in ((ref, 0.44, ref_ops), (measured, -0.44, speed_ops)):
            length = max(0.004, abs(value) / 30 * 1.25)
            center = 0.66 + math.copysign(length / 2, value) if value else 0.66
            ops[0].Set(Gf.Vec3d(center, y, 0.075), Usd.TimeCode(frame))
            ops[1].Set(Gf.Vec3f(length, 0.085, 0.035), Usd.TimeCode(frame))
    stage.GetRootLayer().Save()
    # Sdf's ASCII writer adds a spare blank line; keep the checked-in layer tidy.
    OUTPUT.write_bytes(OUTPUT.read_bytes().rstrip(b"\n") + b"\n")
    print(OUTPUT.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
