#!/usr/bin/env python3
"""Open a synthetic, FMU-derived animation in the Isaac Sim GUI."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", choices=("A", "B"), default="B")
    parser.add_argument("--no-autoplay", action="store_true")
    args, kit_args = parser.parse_known_args()
    sys.argv = [sys.argv[0], *kit_args]
    scene = ROOT / "scenes" / ("demo_A_robot_only.usda" if args.demo == "A" else "demo_B_synthetic_room.usda")

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": False})
    try:
        import omni.timeline
        import omni.usd
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        if not omni.usd.get_context().open_stage(str(scene)):
            raise RuntimeError(f"Isaac Sim could not open {scene}")
        for _ in range(40):
            app.update()
        stage = omni.usd.get_context().get_stage()
        if not stage or stage.GetEndTimeCode() != 789:
            raise RuntimeError("The animated USD stage was not loaded")
        stage.SetEditTarget(stage.GetSessionLayer())
        disabled_bodies = 0
        disabled_joints = 0
        for prim in list(stage.Traverse()):
            if prim.IsA(UsdPhysics.Joint):
                prim.SetActive(False)
                disabled_joints += 1
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr().Set(False)
                disabled_bodies += 1
        with (ROOT / "results/drive/trajectory.csv").open(newline="", encoding="utf-8") as stream:
            trajectory = list(csv.DictReader(stream))
        first, last = trajectory[0], trajectory[-1]
        center_x = (float(first["x_m"]) + float(last["x_m"])) / 2
        center_y = (float(first["y_m"]) + float(last["y_m"])) / 2
        viewport_camera = stage.GetPrimAtPath("/OmniverseKit_Persp")
        if viewport_camera.IsValid():
            center = Gf.Vec3d(center_x, center_y, 0.0)
            eye = Gf.Vec3d(center_x, center_y, 7.2 if args.demo == "B" else 4.3)
            xform = UsdGeom.Xformable(viewport_camera)
            xform.ClearXformOpOrder()
            xform.AddTransformOp().Set(
                Gf.Matrix4d().SetLookAt(eye, center, Gf.Vec3d(0, 1, 0)).GetInverse()
            )
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        timeline.set_start_time(0.0)
        timeline.set_end_time(7.89)
        timeline.set_auto_update(True)
        timeline.set_current_time(0.0)
        timeline.set_looping(True)
        if not args.no_autoplay:
            timeline.play()
        print(f"Demo {args.demo} open: {scene}", flush=True)
        print(
            f"Visual-only replay: {disabled_bodies} rigid bodies and "
            f"{disabled_joints} joints disabled in the session layer", flush=True,
        )
        print("Timeline: 0.00–7.89 s. Press Play to see the robot move; the physical robot is untouched.", flush=True)
        last_report = time.monotonic()
        while app.is_running():
            app.update()
            if time.monotonic() - last_report >= 5.0:
                current = timeline.get_current_time()
                pose = stage.GetPrimAtPath("/World/jetauto").GetAttribute(
                    "xformOp:translate"
                ).Get(Usd.TimeCode(current * stage.GetTimeCodesPerSecond()))
                print(
                    f"Timeline t={current:.2f} s, robot x={pose[0]:.2f} m, "
                    f"y={pose[1]:.2f} m", flush=True,
                )
                last_report = time.monotonic()
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
