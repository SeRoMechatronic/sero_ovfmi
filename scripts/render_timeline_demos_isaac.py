#!/usr/bin/env python3
"""Render Demo A or B from the animated USD timeline in Isaac Sim.

The USD already contains the FMU-derived pose samples. This script samples
the timeline; it does not author robot motion during rendering.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCENES = {
    "A": ROOT / "scenes/demo_A_robot_only.usda",
    "B": ROOT / "scenes/demo_B_synthetic_room.usda",
}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", choices=SCENES, required=True)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--subframes", type=int, default=2)
    args, kit_args = parser.parse_known_args()
    if min(args.width, args.height, args.fps, args.subframes) <= 0:
        raise ValueError("Resolution, frame rate, and subframes must be positive")
    sys.argv = [sys.argv[0], *kit_args]

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "width": args.width, "height": args.height})
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

        scene = SCENES[args.demo]
        omni.usd.get_context().open_stage(str(scene))
        for _ in range(30):
            app.update()
        stage = omni.usd.get_context().get_stage()
        robot = stage.GetPrimAtPath("/World/jetauto") if stage else None
        if not robot or not robot.IsValid():
            raise RuntimeError(f"Isaac Sim could not load {scene}")
        if robot.GetAttribute("xformOp:translate").GetNumTimeSamples() != 790:
            raise RuntimeError("Robot USD animation is missing")
        stage.SetEditTarget(stage.GetSessionLayer())
        # Rendering-only edits: room and saved animation layers stay unchanged.
        disabled_bodies = 0
        disabled_joints = 0
        for prim in list(stage.Traverse()):
            if prim.IsA(UsdPhysics.Joint):
                prim.SetActive(False)
                disabled_joints += 1
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr().Set(False)
                disabled_bodies += 1
        print(
            f"Disabled {disabled_bodies} rigid bodies and {disabled_joints} joints "
            "in the render session", flush=True,
        )
        with (ROOT / "results/drive/trajectory.csv").open(newline="", encoding="utf-8") as stream:
            trajectory = list(csv.DictReader(stream))
        first, last = trajectory[0], trajectory[-1]
        center_x = (float(first["x_m"]) + float(last["x_m"])) / 2
        center_y = (float(first["y_m"]) + float(last["y_m"])) / 2
        camera = UsdGeom.Camera.Define(stage, "/Render/TimelineCamera")
        camera.CreateFocalLengthAttr(18.0)
        camera.CreateHorizontalApertureAttr(20.955)
        camera.CreateVerticalApertureAttr(20.955 * args.height / args.width)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
        look = Gf.Vec3d(center_x, center_y, 0)
        eye = Gf.Vec3d(center_x, center_y, 7.2 if args.demo == "B" else 4.3)
        up = Gf.Vec3d(0, 1, 0)
        UsdGeom.Xformable(camera).AddTransformOp().Set(
            Gf.Matrix4d().SetLookAt(eye, look, up).GetInverse()
        )
        UsdLux.DomeLight.Define(stage, "/Render/Sky").CreateIntensityAttr(900.0)
        sun = UsdLux.DistantLight.Define(stage, "/Render/Sun")
        sun.CreateIntensityAttr(2500.0)
        sun.CreateAngleAttr(1.0)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35, 20, 0))

        product = rep.create.render_product("/Render/TimelineCamera", (args.width, args.height))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach([product])
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        output = ROOT / "results" / f"demo_{args.demo}"
        frames = output / "frames"
        frames.mkdir(parents=True, exist_ok=True)
        duration = stage.GetEndTimeCode() / stage.GetTimeCodesPerSecond()
        timeline.set_start_time(0.0)
        timeline.set_end_time(duration)
        timeline.set_auto_update(False)
        count = math.ceil(duration * args.fps) + 1
        for index in range(count):
            t = min(index / args.fps, duration)
            timeline.set_current_time(t)
            expected = robot.GetAttribute("xformOp:translate").Get(
                Usd.TimeCode(t * stage.GetTimeCodesPerSecond())
            )
            if expected is None:
                raise RuntimeError(f"No animated robot pose at {t:.3f} s")
            app.update()
            if abs(timeline.get_current_time() - t) > 0.02:
                raise RuntimeError(
                    f"Timeline did not seek to {t:.3f} s; actual {timeline.get_current_time():.3f} s"
                )
            rep.orchestrator.step(rt_subframes=args.subframes, delta_time=0.0)
            rgb = np.asarray(annotator.get_data())[:, :, :3]
            if rgb.shape != (args.height, args.width, 3):
                raise RuntimeError(f"Unexpected image dimensions at frame {index}: {rgb.shape}")
            Image.fromarray(rgb).save(frames / f"frame_{index:04d}.jpg", quality=89)
            if index % 12 == 0 or index == count - 1:
                print(f"Demo {args.demo}: frame {index + 1}/{count}, t={t:.2f} s", flush=True)
        annotator.detach([product])
        product.destroy()
        combined = hashlib.sha256()
        for index in range(count):
            combined.update((frames / f"frame_{index:04d}.jpg").read_bytes())
        metadata = {
            "demo": args.demo,
            "status": "complete",
            "scene": str(scene.relative_to(ROOT)),
            "scene_sha256": digest(scene),
            "motion_layer_sha256": digest(ROOT / "scenes/robot_motion.usda"),
            "trace_sha256": digest(ROOT / "results/drive/trajectory.csv"),
            "frame_stream_sha256": combined.hexdigest(),
            "frames": count,
            "fps": args.fps,
            "duration_s": duration,
            "width": args.width,
            "height": args.height,
            "scope": "Isaac Sim render of offline, time-sampled FMU-derived USD robot animation",
            "rigid_bodies_disabled_for_replay": disabled_bodies,
            "joints_disabled_for_replay": disabled_joints,
        }
        (output / "render_manifest.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(metadata, indent=2), flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
