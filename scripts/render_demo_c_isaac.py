#!/usr/bin/env python3
"""Render Demo C's offline USD signal dial with Isaac Sim (no hardware/physics)."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes/demo_C_wheel_visual.usda"
TRACE = ROOT / "results/demo_C/trace.csv"
OUTPUT = ROOT / "results/demo_C"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=6)
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
        from pxr import Gf, Usd, UsdGeom, UsdLux

        omni.usd.get_context().open_stage(str(SCENE))
        for _ in range(30):
            app.update()
        stage = omni.usd.get_context().get_stage()
        wheel = stage.GetPrimAtPath("/World/Visual/WheelAngle") if stage else None
        if not wheel or not wheel.IsValid():
            raise RuntimeError(f"Isaac Sim could not load {SCENE}")
        angle = wheel.GetAttribute("xformOp:rotateZ")
        if angle.GetNumTimeSamples() != 1001:
            raise RuntimeError("Wheel USD animation is missing")
        stage.SetEditTarget(stage.GetSessionLayer())
        camera = UsdGeom.Camera.Define(stage, "/Render/DemoCCamera")
        camera.CreateFocalLengthAttr(18.0)
        camera.CreateHorizontalApertureAttr(20.955)
        camera.CreateVerticalApertureAttr(20.955 * args.height / args.width)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 50.0))
        UsdGeom.Xformable(camera).AddTransformOp().Set(
            Gf.Matrix4d().SetLookAt(Gf.Vec3d(0, 0, 4.5), Gf.Vec3d(0, 0, 0),
                                     Gf.Vec3d(0, 1, 0)).GetInverse()
        )
        UsdLux.DomeLight.Define(stage, "/Render/Sky").CreateIntensityAttr(1100.0)
        product = rep.create.render_product("/Render/DemoCCamera", (args.width, args.height))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach([product])
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        duration = 10.0
        timeline.set_start_time(0.0)
        timeline.set_end_time(duration)
        timeline.set_auto_update(False)
        count = math.ceil(duration * args.fps) + 1
        frames = OUTPUT / "frames"
        frames.mkdir(parents=True, exist_ok=True)
        for index in range(count):
            t = min(index / args.fps, duration)
            timeline.set_current_time(t)
            if angle.Get(Usd.TimeCode(t * 100)) is None:
                raise RuntimeError(f"No wheel angle sample at {t:.3f} s")
            app.update()
            rep.orchestrator.step(rt_subframes=args.subframes, delta_time=0.0)
            rgb = np.asarray(annotator.get_data())[:, :, :3]
            if rgb.shape != (args.height, args.width, 3):
                raise RuntimeError(f"Unexpected image dimensions at frame {index}: {rgb.shape}")
            Image.fromarray(rgb).save(frames / f"frame_{index:04d}.jpg", quality=90)
            if index % 12 == 0 or index == count - 1:
                print(f"Demo C: frame {index + 1}/{count}, t={t:.2f} s", flush=True)
        annotator.detach([product])
        product.destroy()
        stream = hashlib.sha256()
        for index in range(count):
            stream.update((frames / f"frame_{index:04d}.jpg").read_bytes())
        metadata = {
            "demo": "C", "status": "complete", "scene": str(SCENE.relative_to(ROOT)),
            "scene_sha256": digest(SCENE), "trace_sha256": digest(TRACE),
            "frame_stream_sha256": stream.hexdigest(), "frames": count,
            "fps": args.fps, "duration_s": duration, "width": args.width,
            "height": args.height,
            "scope": "Isaac Sim render of offline FMU-derived USD signal visualization; no physical simulation",
        }
        (OUTPUT / "render_manifest.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(metadata, indent=2), flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
