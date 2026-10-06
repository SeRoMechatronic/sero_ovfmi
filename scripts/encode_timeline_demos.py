#!/usr/bin/env python3
"""Verify Isaac frame renders and encode English-labelled Demo A/B MP4 files."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parents[1]
TRACE = ROOT / "results/drive/trajectory.csv"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def encode(demo: str) -> dict:
    output = ROOT / "results" / f"demo_{demo}"
    manifest = json.loads((output / "render_manifest.json").read_text(encoding="utf-8"))
    if manifest.get("demo") != demo or manifest.get("status") != "complete":
        raise RuntimeError(f"Demo {demo}: complete render manifest not found")
    if manifest["trace_sha256"] != sha256(TRACE):
        raise RuntimeError(f"Demo {demo}: render and trajectory have different hashes")
    scene = ROOT / manifest["scene"]
    if manifest["scene_sha256"] != sha256(scene):
        raise RuntimeError(f"Demo {demo}: animated scene has changed since rendering")
    with TRACE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    fps = manifest["fps"]
    count = manifest["frames"]
    width, height = manifest["width"], manifest["height"]
    if count != math.ceil(manifest["duration_s"] * fps) + 1:
        raise RuntimeError(f"Demo {demo}: unexpected frame count")
    paths = [output / "frames" / f"frame_{i:04d}.jpg" for i in range(count)]
    combined = hashlib.sha256()
    for path in paths:
        combined.update(path.read_bytes())
    if combined.hexdigest() != manifest["frame_stream_sha256"]:
        raise RuntimeError(f"Demo {demo}: rendered frames changed")
    video = output / (
        "demo_A_robot_only.mp4" if demo == "A" else "demo_B_synthetic_room.mp4"
    )
    writer = cv2.VideoWriter(
        str(video), cv2.VideoWriter_fourcc(*"avc1"), fps, (width, height)
    )
    if not writer.isOpened():
        raise RuntimeError("Could not initialize the H.264 encoder")
    try:
        for index, path in enumerate(paths):
            frame = cv2.imread(str(path))
            if frame is None or frame.shape[:2] != (height, width):
                raise RuntimeError(f"Invalid rendered frame: {path}")
            t = min(index / fps, manifest["duration_s"])
            row = rows[max(0, min(len(rows) - 1, round(t * 100) - 1))]
            cv2.rectangle(frame, (0, 0), (width, 70), (14, 26, 40), -1)
            cv2.putText(
                frame, f"DEMO {demo}  |  MOLONBOT x ovfmi  |  ISAAC SIM",
                (20, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 238, 222), 2,
                cv2.LINE_AA,
            )
            subtitle = (
                "ROBOT ONLY - OFFLINE FMU-DRIVEN KINEMATIC REPLAY" if demo == "A"
                else "ROBOT IN SYNTHETIC ROOM - OFFLINE FMU-DRIVEN REPLAY"
            )
            cv2.putText(
                frame, subtitle, (21, 53), cv2.FONT_HERSHEY_SIMPLEX,
                .42, (187, 217, 235), 1, cv2.LINE_AA,
            )
            cv2.rectangle(frame, (0, height - 48), (width, height), (14, 26, 40), -1)
            cv2.putText(
                frame,
                f"t={t:.2f} s   x={float(row['x_m']):+.2f} m   "
                f"y={float(row['y_m']):+.2f} m   "
                f"yaw={math.degrees(float(row['yaw_rad'])):+.1f} deg",
                (21, height - 18), cv2.FONT_HERSHEY_SIMPLEX,
                .52, (255, 238, 222), 1, cv2.LINE_AA,
            )
            writer.write(frame)
    finally:
        writer.release()
    capture = cv2.VideoCapture(str(video))
    actual = (
        int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),
        int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
        int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    )
    capture.release()
    if actual != (count, width, height):
        raise RuntimeError(f"Encoded video dimensions/frame count mismatch: {actual}")
    metadata = {
        "demo": demo,
        "video": video.name,
        "sha256": sha256(video),
        "scene": manifest["scene"],
        "scene_sha256": manifest["scene_sha256"],
        "trace_sha256": manifest["trace_sha256"],
        "fps": fps,
        "frames": count,
        "width": width,
        "height": height,
        "duration_s": manifest["duration_s"],
        "codec": "H.264",
        "scope": "Offline kinematic replay; no robot hardware or contact physics",
    }
    (output / "video_metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    return metadata


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", choices=("A", "B", "both"), default="both")
    args = parser.parse_args()
    for demo in (("A", "B") if args.demo == "both" else (args.demo,)):
        print(json.dumps(encode(demo), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
