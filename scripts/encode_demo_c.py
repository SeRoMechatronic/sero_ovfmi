#!/usr/bin/env python3
"""Validate Isaac frames and encode the English-labelled Demo C MP4."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "results/demo_C"
TRACE = OUTPUT / "trace.csv"
VIDEO = OUTPUT / "demo_C_closed_loop.mp4"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    manifest = json.loads((OUTPUT / "render_manifest.json").read_text(encoding="utf-8"))
    scene = ROOT / manifest["scene"]
    if manifest["status"] != "complete" or manifest["demo"] != "C":
        raise RuntimeError("Complete Demo C render manifest required")
    if manifest["scene_sha256"] != digest(scene) or manifest["trace_sha256"] != digest(TRACE):
        raise RuntimeError("Rendered scene or trace changed")
    frames = [OUTPUT / "frames" / f"frame_{index:04d}.jpg"
              for index in range(manifest["frames"])]
    stream = hashlib.sha256()
    for path in frames:
        stream.update(path.read_bytes())
    if stream.hexdigest() != manifest["frame_stream_sha256"]:
        raise RuntimeError("Isaac Sim rendered frames changed")
    with TRACE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 1001:
        raise RuntimeError("Expected 1,001 time points in Demo C trace")
    width, height, fps = (manifest[key] for key in ("width", "height", "fps"))
    writer = cv2.VideoWriter(str(VIDEO), cv2.VideoWriter_fourcc(*"avc1"),
                             fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError("Could not initialize H.264 encoder")
    try:
        for index, path in enumerate(frames):
            frame = cv2.imread(str(path))
            if frame is None or frame.shape[:2] != (height, width):
                raise RuntimeError(f"Invalid rendered frame: {path}")
            t = min(index / fps, manifest["duration_s"])
            row = rows[min(1000, round(t * 100))]
            cv2.rectangle(frame, (0, 0), (width, 73), (13, 23, 38), -1)
            cv2.putText(frame, "DEMO C  |  openSeRo x ovfmi  |  ISAAC SIM",
                        (18, 28), cv2.FONT_HERSHEY_SIMPLEX, .64,
                        (255, 238, 221), 2, cv2.LINE_AA)
            cv2.putText(frame, "TWO FMI 3.0 FMUs / CLOSED LOOP / ONE-STEP DELAY / OFFLINE",
                        (20, 55), cv2.FONT_HERSHEY_SIMPLEX, .42,
                        (190, 218, 235), 1, cv2.LINE_AA)
            cv2.rectangle(frame, (0, height - 52), (width, height), (13, 23, 38), -1)
            label = (
                f"t={t:4.2f}s  REF={float(row['reference_rad_s']):+5.1f} rad/s  "
                f"SPEED={float(row['omega_rad_s_ovstage']):+5.1f} rad/s  "
                f"CMD={float(row['voltage_cmd_V_ovstage']):+5.1f} V"
            )
            cv2.putText(frame, label, (18, height - 19), cv2.FONT_HERSHEY_SIMPLEX,
                        .52, (255, 238, 221), 1, cv2.LINE_AA)
            writer.write(frame)
    finally:
        writer.release()
    capture = cv2.VideoCapture(str(VIDEO))
    actual = (int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),
              int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
              int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    capture.release()
    if actual != (manifest["frames"], width, height):
        raise RuntimeError(f"Encoded video frame count/dimensions mismatch: {actual}")
    metadata = {
        "demo": "C", "video": VIDEO.name, "sha256": digest(VIDEO),
        "scene": manifest["scene"], "scene_sha256": digest(scene),
        "trace_sha256": digest(TRACE), "frames": manifest["frames"],
        "fps": fps, "width": width, "height": height,
        "duration_s": manifest["duration_s"], "codec": "H.264",
        "scope": "Offline data visualization; wheel pointer follows FMU angle; no physics or hardware",
    }
    (OUTPUT / "video_metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(metadata, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
