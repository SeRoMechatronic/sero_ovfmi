#!/usr/bin/env python3
"""Plot the published live PhysX turn trace as a dependency-free English SVG."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TRACE = ROOT / "results/demo_D_turn/trace.csv"
REPORT = ROOT / "results/demo_D_turn/report.json"
OUTPUT = ROOT / "results/demo_D_turn/turn.svg"
WIDTH, HEIGHT = 1100, 805
LEFT, RIGHT = 95, 45
WHEELS = ("FL", "FR", "BL", "BR")
COLORS = {"FL": "#265dca", "FR": "#e18921", "BL": "#15a28d", "BR": "#a257bc"}


def path(points: list[tuple[float, float]], t_max: float,
         low: float, high: float, top: float, bottom: float) -> str:
    return " ".join(
        f"{LEFT + t / t_max * (WIDTH - LEFT - RIGHT):.2f},"
        f"{bottom - (value - low) / (high - low) * (bottom - top):.2f}"
        for t, value in points
    )


def main() -> int:
    report = json.loads(REPORT.read_text(encoding="utf-8"))
    with TRACE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    steps = int(report["steps"])
    if (report["status"] != "passed" or report["profile"] != "turn"
            or len(rows) != 4 * steps or [row["wheel"] for row in rows[:4]] != list(WHEELS)):
        raise ValueError("Expected a complete, passing four-wheel PhysX turn trace")
    groups = [rows[4 * index:4 * index + 4] for index in range(steps)]
    t_max = steps * 0.01
    x0, y0 = float(groups[0][0]["base_x_m"]), float(groups[0][0]["base_y_m"])
    yaw0 = float(groups[0][0]["base_yaw_rad"])
    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {WIDTH} {HEIGHT}" '
        'role="img" aria-label="Demo D live PhysX turn, measured wheel speeds, heading, and base drift">',
        '<rect width="100%" height="100%" fill="#fbfcff"/>',
        '<text x="95" y="34" font-family="sans-serif" font-size="23" fill="#1b2430">'
        'Demo D — live FMU-controlled PhysX turn</text>',
        f'<text x="95" y="57" font-family="sans-serif" font-size="13" fill="#526071">'
        f'{steps} × 10 ms · bounded velocity drive · semantic room · simulated robot only</text>',
    ]
    panels = (
        ("Wheel angular velocity (rad/s)", -6.0, 6.0, 103, 272),
        ("Base heading change from start (rad)", -0.30, 0.12, 336, 505),
        ("Base displacement from start (m)", 0.0, 0.30, 569, 738),
    )
    for title, low, high, top, bottom in panels:
        pieces.append(f'<text x="{LEFT}" y="{top-14}" font-family="sans-serif" '
                      f'font-size="16" font-weight="600" fill="#1b2430">{title}</text>')
        for tick in range(5):
            y = top + tick * (bottom - top) / 4
            value = high - tick * (high - low) / 4
            pieces.append(f'<line x1="{LEFT}" y1="{y:.2f}" x2="{WIDTH-RIGHT}" '
                          f'y2="{y:.2f}" stroke="#dce3ea"/>'
                          f'<text x="{LEFT-9}" y="{y+4:.2f}" text-anchor="end" '
                          f'font-family="sans-serif" font-size="11" fill="#526071">'
                          f'{value:.2f}</text>')
        for second in range(math.floor(t_max) + 1):
            x = LEFT + second / t_max * (WIDTH - LEFT - RIGHT)
            pieces.append(f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" '
                          f'y2="{bottom}" stroke="#eef1f5"/>'
                          f'<text x="{x:.2f}" y="{bottom+18}" text-anchor="middle" '
                          f'font-family="sans-serif" font-size="11" fill="#526071">'
                          f'{second}s</text>')
        for start, stop in ((0.2, 1.0), (1.4, 2.2)):
            x = LEFT + start / t_max * (WIDTH - LEFT - RIGHT)
            width = (stop - start) / t_max * (WIDTH - LEFT - RIGHT)
            pieces.append(f'<rect x="{x:.2f}" y="{top}" width="{width:.2f}" '
                          f'height="{bottom-top}" fill="#e8efff" opacity="0.45"/>')
        if title.startswith("Wheel"):
            for index, wheel in enumerate(WHEELS):
                values = [(float(group[index]["t_s"]),
                           float(group[index]["measured_after_rad_s"])) for group in groups]
                pieces.append(f'<polyline points="{path(values, t_max, low, high, top, bottom)}" '
                              f'fill="none" stroke="{COLORS[wheel]}" stroke-width="1.7"/>')
        elif title.startswith("Base heading"):
            values = [(float(group[0]["t_s"]), float(group[0]["base_yaw_rad"]) - yaw0)
                      for group in groups]
            pieces.append(f'<polyline points="{path(values, t_max, low, high, top, bottom)}" '
                          'fill="none" stroke="#264e96" stroke-width="2.4"/>')
        else:
            values = [(float(group[0]["t_s"]),
                       math.hypot(float(group[0]["base_x_m"]) - x0,
                                  float(group[0]["base_y_m"]) - y0)) for group in groups]
            pieces.append(f'<polyline points="{path(values, t_max, low, high, top, bottom)}" '
                          'fill="none" stroke="#cf5e3c" stroke-width="2.2"/>')
    legend = [("FL", COLORS["FL"]), ("FR", COLORS["FR"]),
              ("BL", COLORS["BL"]), ("BR", COLORS["BR"]),
              ("heading", "#264e96"), ("drift", "#cf5e3c")]
    for index, (label, color) in enumerate(legend):
        x = 95 + index * 150
        pieces.append(f'<line x1="{x}" y1="788" x2="{x+25}" y2="788" '
                      f'stroke="{color}" stroke-width="3"/>'
                      f'<text x="{x+32}" y="792" font-family="sans-serif" '
                      f'font-size="12" fill="#1b2430">{label}</text>')
    pieces.append("</svg>")
    OUTPUT.write_text("\n".join(pieces) + "\n", encoding="utf-8")
    print(OUTPUT.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
