#!/usr/bin/env python3
"""Render the published two-FMU closed-loop trace as an English SVG."""

from __future__ import annotations

import csv
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TRACE = ROOT / "results/demo_C/trace.csv"
OUTPUT = ROOT / "results/demo_C/trace.svg"
WIDTH, HEIGHT = 1060, 810
LEFT, RIGHT = 94, 33


def polyline(rows: list[dict], key: str, low: float, high: float, top: int, bottom: int) -> str:
    return " ".join(
        f"{LEFT + float(row['t_s']) / 10 * (WIDTH - LEFT - RIGHT):.2f},"
        f"{bottom - (float(row[key]) - low) / (high - low) * (bottom - top):.2f}"
        for row in rows
    )


def main() -> int:
    with TRACE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 1001 or rows[0]["step"] != "0" or rows[-1]["step"] != "1000":
        raise ValueError("Expected initial state plus 1,000 closed-loop steps")
    panels = (
        ("Wheel speed and reference (rad/s)", -10, 34,
         (("reference_rad_s", "#ef8b2c", 2.7), ("omega_rad_s_ovstage", "#1b70b6", 2.0))),
        ("Controller command and applied voltage (V)", -13, 13,
         (("voltage_cmd_V_ovstage", "#1b70b6", 2.2),
          ("voltage_applied_V_ovstage", "#a248a4", 1.6))),
        ("Controller error and integral contribution", -13, 35,
         (("error_rad_s_ovstage", "#ce5e3a", 1.9),
          ("integral_V_ovstage", "#2c9877", 1.9))),
    )
    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {WIDTH} {HEIGHT}" '
        'role="img" aria-label="Demo C controller and wheel-plant closed-loop signals">',
        '<rect width="100%" height="100%" fill="#fbfcff"/>',
        '<text x="94" y="34" font-family="sans-serif" font-size="23" fill="#1b2430">'
        'Demo C — FMI 3.0 closed-loop wheel control</text>',
        '<text x="94" y="57" font-family="sans-serif" font-size="13" fill="#526071">'
        'Two FMUs · 1,000 × 10 ms steps · USD / ovstage / ovfmi · offline</text>',
    ]
    for index, (title, low, high, series) in enumerate(panels):
        top = 111 + index * 230
        bottom = top + 174
        pieces.append(
            f'<text x="{LEFT}" y="{top - 12}" font-family="sans-serif" font-size="15" '
            f'font-weight="600" fill="#1b2430">{title}</text>'
        )
        for j in range(5):
            y = top + j * (bottom - top) / 4
            value = high - j * (high - low) / 4
            pieces.append(
                f'<line x1="{LEFT}" y1="{y:.2f}" x2="{WIDTH-RIGHT}" y2="{y:.2f}" '
                'stroke="#dce3ea"/>'
                f'<text x="{LEFT-8}" y="{y+4:.2f}" text-anchor="end" '
                f'font-family="sans-serif" font-size="11" fill="#526071">{value:g}</text>'
            )
        for second in range(11):
            x = LEFT + second / 10 * (WIDTH - LEFT - RIGHT)
            pieces.append(
                f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{bottom}" '
                'stroke="#eef1f5"/>'
                f'<text x="{x:.2f}" y="{bottom+18}" text-anchor="middle" '
                f'font-family="sans-serif" font-size="11" fill="#526071">{second}</text>'
            )
        for key, color, stroke in series:
            pieces.append(
                f'<polyline fill="none" stroke="{color}" stroke-width="{stroke}" '
                f'points="{polyline(rows, key, low, high, top, bottom)}"/>'
            )
    legend = (
        ("reference", "#ef8b2c"), ("wheel speed", "#1b70b6"),
        ("applied voltage", "#a248a4"), ("error", "#ce5e3a"),
        ("integral", "#2c9877"),
    )
    x = 94
    for label, color in legend:
        pieces.append(
            f'<line x1="{x}" y1="790" x2="{x+25}" y2="790" '
            f'stroke="{color}" stroke-width="3"/>'
            f'<text x="{x+32}" y="794" font-family="sans-serif" font-size="12" '
            f'fill="#1b2430">{label}</text>'
        )
        x += 170 if label != "applied voltage" else 185
    pieces.append('</svg>')
    OUTPUT.write_text("\n".join(pieces) + "\n", encoding="utf-8")
    print(OUTPUT.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
