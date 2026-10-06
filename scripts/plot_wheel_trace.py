#!/usr/bin/env python3
"""Generate a dependency-free SVG of the measured ovstage and FMPy traces."""

import argparse
import csv
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 1040, 760
LEFT, RIGHT = 95, 30
PANEL_HEIGHT = 200
X0, X1 = LEFT, WIDTH - RIGHT


def points(rows, key, y0, y1, lower, upper):
    scale = upper - lower or 1.0
    return " ".join(
        f"{X0 + float(row['t_s']) / 5.0 * (X1 - X0):.2f},"
        f"{y1 - (float(row[key]) - lower) / scale * (y1 - y0):.2f}"
        for row in rows
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=ROOT / "results" / "demo_a" / "trace.csv")
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "demo_a" / "trace.svg")
    args = parser.parse_args()
    with args.csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 500:
        raise ValueError(f"Expected 500 steps; got {len(rows)}")

    panels = (
        ("Commanded and applied voltage (V)", "voltage_cmd_V", "voltage_applied_ovstage_V", -0.5, 6.5),
        ("Wheel speed (rad/s)", "omega_fmpy_rad_s", "omega_ovstage_rad_s", -0.5, 12.5),
        ("Integrated angle (rad)", "theta_fmpy_rad", "theta_ovstage_rad", -1.0, 31.0),
    )
    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {WIDTH} {HEIGHT}" role="img" '
        'aria-label="Demo A traces: voltage, speed, and angle">',
        '<rect width="100%" height="100%" fill="#fbfcff"/>',
        '<text x="95" y="34" font-family="sans-serif" font-size="23" fill="#1b2430">'
        'Molonbot × ovfmi — 500 offline steps</text>',
        '<text x="95" y="56" font-family="sans-serif" font-size="13" fill="#526071">'
        'USD → ovstage → ovfmi → FMU → ovstage, compared with direct FMPy</text>',
    ]
    for index, (title, reference, actual, low, high) in enumerate(panels):
        top = 86 + index * 218
        bottom = top + PANEL_HEIGHT - 40
        pieces.append(
            f'<text x="{LEFT}" y="{top - 8}" font-family="sans-serif" font-size="15" '
            f'font-weight="600" fill="#1b2430">{title}</text>'
        )
        for j in range(5):
            y = top + j * (bottom - top) / 4
            value = high - j * (high - low) / 4
            pieces.append(
                f'<line x1="{X0}" y1="{y:.2f}" x2="{X1}" y2="{y:.2f}" '
                'stroke="#dce3ea" stroke-width="1"/>'
                f'<text x="{X0 - 8}" y="{y + 4:.2f}" text-anchor="end" '
                f'font-family="sans-serif" font-size="11" fill="#526071">{value:g}</text>'
            )
        for second in range(6):
            x = X0 + second / 5 * (X1 - X0)
            pieces.append(
                f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{bottom}" '
                'stroke="#eef1f5" stroke-width="1"/>'
                f'<text x="{x:.2f}" y="{bottom + 17}" text-anchor="middle" '
                f'font-family="sans-serif" font-size="11" fill="#526071">{second}</text>'
            )
        pieces.append(
            f'<polyline fill="none" stroke="#ec8b2c" stroke-width="3" '
            f'points="{points(rows, reference, top, bottom, low, high)}"/>'
        )
        pieces.append(
            f'<polyline fill="none" stroke="#1763a6" stroke-width="1.5" '
            f'points="{points(rows, actual, top, bottom, low, high)}"/>'
        )
    pieces += [
        '<line x1="95" y1="736" x2="125" y2="736" stroke="#ec8b2c" stroke-width="3"/>',
        '<text x="132" y="740" font-family="sans-serif" font-size="12" fill="#1b2430">direct FMPy</text>',
        '<line x1="250" y1="736" x2="280" y2="736" stroke="#1763a6" stroke-width="2"/>',
        '<text x="287" y="740" font-family="sans-serif" font-size="12" fill="#1b2430">published ovstage</text>',
        '<text x="1010" y="740" text-anchor="end" font-family="sans-serif" font-size="12" '
        'fill="#526071">simulation time (s)</text>',
        '</svg>',
    ]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(pieces) + "\n", encoding="utf-8")
    print(args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
