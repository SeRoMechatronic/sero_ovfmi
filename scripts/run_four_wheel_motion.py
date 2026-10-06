#!/usr/bin/env python3
"""Four-FMU offline drive trace in an explicitly synthetic USD room.

This is kinematic playback based on four independent wheel-plant FMUs and
chosen synthetic dimensions. It is NOT Nav2, robot control, contact physics,
or a calibrated dynamics model of the real JetAutoPro.
"""

from __future__ import annotations

import contextlib
import csv
import hashlib
import io
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import fmpy
import numpy as np
import ovstage
from fmpy.fmi3 import FMU3Slave
from ovfmi import FmiHost
from ovstage import population


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "demo_movimiento_4ruedas.usda"
FMU = ROOT / "fmus" / "MolonbotWheelPlant.fmu"
OUT = ROOT / "results" / "drive"
NAMES = ("FL", "FR", "BL", "BR")
PATHS = tuple(f"/World/DriveStates/{name}" for name in NAMES)
OUTPUTS = ("voltage_applied_V", "omega_rad_s", "theta_rad")
DT = 0.01
TURN_V = 1.5
FORWARD_V = 6.0
LEAD_STEPS = 20
SETTLE_STEPS = 150


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def geometry() -> dict:
    helper = ROOT / "scripts" / "inspect_drive_geometry.py"
    result = subprocess.run(
        [sys.executable, str(helper)], check=True, text=True,
        capture_output=True, timeout=30,
    )
    return json.loads(result.stdout)


def profile(geometry_info: dict) -> tuple[list[tuple[float, float, float, float]], dict]:
    radius = geometry_info["wheel_radius_visual_estimate_m"]
    track = geometry_info["wheel_track_m"]
    yaw_change = geometry_info["required_yaw_change_rad"]
    turn_omega = 2.0 * TURN_V
    forward_omega = 2.0 * FORWARD_V
    turn_steps = round(abs(yaw_change) * track / (2.0 * radius * turn_omega * DT))
    drive_steps = round(geometry_info["nominal_travel_m"] / (radius * forward_omega * DT))
    left_turn = -math.copysign(TURN_V, yaw_change)
    right_turn = -left_turn
    commands = (
        [(0.0, 0.0, 0.0, 0.0)] * LEAD_STEPS
        + [(left_turn, right_turn, left_turn, right_turn)] * turn_steps
        + [(0.0, 0.0, 0.0, 0.0)] * SETTLE_STEPS
        + [(FORWARD_V, FORWARD_V, FORWARD_V, FORWARD_V)] * drive_steps
        + [(0.0, 0.0, 0.0, 0.0)] * SETTLE_STEPS
    )
    info = {
        "lead_steps": LEAD_STEPS,
        "turn_steps": turn_steps,
        "turn_settle_steps": SETTLE_STEPS,
        "drive_steps": drive_steps,
        "drive_settle_steps": SETTLE_STEPS,
        "turn_voltage_left_V": left_turn,
        "turn_voltage_right_V": right_turn,
        "forward_voltage_V": FORWARD_V,
        "profile_note": "Zero input first; symmetric opposite wheel commands for turn; equal commands for drive; settle with 0 V.",
    }
    return commands, info


def fmpy_reference(commands: list[tuple[float, ...]]) -> list[dict[str, tuple[float, float, float]]]:
    model = fmpy.read_model_description(str(FMU), validate=True)
    references = {variable.name: variable.valueReference for variable in model.modelVariables}
    all_samples: list[dict[str, tuple[float, float, float]]] = []
    with tempfile.TemporaryDirectory(prefix="molonbot_four_wheels_") as directory:
        fmpy.extract(str(FMU), unzipdir=directory)
        wheels = []
        for name in NAMES:
            slave = FMU3Slave(
                guid=model.guid, unzipDirectory=directory,
                modelIdentifier=model.coSimulation.modelIdentifier,
                instanceName=f"reference_{name}",
            )
            slave.instantiate()
            slave.enterInitializationMode(startTime=0.0)
            slave.exitInitializationMode()
            wheels.append(slave)
        try:
            for step, row in enumerate(commands):
                values = {}
                for name, voltage, slave in zip(NAMES, row, wheels):
                    slave.setFloat64([references["voltage_cmd_V"]], [voltage])
                    slave.doStep(
                        currentCommunicationPoint=step * DT,
                        communicationStepSize=DT,
                    )
                    values[name] = tuple(float(value) for value in slave.getFloat64(
                        [references[key] for key in OUTPUTS]
                    ))
                all_samples.append(values)
            for slave in wheels:
                slave.terminate()
        finally:
            for slave in wheels:
                slave.freeInstance()
    return all_samples


def stage_outputs(stage, dictionary, query, tokens: dict[int, str], ordinal: int) -> dict:
    values = {name: {} for name in NAMES}
    with stage.read_attributes(query, list(tokens), ovstage.OrdinalRange.latest(ordinal)) as read:
        read.wait()
        for group in read.groups():
            with group:
                attribute = tokens.get(group.attribute)
                if attribute is None or group.tensor_count != 1 or group.is_array:
                    continue
                paths = dictionary.get_path_strings(group.prim_list)
                column = group.array(0)
                for local_index in range(group.prim_count):
                    path = paths[group.prim_index(local_index)]
                    if path in PATHS:
                        name = path.rsplit("/", 1)[1]
                        values[name][attribute] = float(column[group.data_row_index(local_index)])
    if any(set(value) != set(OUTPUTS) for value in values.values()):
        raise RuntimeError(f"Incomplete USD read at ordinal {ordinal}: {values}")
    return values


def integrate(x: float, y: float, yaw: float, ds: float, dyaw: float) -> tuple[float, float, float]:
    if abs(dyaw) < 1e-10:
        x += ds * math.cos(yaw)
        y += ds * math.sin(yaw)
    else:
        x += ds / dyaw * (math.sin(yaw + dyaw) - math.sin(yaw))
        y += ds / dyaw * (-math.cos(yaw + dyaw) + math.cos(yaw))
    return x, y, yaw + dyaw


def run() -> dict:
    if not FMU.is_file():
        raise FileNotFoundError(FMU)
    dimensions = geometry()
    commands, plan = profile(dimensions)
    baseline = fmpy_reference(commands)
    x, y, yaw = (dimensions[key] for key in ("home_x_m", "home_y_m", "home_yaw_rad"))
    radius = dimensions["wheel_radius_visual_estimate_m"]
    track = dimensions["wheel_track_m"]
    previous_theta = {name: 0.0 for name in NAMES}
    rows = []
    max_fmpy_error = 0.0
    max_side_error = 0.0
    host_log = io.StringIO()

    with ovstage.Stage("molonbot.nvidia.four_wheel_drive") as stage:
        population.open_usd(stage, str(SCENE))
        dictionary = ovstage.PathDictionary(stage)
        path_list = dictionary.create_path_list_from_strings(PATHS)
        query = stage.query_from_path_list(path_list)
        try:
            command_token = dictionary.intern_token("molonbot:voltage_cmd_V")
            output_tokens = {
                dictionary.intern_token("molonbot:" + name): name for name in OUTPUTS
            }
            with FmiHost() as host:
                with contextlib.redirect_stdout(host_log), contextlib.redirect_stderr(host_log):
                    report = host.attach_ovstage(stage, source_asset=SCENE)
                if len(report.instances) != 4:
                    raise RuntimeError(f"Expected four FMU instances: {report.instances}")
                for info in report.instances:
                    if Path(info.source_asset).resolve() != FMU.resolve():
                        raise RuntimeError(f"Unexpected FMU asset: {info.source_asset}")

                for step, voltages in enumerate(commands):
                    input_ordinal = 2 * step + 2
                    output_ordinal = input_ordinal + 1
                    stage.write_attribute(
                        query, command_token, input_ordinal,
                        np.asarray(voltages, dtype=np.float32), is_array=False,
                    ).wait()
                    stage.advance_write_floor(input_ordinal).wait()
                    host.update_from_ovstage(input_ordinal, input_ordinal)
                    with contextlib.redirect_stdout(host_log), contextlib.redirect_stderr(host_log):
                        host.step_sync(DT)
                    if host.write_to_ovstage(output_ordinal) != 3:
                        raise RuntimeError(f"Three output groups were not published at step {step}")
                    stage.advance_write_floor(output_ordinal).wait()
                    current = stage_outputs(stage, dictionary, query, output_tokens, output_ordinal)
                    for name in NAMES:
                        max_fmpy_error = max(
                            max_fmpy_error,
                            *(abs(current[name][key] - value) for key, value in zip(
                                OUTPUTS, baseline[step][name]
                            )),
                        )
                    max_side_error = max(
                        max_side_error,
                        *(abs(current[a][key] - current[b][key]) for a, b in (
                            ("FL", "BL"), ("FR", "BR")
                        ) for key in OUTPUTS),
                    )
                    increments = {
                        name: current[name]["theta_rad"] - previous_theta[name]
                        for name in NAMES
                    }
                    for name in NAMES:
                        previous_theta[name] = current[name]["theta_rad"]
                    left_distance = radius * (increments["FL"] + increments["BL"]) / 2
                    right_distance = radius * (increments["FR"] + increments["BR"]) / 2
                    x, y, yaw = integrate(
                        x, y, yaw,
                        (left_distance + right_distance) / 2,
                        (right_distance - left_distance) / track,
                    )
                    row = {
                        "t_s": f"{(step + 1) * DT:.2f}",
                        "x_m": x,
                        "y_m": y,
                        "yaw_rad": yaw,
                        "left_distance_m": left_distance,
                        "right_distance_m": right_distance,
                        "step_status": "ok",
                    }
                    for name, voltage in zip(NAMES, voltages):
                        row[f"{name}_command_V"] = voltage
                        row[f"{name}_omega_rad_s"] = current[name]["omega_rad_s"]
                        row[f"{name}_theta_rad"] = current[name]["theta_rad"]
                    rows.append(row)
                if abs(host.time - len(commands) * DT) > 1e-10:
                    raise RuntimeError(f"Incorrect ovfmi time: {host.time}")
        finally:
            stage.release_query(query).wait()
            dictionary.destroy_path_list(path_list)
            dictionary.destroy()

    target = dimensions["aisle_heading_rad"]
    yaw_error = (yaw - target + math.pi) % (2 * math.pi) - math.pi
    direction = np.array([dimensions["aisle_unit_x"], dimensions["aisle_unit_y"]])
    displacement = np.array([x - dimensions["home_x_m"], y - dimensions["home_y_m"]])
    along = float(np.dot(displacement, direction))
    across = float(direction[0] * displacement[1] - direction[1] * displacement[0])
    if max_fmpy_error > 5e-6 or max_side_error > 1e-12:
        raise AssertionError(f"FMU/wheel output mismatch: {max_fmpy_error}, {max_side_error}")
    if abs(yaw_error) > 0.02 or abs(along - dimensions["nominal_travel_m"]) > 0.03:
        raise AssertionError(f"Path misses the target: yaw={yaw_error}, travel={along}")
    if abs(across) > 0.08:
        raise AssertionError(f"Excessive lateral deviation: {across}")

    OUT.mkdir(parents=True, exist_ok=True)
    trace = OUT / "trajectory.csv"
    with trace.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    checked = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "inspect_drive_geometry.py"),
         "--trace", str(trace)],
        check=True, text=True, capture_output=True, timeout=30,
    )
    path_validation = json.loads(checked.stdout)
    (OUT / "ovfmi_runtime.log").write_text(host_log.getvalue(), encoding="utf-8")
    results = {
        "status": "passed",
        "scope": "offline four-wheel kinematic replay in a synthetic scene; no physics solver, Nav2, ROS commands or real robot actuation",
        "scene": str(SCENE.relative_to(ROOT)),
        "scene_sha256": sha256(SCENE),
        "fmu_sha256": sha256(FMU),
        "trajectory_sha256": sha256(trace),
        "steps": len(commands),
        "duration_s": len(commands) * DT,
        "dt_s": DT,
        "geometry": dimensions,
        "path_validation": {
            "checked_path": path_validation["checked_path"],
            "checked_path_samples": path_validation["checked_path_samples"],
            "semantic_aabb_center_clearance_m": path_validation["semantic_aabb_center_clearance_m"],
            "semantic_aabb_clearance_after_wheel_footprint_m": path_validation["semantic_aabb_clearance_after_wheel_footprint_m"],
            "nearest_semantic_object": path_validation["nearest_semantic_object"],
        },
        "profile": plan,
        "max_abs_ovfmi_vs_fmpy": max_fmpy_error,
        "max_abs_front_back_same_side": max_side_error,
        "final_x_m": x,
        "final_y_m": y,
        "final_yaw_rad": yaw,
        "final_heading_error_rad": yaw_error,
        "final_along_aisle_m": along,
        "final_cross_aisle_m": across,
        "limitations": [
            "Wheel radius and track are chosen synthetic parameters, not physical measurements.",
            "Differential-drive no-slip assumption; no inertial/contact physics or obstacle avoidance.",
            "Pose is derived from FMU wheel angles; it is not an AMCL or SLAM estimate.",
            "The room and objects are original synthetic USD primitives, not a live or measured reconstruction.",
        ],
    }
    (OUT / "report.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    return results


if __name__ == "__main__":
    report = run()
    print(json.dumps({key: report[key] for key in (
        "status", "steps", "duration_s", "max_abs_ovfmi_vs_fmpy",
        "final_along_aisle_m", "final_heading_error_rad", "trajectory_sha256",
    )}, indent=2))
