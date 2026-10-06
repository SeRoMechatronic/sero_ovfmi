#!/usr/bin/env python3
"""Run Demo C's offline PI-controller/plant FMU loop through ovfmi and FMPy.

The two FMUs are stepped with an explicit one-step communication delay. This
script never imports ROS or sends commands to robot hardware.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import hashlib
import importlib.metadata
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
from fmpy.validation import validate_fmu
from ovfmi import FmiHost
from ovstage import population


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes/demo_C_closed_loop.usda"
CONTROLLER = ROOT / "fmus/MolonbotWheelPIController.fmu"
PLANT = ROOT / "fmus/MolonbotWheelPlant.fmu"
OUTPUT = ROOT / "results/demo_C"
C_STATE = "/World/ControllerState"
P_STATE = "/World/PlantState"
PREFIX = "molonbot:"
H = 0.01
N = 1000
TOLERANCE = 5e-5
C_OUTPUTS = ("voltage_cmd_V", "error_rad_s", "integral_V")
P_OUTPUTS = ("voltage_applied_V", "omega_rad_s", "theta_rad")
SIGNALS = (*C_OUTPUTS, *P_OUTPUTS)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def reference(step: int) -> float:
    if step < 20:
        return 0.0
    if step < 320:
        return 8.0
    if step < 420:
        return 0.0
    if step < 620:
        return -6.0
    if step < 720:
        return 30.0
    return 0.0


def pi_reference(integral: float, ref: float, measured: float) -> tuple[float, float, float]:
    error = ref - measured
    raw = 0.4 * error + integral
    command = max(-12.0, min(12.0, raw))
    integral_next = max(-12.0, min(12.0,
        integral + 1.2 * H * error + 4.0 * H * (command - raw)))
    return command, error, integral_next


def plant_reference(speed: float, angle: float, voltage: float) -> tuple[float, float, float]:
    applied = max(-12.0, min(12.0, voltage))
    target = 2.0 * applied
    decay = math.exp(-H / 0.25)
    return (target + (speed - target) * decay,
            angle + target * H + (speed - target) * 0.25 * (1.0 - decay),
            applied)


def initial_row() -> dict[str, float | int | str]:
    return {
        "step": 0, "t_s": 0.0, "reference_rad_s": 0.0,
        "controller_input_meas_rad_s": 0.0, "plant_input_voltage_V": 0.0,
        "voltage_cmd_V": 0.0, "error_rad_s": 0.0, "integral_V": 0.0,
        "voltage_applied_V": 0.0, "omega_rad_s": 0.0, "theta_rad": 0.0,
        "step_status": "init",
    }


def run_analytic() -> list[dict]:
    rows = [initial_row()]
    speed = angle = command = integral = 0.0
    for step in range(N):
        ref = reference(step)
        measured_input = speed
        plant_input = command
        command_next, error, integral = pi_reference(integral, ref, measured_input)
        speed, angle, applied = plant_reference(speed, angle, plant_input)
        command = command_next
        rows.append({
            "step": step + 1, "t_s": (step + 1) * H,
            "reference_rad_s": ref,
            "controller_input_meas_rad_s": measured_input,
            "plant_input_voltage_V": plant_input,
            "voltage_cmd_V": command, "error_rad_s": error, "integral_V": integral,
            "voltage_applied_V": applied, "omega_rad_s": speed, "theta_rad": angle,
            "step_status": "ok",
        })
    return rows


def open_fmu(path: Path, directory: str, instance_name: str) -> tuple[FMU3Slave, dict]:
    description = fmpy.read_model_description(str(path), validate=True)
    slave = FMU3Slave(
        guid=description.guid, unzipDirectory=directory,
        modelIdentifier=description.coSimulation.modelIdentifier,
        instanceName=instance_name,
    )
    slave.instantiate()
    slave.enterInitializationMode(startTime=0.0)
    slave.exitInitializationMode()
    refs = {variable.name: variable.valueReference for variable in description.modelVariables}
    return slave, refs


def run_direct_fmpy() -> list[dict]:
    rows = [initial_row()]
    with tempfile.TemporaryDirectory(prefix="sero_demo_c_direct_") as directory:
        controller_dir = str(Path(directory) / "controller")
        plant_dir = str(Path(directory) / "plant")
        fmpy.extract(str(CONTROLLER), unzipdir=controller_dir)
        fmpy.extract(str(PLANT), unzipdir=plant_dir)
        controller, c_refs = open_fmu(CONTROLLER, controller_dir, "controller_reference")
        plant, p_refs = open_fmu(PLANT, plant_dir, "plant_reference")
        try:
            c_initial = controller.getFloat64([c_refs[name] for name in C_OUTPUTS])
            p_initial = plant.getFloat64([p_refs[name] for name in P_OUTPUTS])
            if c_initial != [0.0] * 3 or p_initial != [0.0] * 3:
                raise AssertionError("FMU outputs must start at zero")
            speed_previous = command_previous = 0.0
            for step in range(N):
                ref = reference(step)
                controller.setFloat64(
                    [c_refs["omega_ref_rad_s"], c_refs["omega_meas_rad_s"]],
                    [ref, speed_previous],
                )
                plant.setFloat64([p_refs["voltage_cmd_V"]], [command_previous])
                controller.doStep(currentCommunicationPoint=step * H, communicationStepSize=H)
                plant.doStep(currentCommunicationPoint=step * H, communicationStepSize=H)
                c_values = controller.getFloat64([c_refs[name] for name in C_OUTPUTS])
                p_values = plant.getFloat64([p_refs[name] for name in P_OUTPUTS])
                row = {
                    "step": step + 1, "t_s": (step + 1) * H,
                    "reference_rad_s": ref,
                    "controller_input_meas_rad_s": speed_previous,
                    "plant_input_voltage_V": command_previous,
                    **dict(zip(C_OUTPUTS, c_values)),
                    **dict(zip(P_OUTPUTS, p_values)),
                    "step_status": "ok",
                }
                rows.append(row)
                command_previous = row["voltage_cmd_V"]
                speed_previous = row["omega_rad_s"]
            controller.terminate()
            plant.terminate()
        finally:
            controller.freeInstance()
            plant.freeInstance()
    return rows


def read_stage(stage, query, dictionary, names: tuple[str, ...], ordinal: int) -> dict[str, float]:
    tokens = {dictionary.intern_token(PREFIX + name): name for name in names}
    values: dict[str, float] = {}
    with stage.read_attributes(query, list(tokens), ovstage.OrdinalRange.latest(ordinal)) as read:
        read.wait()
        for group in read.groups():
            with group:
                name = tokens.get(group.attribute)
                if name is not None and group.tensor_count == 1 and group.prim_count == 1:
                    values[name] = float(group.array(0)[group.data_row_index(0)])
    if set(values) != set(names):
        raise RuntimeError(f"Incomplete ovstage outputs at ordinal {ordinal}: {values}")
    return values


def read_host(host: FmiHost, path: str, names: tuple[str, ...]) -> dict[str, float]:
    values: dict[str, float] = {}
    with host.read(prim_paths=[path]) as read:
        for group in read.groups:
            if len(group.prim_paths) != 1 or group.prim_paths[0] != path:
                raise RuntimeError(f"Unexpected FMU target: {group.prim_paths}")
            values[group.attribute_name.removeprefix(PREFIX)] = float(
                np.asarray(group.tensors[0]).reshape(-1)[0]
            )
    if set(values) != set(names):
        raise RuntimeError(f"Incomplete ovfmi outputs for {path}: {values}")
    return values


def run_ovfmi() -> tuple[list[dict], float, str]:
    rows = [initial_row()]
    log = io.StringIO()
    max_stage_host = 0.0
    with ovstage.Stage("sero.demo_c.closed_loop") as stage:
        population.open_usd(stage, str(SCENE))
        dictionary = ovstage.PathDictionary(stage)
        controller_paths = dictionary.create_path_list_from_strings([C_STATE])
        plant_paths = dictionary.create_path_list_from_strings([P_STATE])
        controller_query = stage.query_from_path_list(controller_paths)
        plant_query = stage.query_from_path_list(plant_paths)
        try:
            input_tokens = {name: dictionary.intern_token(PREFIX + name) for name in (
                "omega_ref_rad_s", "omega_meas_rad_s", "voltage_cmd_V"
            )}
            with FmiHost() as host:
                with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
                    attached = host.attach_ovstage(stage, source_asset=SCENE)
                assets = {Path(info.source_asset).resolve() for info in attached.instances}
                if len(attached.instances) != 2 or assets != {CONTROLLER.resolve(), PLANT.resolve()}:
                    raise RuntimeError(f"Wrong FMUs attached: {attached.instances}")
                command_previous = speed_previous = 0.0
                for step in range(N):
                    ref = reference(step)
                    input_ordinal = 2 * step + 2
                    output_ordinal = input_ordinal + 1
                    for query, name, value in (
                        (controller_query, "omega_ref_rad_s", ref),
                        (controller_query, "omega_meas_rad_s", speed_previous),
                        (plant_query, "voltage_cmd_V", command_previous),
                    ):
                        stage.write_attribute(
                            query, input_tokens[name], input_ordinal,
                            np.asarray([value], dtype=np.float32), is_array=False,
                        ).wait()
                    stage.advance_write_floor(input_ordinal).wait()
                    host.update_from_ovstage(input_ordinal, input_ordinal)
                    with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
                        host.step_sync(H)
                    if host.write_to_ovstage(output_ordinal) != len(SIGNALS):
                        raise RuntimeError(f"Missing FMU output groups at step {step}")
                    stage.advance_write_floor(output_ordinal).wait()
                    c_stage = read_stage(stage, controller_query, dictionary, C_OUTPUTS, output_ordinal)
                    p_stage = read_stage(stage, plant_query, dictionary, P_OUTPUTS, output_ordinal)
                    c_host = read_host(host, C_STATE, C_OUTPUTS)
                    p_host = read_host(host, P_STATE, P_OUTPUTS)
                    for name in C_OUTPUTS:
                        max_stage_host = max(max_stage_host, abs(c_stage[name] - c_host[name]))
                    for name in P_OUTPUTS:
                        max_stage_host = max(max_stage_host, abs(p_stage[name] - p_host[name]))
                    row = {
                        "step": step + 1, "t_s": (step + 1) * H,
                        "reference_rad_s": ref,
                        "controller_input_meas_rad_s": speed_previous,
                        "plant_input_voltage_V": command_previous,
                        **c_stage, **p_stage, "step_status": "ok",
                    }
                    rows.append(row)
                    command_previous = c_stage["voltage_cmd_V"]
                    speed_previous = p_stage["omega_rad_s"]
                if not math.isclose(host.time, N * H, abs_tol=1e-10):
                    raise RuntimeError(f"Wrong ovfmi final time: {host.time}")
        finally:
            stage.release_query(controller_query).wait()
            stage.release_query(plant_query).wait()
            dictionary.destroy_path_list(controller_paths)
            dictionary.destroy_path_list(plant_paths)
            dictionary.destroy()
    return rows, max_stage_host, log.getvalue()


def metrics(rows: list[dict]) -> dict:
    segments = ((20, 320, 8.0, 0.0), (320, 420, 0.0, 8.0),
                (420, 620, -6.0, 0.0), (620, 720, 30.0, -6.0),
                (720, N, 0.0, 30.0))
    segment_results = []
    for start, end, target, previous_target in segments:
        final_speed = rows[end]["omega_rad_s"]
        saturation_steps = sum(abs(rows[index]["voltage_cmd_V"]) >= 12.0
                               for index in range(start + 1, end + 1))
        step_size = target - previous_target
        result = {
            "interval_s": [start * H, end * H],
            "reference_rad_s": target,
            "reference_step_rad_s": step_size,
            "final_error_rad_s": target - final_speed,
            "saturated_time_s": saturation_steps * H,
        }
        if step_size:
            observed = [rows[index]["omega_rad_s"] for index in range(start + 1, end + 1)]
            overshoot = (max(observed) - target) if step_size > 0 else (target - min(observed))
            overshoot = max(0.0, overshoot)
            band = 0.02 * abs(step_size)
            last_outside = max((index for index in range(start + 1, end + 1)
                                if abs(rows[index]["omega_rad_s"] - target) > band),
                               default=start)
            result.update({
                "overshoot_rad_s": overshoot,
                "overshoot_pct_of_step": overshoot / abs(step_size) * 100.0,
                "settling_time_2pct_s": (
                    None if last_outside == end else round((last_outside + 1 - start) * H, 10)
                ),
            })
        segment_results.append(result)
    recovery = next((index for index in range(721, N + 1)
                     if abs(rows[index]["voltage_cmd_V"]) < 12.0), None)
    return {
        "segments": segment_results,
        "saturation_recovery_after_7_2_s": (
            None if recovery is None else round((recovery - 720) * H, 10)
        ),
        "max_abs_command_V": max(abs(row["voltage_cmd_V"]) for row in rows),
        "max_abs_integral_V": max(abs(row["integral_V"]) for row in rows),
        "command_sign_changes": sum(
            rows[index - 1]["voltage_cmd_V"] * rows[index]["voltage_cmd_V"] < 0
            for index in range(1, len(rows))
        ),
    }


def run(output: Path) -> dict:
    for path in (CONTROLLER, PLANT, SCENE):
        if not path.is_file():
            raise FileNotFoundError(path)
    preflight = subprocess.run(
        [sys.executable, str(ROOT / "scripts/validate_demo_c_scene.py"),
         "--scene", str(SCENE)], capture_output=True, text=True, check=False,
    )
    if preflight.returncode:
        raise RuntimeError(f"Demo C USD/FMU mapping preflight failed: {preflight.stderr.strip()}")
    for fmu in (CONTROLLER, PLANT):
        issues = validate_fmu(str(fmu))
        if issues:
            raise RuntimeError(f"Invalid FMU {fmu.name}: {issues}")
    direct = run_direct_fmpy()
    analytic = run_analytic()
    ovfmi_rows, max_stage_host, host_log = run_ovfmi()
    second_rows, _, _ = run_ovfmi()
    if ovfmi_rows != second_rows:
        raise AssertionError("The ovfmi run is not repeatable")
    max_direct_analytic = max(
        abs(a[name] - b[name]) for a, b in zip(direct, analytic) for name in SIGNALS
    )
    max_ovfmi_direct = {
        name: max(abs(a[name] - b[name]) for a, b in zip(ovfmi_rows, direct))
        for name in SIGNALS
    }
    if max_direct_analytic > 1e-10:
        raise AssertionError(f"Direct FMPy differs from analytic reference: {max_direct_analytic}")
    if max(max_ovfmi_direct.values()) > TOLERANCE:
        raise AssertionError(f"ovfmi/direct FMPy difference exceeds {TOLERANCE}: {max_ovfmi_direct}")
    if max_stage_host > 1e-12:
        raise AssertionError(f"ovstage/ovfmi publication mismatch: {max_stage_host}")
    if not all(math.isfinite(float(row[name])) for row in ovfmi_rows for name in SIGNALS):
        raise AssertionError("Non-finite FMU output")
    if not all(abs(row["voltage_cmd_V"]) <= 12.0 for row in ovfmi_rows):
        raise AssertionError("Controller voltage limit exceeded")
    if not all(abs(row["voltage_applied_V"] - row["plant_input_voltage_V"]) <= 1e-6
               for row in ovfmi_rows[1:]):
        raise AssertionError("Plant input is not the prior controller command")
    if not all(abs(row["error_rad_s"] - (row["reference_rad_s"]
               - row["controller_input_meas_rad_s"])) <= 5e-6
               for row in ovfmi_rows[1:]):
        raise AssertionError("Controller error does not match the published inputs")

    output.mkdir(parents=True, exist_ok=True)
    trace = output / "trace.csv"
    fieldnames = ["step", "t_s", "reference_rad_s", "controller_input_meas_rad_s",
                  "plant_input_voltage_V"]
    for name in SIGNALS:
        fieldnames += [f"{name}_ovstage", f"{name}_fmpy"]
    fieldnames.append("step_status")
    with trace.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for stage_row, direct_row in zip(ovfmi_rows, direct):
            row = {name: stage_row[name] for name in fieldnames[:5]}
            for name in SIGNALS:
                row[f"{name}_ovstage"] = stage_row[name]
                row[f"{name}_fmpy"] = direct_row[name]
            row["step_status"] = stage_row["step_status"]
            writer.writerow(row)
    (output / "ovfmi_runtime.log").write_text(host_log, encoding="utf-8")
    report = {
        "status": "passed",
        "scope": "offline two-FMU closed loop; no ROS or physical robot control",
        "schedule": "Jacobi with one-step communication delay; controller and plant receive previous published peer outputs",
        "scene": str(SCENE.relative_to(ROOT)),
        "scene_sha256": sha256(SCENE),
        "controller_fmu_sha256": sha256(CONTROLLER),
        "plant_fmu_sha256": sha256(PLANT),
        "usd_fmu_mapping_preflight": "passed",
        "versions": {
            "fmpy": fmpy.__version__, "ovstage": ovstage.__version__,
            "ovfmi": importlib.metadata.version("ovfmi"),
        },
        "steps": N, "h_s": H,
        "trace_sha256": sha256(trace),
        "max_abs_ovstage_vs_ovfmi": max_stage_host,
        "max_abs_direct_fmpy_vs_analytic": max_direct_analytic,
        "max_abs_ovfmi_vs_direct_fmpy_by_signal": max_ovfmi_direct,
        "declared_ovfmi_vs_fmpy_tolerance": TOLERANCE,
        "ovfmi_repeatable": True,
        "metrics": metrics(ovfmi_rows),
        "limitations": [
            "The plant is a didactic first-order model, not a calibrated physical motor.",
            "USD/ovstage ordinary signals are observed as float32 in ovfmi 0.2; direct FMPy uses FMU Float64 values.",
            "No hardware actuation, contact physics, Nav2, or zero-delay feedback is claimed.",
        ],
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    report = run(args.output.resolve())
    print(json.dumps({key: report[key] for key in (
        "status", "steps", "max_abs_ovstage_vs_ovfmi",
        "max_abs_direct_fmpy_vs_analytic", "max_abs_ovfmi_vs_direct_fmpy_by_signal",
        "trace_sha256",
    )}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
