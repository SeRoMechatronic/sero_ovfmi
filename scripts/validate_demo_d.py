#!/usr/bin/env python3
"""Independently replay a live Isaac trace through FMPy and check causality.

This runs only against recorded simulated signals. It never opens Isaac, ROS,
or the physical robot. A pass proves FMU/ovfmi signal agreement, not that the
synthetic motor parameters identify real hardware.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import tempfile
from pathlib import Path

import fmpy
import numpy as np
from fmpy.fmi3 import FMU3Slave


ROOT = Path(__file__).resolve().parents[1]
WHEELS = ("FL", "FR", "BL", "BR")
H = 0.01
CONTROLLER = ROOT / "fmus/MolonbotWheelPIController.fmu"
PLANT = ROOT / "fmus/MolonbotWheelPlant.fmu"
TOL = 1e-4  # ovstage's USD attributes are Float32, FMU variables are Float64.
C_OUTPUTS = ("voltage_cmd_V", "error_rad_s", "integral_V")
P_OUTPUTS = ("voltage_applied_V", "omega_rad_s", "theta_rad")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def open_fmu(path: Path, extracted: str, name: str):
    description = fmpy.read_model_description(str(path), validate=True)
    instance = FMU3Slave(
        guid=description.guid, unzipDirectory=extracted,
        modelIdentifier=description.coSimulation.modelIdentifier,
        instanceName=name,
    )
    instance.instantiate()
    instance.enterInitializationMode(startTime=0.0)
    instance.exitInitializationMode()
    refs = {variable.name: variable.valueReference for variable in description.modelVariables}
    return instance, refs


def close_fmu(instance: FMU3Slave) -> None:
    try:
        instance.terminate()
    finally:
        instance.freeInstance()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    trace = args.trace.resolve()
    report_path = trace.with_name("report.json")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    motor = report["synthetic_motor"]
    gain = float(motor["torque_gain_Nm_per_V"])
    back_emf = float(motor["back_emf_V_per_rad_s"])
    torque_limit = float(motor["torque_limit_Nm"])
    with trace.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or len(rows) % 4:
        raise AssertionError("Expected a non-empty trace with exactly four rows per step")
    steps = len(rows) // 4
    max_error = 0.0
    max_torque_error = 0.0
    max_continuity_error = 0.0
    max_abs_speed = 0.0
    max_abs_voltage = 0.0
    saturated_samples = 0
    previous_measured = {wheel: 0.0 for wheel in WHEELS}
    previous_command = {wheel: 0.0 for wheel in WHEELS}
    instances = []
    with tempfile.TemporaryDirectory(prefix="sero_demo_d_fmpy_") as directory:
        controller_dir = str(Path(directory) / "controller")
        plant_dir = str(Path(directory) / "plant")
        fmpy.extract(str(CONTROLLER), unzipdir=controller_dir)
        fmpy.extract(str(PLANT), unzipdir=plant_dir)
        models = {}
        try:
            for wheel in WHEELS:
                controller, c_refs = open_fmu(CONTROLLER, controller_dir,
                                              f"controller_replay_{wheel}")
                plant, p_refs = open_fmu(PLANT, plant_dir, f"plant_replay_{wheel}")
                instances.extend((controller, plant))
                models[wheel] = (controller, c_refs, plant, p_refs)
            for step in range(steps):
                group = rows[4 * step:4 * (step + 1)]
                if [row["wheel"] for row in group] != list(WHEELS):
                    raise AssertionError(f"Wheel row order mismatch at step {step}")
                for row in group:
                    wheel = row["wheel"]
                    if int(row["step"]) != step + 1 or not math.isclose(
                            float(row["t_s"]), (step + 1) * H, abs_tol=1e-10):
                        raise AssertionError(f"Time/step mismatch for {wheel} at {step}")
                    ref = float(row["reference_rad_s"])
                    measured = float(row["measured_before_rad_s"])
                    applied = float(row["voltage_applied_to_physx_V"])
                    command = float(row["voltage_cmd_V"])
                    after = float(row["measured_after_rad_s"])
                    if not all(map(math.isfinite, (ref, measured, applied, command, after))):
                        raise AssertionError(f"Non-finite signal at step {step}")
                    max_continuity_error = max(max_continuity_error,
                                               abs(measured - previous_measured[wheel]),
                                               abs(applied - previous_command[wheel]))
                    expected_torque = np.clip(gain * (applied - back_emf * measured),
                                              -torque_limit, torque_limit)
                    max_torque_error = max(max_torque_error,
                                           abs(float(row["torque_Nm"]) - expected_torque))
                    controller, c_refs, plant, p_refs = models[wheel]
                    # ovstage writes scalar Float attributes, therefore its FMU inputs
                    # are quantized to Float32 even though the FMUs use Float64.
                    controller.setFloat64(
                        [c_refs["omega_ref_rad_s"], c_refs["omega_meas_rad_s"]],
                        [float(np.float32(ref)), float(np.float32(measured))],
                    )
                    plant.setFloat64([p_refs["voltage_cmd_V"]],
                                     [float(np.float32(applied))])
                    controller.doStep(currentCommunicationPoint=step * H,
                                      communicationStepSize=H)
                    plant.doStep(currentCommunicationPoint=step * H,
                                 communicationStepSize=H)
                    for name, expected in zip(C_OUTPUTS, controller.getFloat64(
                            [c_refs[name] for name in C_OUTPUTS])):
                        max_error = max(max_error, abs(float(row[name]) - expected))
                    for name, expected in zip(P_OUTPUTS, plant.getFloat64(
                            [p_refs[name] for name in P_OUTPUTS])):
                        max_error = max(max_error, abs(float(row["shadow_" + name]) - expected))
                    max_abs_speed = max(max_abs_speed, abs(after))
                    max_abs_voltage = max(max_abs_voltage, abs(command))
                    saturated_samples += abs(command) >= 11.999
                    previous_measured[wheel] = after
                    previous_command[wheel] = command
        finally:
            for instance in reversed(instances):
                close_fmu(instance)
    if max_error > TOL:
        raise AssertionError(f"Direct FMPy/ovfmi disagreement {max_error} > {TOL}")
    if max_continuity_error > 1e-5 or max_torque_error > 1e-9:
        raise AssertionError("The logged one-step delay or motor equation is inconsistent")
    if max_abs_voltage > 12.00001:
        raise AssertionError("Controller voltage exceeded FMU limits")
    result = {
        "status": "passed", "steps": steps, "wheel_samples": len(rows),
        "fmu_instances_replayed": 8, "trace_sha256": digest(trace),
        "controller_fmu_sha256": digest(CONTROLLER),
        "plant_fmu_sha256": digest(PLANT),
        "comparison_tolerance": TOL,
        "maximum_abs_fmpy_ovfmi_signal_difference": max_error,
        "maximum_abs_step_continuity_difference": max_continuity_error,
        "maximum_abs_motor_equation_difference_Nm": max_torque_error,
        "maximum_abs_physx_wheel_speed_rad_s": max_abs_speed,
        "maximum_abs_controller_voltage_V": max_abs_voltage,
        "saturated_wheel_samples": saturated_samples,
        "scope": "Numerical FMU/ovfmi replay of live Isaac measurements; not a hardware validation",
    }
    output = args.output.resolve() if args.output else trace.with_name("replay_validation.json")
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
