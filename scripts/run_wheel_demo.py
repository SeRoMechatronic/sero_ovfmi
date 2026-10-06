#!/usr/bin/env python3
"""Offline USD -> ovstage -> ovfmi -> FMI 3 -> ovstage wheel demo.

The FMU is a didactic first-order wheel model. This program does not import
ROS, Isaac's physics runtime, or any robot actuator interface.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import hashlib
import io
import json
import math
import statistics
import tempfile
import time
from importlib.metadata import version
from pathlib import Path

import fmpy
import numpy as np
import ovstage
from fmpy.fmi3 import FMU3Slave
from fmpy.validation import validate_fmu
from ovfmi import FmiHost
from ovstage import population


ROOT = Path(__file__).resolve().parents[1]
FMU = ROOT / "fmus" / "MolonbotWheelPlant.fmu"
STATE = "/World/WheelState"
PREFIX = "molonbot:"
OUTPUTS = ("voltage_applied_V", "omega_rad_s", "theta_rad")
H = 0.01
N = 500
TOL_F32 = 5e-6


def command(step: int) -> float:
    return 6.0 if 50 <= step < 300 else 0.0


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def analytic_step(omega: float, theta: float, voltage: float) -> tuple[float, float]:
    target = 2.0 * max(-12.0, min(12.0, voltage))
    decay = math.exp(-H / 0.25)
    return (
        target + (omega - target) * decay,
        theta + target * H + (omega - target) * 0.25 * (1.0 - decay),
    )


def run_fmpy() -> tuple[list[tuple[float, float, float]], list[int]]:
    """Use the FMU binary directly, without ovfmi/ovstage."""
    description = fmpy.read_model_description(str(FMU), validate=True)
    if description.fmiVersion != "3.0" or description.coSimulation is None:
        raise RuntimeError("The FMU must be FMI 3.0 Co-Simulation")
    references = {var.name: var.valueReference for var in description.modelVariables}
    samples: list[tuple[float, float, float]] = []
    times: list[int] = []
    with tempfile.TemporaryDirectory(prefix="molonbot_fmpy_") as directory:
        fmpy.extract(str(FMU), unzipdir=directory)
        slave = FMU3Slave(
            guid=description.guid,
            unzipDirectory=directory,
            modelIdentifier=description.coSimulation.modelIdentifier,
            instanceName="independent_reference",
        )
        slave.instantiate()
        try:
            slave.enterInitializationMode(startTime=0.0)
            slave.exitInitializationMode()
            for step in range(N):
                slave.setFloat64([references["voltage_cmd_V"]], [command(step)])
                start = time.perf_counter_ns()
                slave.doStep(currentCommunicationPoint=step * H, communicationStepSize=H)
                times.append(time.perf_counter_ns() - start)
                samples.append(
                    tuple(float(value) for value in slave.getFloat64(
                        [references[name] for name in OUTPUTS]
                    ))
                )
            slave.terminate()
        finally:
            slave.freeInstance()
    return samples, times


def stage_values(stage, query, tokens: dict[int, str], ordinal: int) -> dict[str, float]:
    result: dict[str, float] = {}
    with stage.read_attributes(
        query, list(tokens), ovstage.OrdinalRange.latest(ordinal)
    ) as read:
        read.wait()
        for group in read.groups():
            with group:
                name = tokens.get(group.attribute)
                if name is not None and group.tensor_count == 1 and group.prim_count == 1:
                    result[name] = float(group.array(0)[group.data_row_index(0)])
    if set(result) != set(tokens.values()):
        raise RuntimeError(f"Incomplete ovstage read at ordinal {ordinal}: {result}")
    return result


def host_values(host: FmiHost) -> dict[str, float]:
    result: dict[str, float] = {}
    with host.read(prim_paths=[STATE]) as read:
        for group in read.groups:
            if len(group.prim_paths) != 1 or group.prim_paths[0] != STATE:
                raise RuntimeError(f"Unexpected FMI target: {group.prim_paths}")
            result[group.attribute_name] = float(np.asarray(group.tensors[0]).reshape(-1)[0])
    expected = {PREFIX + name for name in OUTPUTS}
    if set(result) != expected:
        raise RuntimeError(f"Incomplete ovfmi outputs: {result}")
    return result


def percentile95(samples: list[int]) -> float:
    sorted_samples = sorted(samples)
    return sorted_samples[math.ceil(0.95 * len(sorted_samples)) - 1] / 1e6


def run(scene: Path, output: Path) -> dict:
    if not FMU.is_file():
        raise FileNotFoundError(f"Missing FMU: {FMU}")
    if not scene.is_file():
        raise FileNotFoundError(scene)
    issues = validate_fmu(str(FMU))
    if issues:
        raise RuntimeError(f"FMU validation failed: {issues}")

    baseline, baseline_times = run_fmpy()
    rows: list[dict] = []
    step_times: list[int] = []
    fmi_log = io.StringIO()
    analytic_omega = analytic_theta = 0.0
    max_host_baseline = max_stage_host = max_analytic_baseline = 0.0
    max_applied = 0.0
    attach_ms = 0.0

    with ovstage.Stage("molonbot.nvidia.wheel_demo") as stage:
        population.open_usd(stage, str(scene))
        dictionary = ovstage.PathDictionary(stage)
        path_list = dictionary.create_path_list_from_strings([STATE])
        query = stage.query_from_path_list(path_list)
        try:
            input_token = dictionary.intern_token(PREFIX + "voltage_cmd_V")
            output_tokens = {
                dictionary.intern_token(PREFIX + name): PREFIX + name
                for name in OUTPUTS
            }
            with FmiHost() as host:
                start = time.perf_counter_ns()
                with contextlib.redirect_stdout(fmi_log), contextlib.redirect_stderr(fmi_log):
                    report = host.attach_ovstage(stage, source_asset=scene)
                attach_ms = (time.perf_counter_ns() - start) / 1e6
                if len(report.instances) != 1 or report.instances[0].kind != "fmu":
                    raise RuntimeError(f"Expected one FMU: {report.instances}")
                if Path(report.instances[0].source_asset).resolve() != FMU.resolve():
                    raise RuntimeError("The scene references a different FMU")

                for step in range(N):
                    voltage = command(step)
                    input_ordinal = 2 * step + 2
                    output_ordinal = input_ordinal + 1
                    stage.write_attribute(
                        query, input_token, input_ordinal,
                        np.asarray([voltage], dtype=np.float32), is_array=False,
                    ).wait()
                    stage.advance_write_floor(input_ordinal).wait()
                    host.update_from_ovstage(input_ordinal, input_ordinal)

                    start = time.perf_counter_ns()
                    with contextlib.redirect_stdout(fmi_log), contextlib.redirect_stderr(fmi_log):
                        host.step_sync(H)
                    step_times.append(time.perf_counter_ns() - start)
                    if host.write_to_ovstage(output_ordinal) != 3:
                        raise RuntimeError("ovfmi did not publish all three outputs")
                    stage.advance_write_floor(output_ordinal).wait()

                    from_host = host_values(host)
                    from_stage = stage_values(stage, query, output_tokens, output_ordinal)
                    direct = baseline[step]
                    analytic_omega, analytic_theta = analytic_step(
                        analytic_omega, analytic_theta, voltage
                    )
                    mapped = [from_host[PREFIX + name] for name in OUTPUTS]
                    published = [from_stage[PREFIX + name] for name in OUTPUTS]
                    max_host_baseline = max(
                        max_host_baseline,
                        *(abs(a - b) for a, b in zip(mapped, direct)),
                    )
                    max_stage_host = max(
                        max_stage_host,
                        *(abs(a - b) for a, b in zip(published, mapped)),
                    )
                    max_analytic_baseline = max(
                        max_analytic_baseline,
                        abs(direct[1] - analytic_omega),
                        abs(direct[2] - analytic_theta),
                    )
                    max_applied = max(max_applied, abs(mapped[0] - voltage))
                    rows.append({
                        "t_s": f"{(step + 1) * H:.2f}",
                        "voltage_cmd_V": voltage,
                        "voltage_applied_ovfmi_V": mapped[0],
                        "omega_ovfmi_rad_s": mapped[1],
                        "theta_ovfmi_rad": mapped[2],
                        "voltage_applied_ovstage_V": published[0],
                        "omega_ovstage_rad_s": published[1],
                        "theta_ovstage_rad": published[2],
                        "omega_fmpy_rad_s": direct[1],
                        "theta_fmpy_rad": direct[2],
                        "omega_analytic_rad_s": analytic_omega,
                        "theta_analytic_rad": analytic_theta,
                        "step_status": "ok",
                    })
                if not math.isclose(host.time, 5.0, abs_tol=1e-12):
                    raise RuntimeError(f"Unexpected ovfmi time: {host.time}")
        finally:
            stage.release_query(query).wait()
            dictionary.destroy_path_list(path_list)
            dictionary.destroy()

    if max_host_baseline > TOL_F32 or max_stage_host > 1e-12:
        raise AssertionError(
            f"Mismatch: ovfmi-FMPy={max_host_baseline}, ovstage-ovfmi={max_stage_host}"
        )
    if max_analytic_baseline > 1e-10 or max_applied > 1e-12:
        raise AssertionError(
            f"Unexpected analytic reference or saturation: {max_analytic_baseline}, {max_applied}"
        )
    if abs(float(rows[74]["omega_fmpy_rad_s"]) - 7.58544670594) > 1e-8:
        raise AssertionError("Checkpoint t=0.75 s failed")
    if abs(float(rows[324]["omega_fmpy_rad_s"]) - 4.41435287365) > 1e-8:
        raise AssertionError("Checkpoint t=3.25 s failed")

    output.mkdir(parents=True, exist_ok=True)
    csv_path = output / "trace.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    (output / "ovfmi_runtime.log").write_text(fmi_log.getvalue(), encoding="utf-8")
    report = {
        "status": "passed",
        "scope": "offline; synthetic voltage command; no physical robot control",
        "scene": str(scene.relative_to(ROOT)) if scene.is_relative_to(ROOT) else str(scene),
        "scene_sha256": sha256(scene),
        "fmu": str(FMU.relative_to(ROOT)),
        "fmu_sha256": sha256(FMU),
        "versions": {
            "ovfmi": version("ovfmi"),
            "ovstage": ovstage.__version__,
            "fmpy": fmpy.__version__,
        },
        "steps": N,
        "step_s": H,
        "trace_sha256": sha256(csv_path),
        "max_abs_ovfmi_vs_fmpy": max_host_baseline,
        "max_abs_ovstage_vs_ovfmi": max_stage_host,
        "max_abs_fmpy_vs_analytic": max_analytic_baseline,
        "max_abs_applied_vs_command_V": max_applied,
        "omega_0_75_fmpy_rad_s": baseline[74][1],
        "omega_3_25_fmpy_rad_s": baseline[324][1],
        "attach_ms": attach_ms,
        "step_sync_median_ms": statistics.median(step_times) / 1e6,
        "step_sync_p95_ms": percentile95(step_times),
        "fmpy_dostep_median_ms": statistics.median(baseline_times) / 1e6,
        "fmpy_dostep_p95_ms": percentile95(baseline_times),
        "timing_note": "Exploratory wall time in one process; not a controlled benchmark.",
    }
    (output / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene", type=Path, default=ROOT / "scenes" / "demo_a_wheel.usda"
    )
    parser.add_argument(
        "--output", type=Path, default=ROOT / "results" / "demo_a"
    )
    args = parser.parse_args()
    report = run(args.scene.resolve(), args.output.resolve())
    print(json.dumps({
        "status": report["status"],
        "steps": report["steps"],
        "max_abs_ovfmi_vs_fmpy": report["max_abs_ovfmi_vs_fmpy"],
        "max_abs_ovstage_vs_ovfmi": report["max_abs_ovstage_vs_ovfmi"],
        "trace_sha256": report["trace_sha256"],
        "output": str(args.output.resolve()),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
