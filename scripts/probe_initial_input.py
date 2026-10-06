#!/usr/bin/env python3
"""Reproduce ovfmi 0.2's first-step input behavior without moving the robot."""

import contextlib
import io
import json
import math
from importlib.metadata import version
from pathlib import Path

import numpy as np
import ovstage
from ovfmi import FmiHost
from ovstage import population


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes" / "demo_a_wheel.usda"
STATE = "/World/WheelState"


def main() -> int:
    # USD authors 0 V. We deliberately update ovstage to 6 V before the first
    # step, then compare against the direct FMU's expected exact first step.
    with ovstage.Stage("molonbot.nvidia.first_input_probe") as stage:
        population.open_usd(stage, str(SCENE))
        dictionary = ovstage.PathDictionary(stage)
        path_list = dictionary.create_path_list_from_strings([STATE])
        query = stage.query_from_path_list(path_list)
        try:
            token = dictionary.intern_token("molonbot:voltage_cmd_V")
            with FmiHost() as host:
                with contextlib.redirect_stdout(io.StringIO()):
                    host.attach_ovstage(stage, source_asset=SCENE)
                stage.write_attribute(
                    query, token, 2, np.asarray([6.0], dtype=np.float32),
                    is_array=False,
                ).wait()
                stage.advance_write_floor(2).wait()
                host.update_from_ovstage(2, 2)
                with contextlib.redirect_stdout(io.StringIO()):
                    host.step_sync(0.01)
                with host.read(attribute_names=["molonbot:omega_rad_s"]) as read:
                    observed = float(np.asarray(read.groups[0].tensors[0]).reshape(-1)[0])
        finally:
            stage.release_query(query).wait()
            dictionary.destroy_path_list(path_list)
            dictionary.destroy()
    expected = 12.0 * (1.0 - math.exp(-0.01 / 0.25))
    result = {
        "ovfmi_version": version("ovfmi"),
        "authored_initial_voltage_V": 0.0,
        "ovstage_voltage_before_first_step_V": 6.0,
        "expected_omega_after_first_step_rad_s": expected,
        "observed_omega_after_first_step_rad_s": observed,
        "first_step_update_applied": math.isclose(observed, expected, abs_tol=5e-6),
        "scope": "offline reproducer; no physical robot actuation",
    }
    output = ROOT / "results" / "first_step_probe.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
