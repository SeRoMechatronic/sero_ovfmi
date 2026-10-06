#!/usr/bin/env python3
"""ovfmi side of Demo D's strictly local, synchronous Isaac/PhysX bridge.

Protocol: one JSON line in/out per 10 ms simulation step on stdio. No ROS,
network socket, or physical motor interface is imported or opened here.
"""

from __future__ import annotations

import contextlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import ovstage
from ovfmi import FmiHost
from ovstage import population


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes/demo_D_live_fmi_contract.usda"
WHEELS = ("FL", "FR", "BL", "BR")
H = 0.01
PREFIX = "molonbot:"
CONTROLLER_OUTPUTS = ("voltage_cmd_V", "error_rad_s", "integral_V")
PLANT_OUTPUTS = ("voltage_applied_V", "omega_rad_s", "theta_rad")


def read_state(stage, query, dictionary, names: tuple[str, ...], ordinal: int) -> dict:
    token_to_name = {dictionary.intern_token(PREFIX + name): name for name in names}
    values = {}
    with stage.read_attributes(query, list(token_to_name),
                               ovstage.OrdinalRange.latest(ordinal)) as read:
        read.wait()
        for group in read.groups():
            with group:
                name = token_to_name.get(group.attribute)
                if name is not None and group.tensor_count == 1 and group.prim_count == 1:
                    values[name] = float(group.array(0)[group.data_row_index(0)])
    if set(values) != set(names):
        raise RuntimeError(f"Incomplete ovstage read at {ordinal}: {values}")
    return values


class Master:
    def __init__(self):
        self.stage = ovstage.Stage("sero.demo_d.live_physx")
        population.open_usd(self.stage, str(SCENE))
        self.dictionary = ovstage.PathDictionary(self.stage)
        self.paths = {}
        self.queries = {}
        for kind in ("Controller", "ShadowPlant"):
            for wheel in WHEELS:
                key = (kind, wheel)
                path = f"/World/{kind}States/{wheel}"
                self.paths[key] = self.dictionary.create_path_list_from_strings([path])
                self.queries[key] = self.stage.query_from_path_list(self.paths[key])
        self.input_tokens = {name: self.dictionary.intern_token(PREFIX + name)
                             for name in ("omega_ref_rad_s", "omega_meas_rad_s",
                                          "voltage_cmd_V")}
        self.host = FmiHost()
        with contextlib.redirect_stdout(sys.stderr):
            attached = self.host.attach_ovstage(self.stage, source_asset=SCENE)
        if len(attached.instances) != 8:
            raise RuntimeError(f"Expected 8 FMU instances, attached {len(attached.instances)}")
        self.previous_command = {wheel: 0.0 for wheel in WHEELS}
        self.next_step = 0

    def step(self, message: dict) -> dict:
        step = message.get("step")
        refs = message.get("reference_rad_s")
        measured = message.get("measurement_rad_s")
        if step != self.next_step:
            raise ValueError(f"Expected step {self.next_step}, got {step}")
        if not isinstance(refs, list) or not isinstance(measured, list):
            raise ValueError("Expected four-element reference and measurement lists")
        if len(refs) != 4 or len(measured) != 4:
            raise ValueError("Expected exactly four wheel signals")
        if not all(isinstance(value, (float, int)) and math.isfinite(value)
                   and abs(value) <= 100.0 for value in (*refs, *measured)):
            raise ValueError("Non-finite or out-of-range wheel signal")
        input_ordinal = 2 * step + 2
        output_ordinal = input_ordinal + 1
        for wheel, ref, omega in zip(WHEELS, refs, measured):
            for kind, name, value in (
                ("Controller", "omega_ref_rad_s", ref),
                ("Controller", "omega_meas_rad_s", omega),
                ("ShadowPlant", "voltage_cmd_V", self.previous_command[wheel]),
            ):
                self.stage.write_attribute(
                    self.queries[(kind, wheel)], self.input_tokens[name], input_ordinal,
                    np.asarray([value], dtype=np.float32), is_array=False,
                ).wait()
        self.stage.advance_write_floor(input_ordinal).wait()
        self.host.update_from_ovstage(input_ordinal, input_ordinal)
        with contextlib.redirect_stdout(sys.stderr):
            self.host.step_sync(H)
        published_groups = self.host.write_to_ovstage(output_ordinal)
        if published_groups < 6:
            raise RuntimeError(f"Only {published_groups} output groups published")
        self.stage.advance_write_floor(output_ordinal).wait()
        controllers = {}
        plants = {}
        for wheel in WHEELS:
            controllers[wheel] = read_state(
                self.stage, self.queries[("Controller", wheel)], self.dictionary,
                CONTROLLER_OUTPUTS, output_ordinal,
            )
            plants[wheel] = read_state(
                self.stage, self.queries[("ShadowPlant", wheel)], self.dictionary,
                PLANT_OUTPUTS, output_ordinal,
            )
            command = controllers[wheel]["voltage_cmd_V"]
            if not math.isfinite(command) or abs(command) > 12.00001:
                raise RuntimeError(f"Controller {wheel} returned invalid voltage {command}")
            self.previous_command[wheel] = command
        self.next_step += 1
        return {
            "status": "ok", "step": step,
            "controller": controllers, "shadow_plant": plants,
            "published_groups": published_groups,
        }

    def close(self):
        self.host.release()
        for key in self.queries:
            self.stage.release_query(self.queries[key]).wait()
            self.dictionary.destroy_path_list(self.paths[key])
        self.dictionary.destroy()
        self.stage.destroy()


def main() -> int:
    master = Master()
    try:
        print(json.dumps({"status": "ready", "instances": 8, "h_s": H}), flush=True)
        for line in sys.stdin:
            message = json.loads(line)
            if message.get("op") == "close":
                print(json.dumps({"status": "closed", "steps": master.next_step}), flush=True)
                return 0
            try:
                result = master.step(message)
            except Exception as exc:
                print(json.dumps({"status": "error", "error": str(exc)}), flush=True)
                raise
            print(json.dumps(result, separators=(",", ":")), flush=True)
        return 0
    finally:
        master.close()


if __name__ == "__main__":
    raise SystemExit(main())
