#!/usr/bin/env python3
"""Preflight Demo C's USD/FMU signal contract before the numerical run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import fmpy
from pxr import Usd


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scenes/demo_C_closed_loop.usda"
EXPECTED = {
    "Controller": {
        "state": "/World/ControllerState",
        "fmu": "MolonbotWheelPIController.fmu",
        "signals": {
            "omega_ref_rad_s": "input",
            "omega_meas_rad_s": "input",
            "voltage_cmd_V": "output",
            "error_rad_s": "output",
            "integral_V": "output",
        },
    },
    "Plant": {
        "state": "/World/PlantState",
        "fmu": "MolonbotWheelPlant.fmu",
        "signals": {
            "voltage_cmd_V": "input",
            "voltage_applied_V": "output",
            "omega_rad_s": "output",
            "theta_rad": "output",
        },
    },
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate(scene_path: Path = SCENE) -> dict:
    scene_path = scene_path.resolve()
    stage = Usd.Stage.Open(str(scene_path))
    require(stage is not None, f"Cannot open USD scene: {scene_path}")
    models = stage.GetPrimAtPath("/World/FmuModels")
    require(models.IsValid(), "Missing /World/FmuModels")
    require({child.GetName() for child in models.GetChildren()} == set(EXPECTED),
            "Expected exactly Controller and Plant FMU instances")
    checked = {}
    for name, contract in EXPECTED.items():
        instance = models.GetChild(name)
        require(instance.GetTypeName() == "FmuInstance", f"{name}: not an FmuInstance")
        asset = instance.GetAttribute("fmi:fmu").Get()
        require(asset is not None and asset.path, f"{name}: missing FMU asset")
        fmu_path = Path(asset.path)
        if not fmu_path.is_absolute():
            fmu_path = scene_path.parent / fmu_path
        fmu_path = fmu_path.resolve()
        require(fmu_path.is_file(), f"{name}: FMU does not exist: {fmu_path}")
        require(fmu_path.name == contract["fmu"],
                f"{name}: expected {contract['fmu']}, got {fmu_path.name}")
        description = fmpy.read_model_description(str(fmu_path), validate=True)
        require(description.coSimulation is not None, f"{name}: missing FMI Co-Simulation")
        variables = {variable.name: variable for variable in description.modelVariables}
        connection = instance.GetChild("Signals")
        require(connection.IsValid(), f"{name}: missing FmuConnection")
        targets = connection.GetRelationship("fmi:targets").GetTargets()
        require([str(target) for target in targets] == [contract["state"]],
                f"{name}: disconnected or wrong USD state target: {targets}")
        state = stage.GetPrimAtPath(contract["state"])
        require(state.IsValid(), f"{name}: missing USD state prim")
        mappings = list(connection.GetChildren())
        require(len(mappings) == len(contract["signals"]),
                f"{name}: expected {len(contract['signals'])} mappings, got {len(mappings)}")
        seen = set()
        for mapping in mappings:
            require(mapping.GetTypeName() == "FmuMapping", f"{name}: non-mapping child")
            variable_name = mapping.GetAttribute("fmi:fmuAttribute").Get()
            direction = mapping.GetAttribute("fmi:direction").Get()
            usd_name = mapping.GetAttribute("fmi:usdAttribute").Get()
            require(variable_name in contract["signals"],
                    f"{name}: unknown or wrong FMU variable {variable_name}")
            require(variable_name not in seen, f"{name}: duplicated mapping {variable_name}")
            require(direction == contract["signals"][variable_name],
                    f"{name}: wrong direction for {variable_name}")
            require(variables[variable_name].causality == direction,
                    f"{name}: FMU causality mismatch for {variable_name}")
            require(usd_name == "molonbot:" + variable_name,
                    f"{name}: wrong USD attribute for {variable_name}")
            attr = state.GetAttribute(usd_name)
            require(attr.IsValid(), f"{name}: missing USD attribute {usd_name}")
            require(attr.Get() == 0.0,
                    f"{name}: {usd_name} must start at 0, got {attr.Get()}")
            seen.add(variable_name)
        require(seen == set(contract["signals"]), f"{name}: disconnected mapping")
        checked[name] = {"fmu": str(fmu_path), "state": contract["state"],
                         "mapped_signals": len(seen)}
    return {"status": "passed", "scene": str(scene_path), "instances": checked}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=SCENE)
    args = parser.parse_args()
    print(json.dumps(validate(args.scene), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
