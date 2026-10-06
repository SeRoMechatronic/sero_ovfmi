#!/usr/bin/env python3
"""Build the eight-instance USD signal contract for live Isaac Demo D.

Four PI controllers close the loop against PhysX wheel velocities. Four plant
FMUs run only as shadow predictions: they never command the simulated robot.
"""

from __future__ import annotations

from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "scenes/demo_D_live_fmi_contract.usda"
WHEELS = ("FL", "FR", "BL", "BR")
CONTROLLER = ROOT / "fmus/MolonbotWheelPIController.fmu"
PLANT = ROOT / "fmus/MolonbotWheelPlant.fmu"
CONTROLLER_SIGNALS = {
    "omega_ref_rad_s": "input",
    "omega_meas_rad_s": "input",
    "voltage_cmd_V": "output",
    "error_rad_s": "output",
    "integral_V": "output",
}
PLANT_SIGNALS = {
    "voltage_cmd_V": "input",
    "voltage_applied_V": "output",
    "omega_rad_s": "output",
    "theta_rad": "output",
}


def make_instance(stage: Usd.Stage, kind: str, wheel: str,
                  fmu: Path, signals: dict[str, str]) -> None:
    state_path = f"/World/{kind}States/{wheel}"
    state = UsdGeom.Xform.Define(stage, state_path).GetPrim()
    for signal in signals:
        state.CreateAttribute(f"molonbot:{signal}", Sdf.ValueTypeNames.Float).Set(0.0)
    instance = stage.DefinePrim(f"/World/FmuModels/{kind}_{wheel}", "FmuInstance")
    instance.CreateAttribute("fmi:fmu", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(f"../fmus/{fmu.name}")
    )
    instance.CreateAttribute("fmi:enabled", Sdf.ValueTypeNames.Bool).Set(True)
    connection = stage.DefinePrim(f"{instance.GetPath()}/Signals", "FmuConnection")
    connection.CreateRelationship("fmi:targets").SetTargets([Sdf.Path(state_path)])
    for signal, direction in signals.items():
        mapping = stage.DefinePrim(f"{connection.GetPath()}/{signal}", "FmuMapping")
        mapping.CreateAttribute("fmi:fmuAttribute", Sdf.ValueTypeNames.String).Set(signal)
        mapping.CreateAttribute("fmi:usdAttribute", Sdf.ValueTypeNames.String).Set(
            f"molonbot:{signal}"
        )
        mapping.CreateAttribute("fmi:direction", Sdf.ValueTypeNames.Token).Set(direction)
        mapping.CreateAttribute("fmi:usdMapping", Sdf.ValueTypeNames.Int2).Set(Gf.Vec2i(0, 1))


def main() -> int:
    for fmu in (CONTROLLER, PLANT):
        if not fmu.is_file():
            raise FileNotFoundError(fmu)
    stage = Usd.Stage.CreateInMemory()
    stage.SetMetadata("metersPerUnit", 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    stage.SetMetadata("customLayerData", {
        "purpose": "Live Isaac/PhysX four-wheel PI control through ovfmi",
        "safety": "Offline simulated hardware only; no ROS or robot actuator connection",
        "plant_role": "The four plant FMUs are shadow predictions, not PhysX actuators",
    })
    UsdGeom.Xform.Define(stage, "/World/FmuModels")
    for kind in ("Controller", "ShadowPlant"):
        UsdGeom.Xform.Define(stage, f"/World/{kind}States")
    for wheel in WHEELS:
        make_instance(stage, "Controller", wheel, CONTROLLER, CONTROLLER_SIGNALS)
        make_instance(stage, "ShadowPlant", wheel, PLANT, PLANT_SIGNALS)
    stage.GetRootLayer().Export(str(OUTPUT))
    OUTPUT.write_bytes(OUTPUT.read_bytes().rstrip(b"\n") + b"\n")
    print(OUTPUT.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
