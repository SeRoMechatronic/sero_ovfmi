"""Portable, offline regression checks for the published Demo C evidence."""

from __future__ import annotations

import hashlib
import csv
import json
import math
import tempfile
import unittest
from pathlib import Path

import fmpy
from fmpy.fmi3 import FMU3Slave
from fmpy.validation import validate_fmu
from pxr import Usd, UsdUtils

from scripts.run_demo_c import CONTROLLER, PLANT, ROOT, run
from scripts.validate_demo_c_scene import validate as validate_scene


class DemoCTests(unittest.TestCase):
    def test_controller_modeling_screenshots(self):
        folder = ROOT / "assets/opensero/controller_screenshots"
        guide = (ROOT / "docs/DEMO_C.md").read_text(encoding="utf-8")
        entries = (folder / "SHA256SUMS").read_text(encoding="utf-8").splitlines()
        self.assertEqual(len(entries), 8)
        self.assertEqual(len(list(folder.glob("*.png"))), 8)
        for entry in entries:
            expected, filename = entry.split("  ", 1)
            self.assertEqual(hashlib.sha256((folder / filename).read_bytes()).hexdigest(),
                             expected)
            self.assertIn(filename, guide)

    def test_controller_fmu_contract_and_first_step(self):
        self.assertEqual(validate_fmu(str(CONTROLLER)), [])
        model = fmpy.read_model_description(str(CONTROLLER), validate=True)
        self.assertEqual(model.fmiVersion, "3.0")
        self.assertIsNotNone(model.coSimulation)
        self.assertEqual(model.defaultExperiment.stepSize, 0.01)
        variables = {variable.name: variable for variable in model.modelVariables}
        self.assertEqual(
            {name for name, var in variables.items() if var.causality == "input"},
            {"omega_ref_rad_s", "omega_meas_rad_s"},
        )
        self.assertEqual(
            {name for name, var in variables.items() if var.causality == "output"},
            {"voltage_cmd_V", "error_rad_s", "integral_V"},
        )
        self.assertEqual(len({var.valueReference for var in variables.values()}), len(variables))
        with tempfile.TemporaryDirectory(prefix="sero_controller_test_") as directory:
            fmpy.extract(str(CONTROLLER), unzipdir=directory)
            slave = FMU3Slave(
                guid=model.guid, unzipDirectory=directory,
                modelIdentifier=model.coSimulation.modelIdentifier,
                instanceName="controller_first_step",
            )
            slave.instantiate()
            try:
                slave.enterInitializationMode(startTime=0.0)
                slave.exitInitializationMode()
                refs = {name: var.valueReference for name, var in variables.items()}
                outputs = [refs[name] for name in
                           ("voltage_cmd_V", "error_rad_s", "integral_V")]
                self.assertEqual(slave.getFloat64(outputs), [0.0, 0.0, 0.0])
                slave.setFloat64([refs["omega_ref_rad_s"], refs["omega_meas_rad_s"]],
                                 [8.0, 0.0])
                slave.doStep(currentCommunicationPoint=0.0, communicationStepSize=0.01)
                self.assertEqual(slave.getFloat64(outputs), [3.2, 8.0, 0.096])
                with self.assertRaises(Exception) as invalid:
                    slave.setFloat64([refs["omega_meas_rad_s"]], [math.nan])
                self.assertEqual(type(invalid.exception).__name__, "FMICallException")
            finally:
                slave.freeInstance()

    def test_fixed_step_and_parameter_rejection(self):
        model = fmpy.read_model_description(str(CONTROLLER), validate=True)
        refs = {var.name: var.valueReference for var in model.modelVariables}
        with tempfile.TemporaryDirectory(prefix="sero_controller_negative_") as directory:
            fmpy.extract(str(CONTROLLER), unzipdir=directory)
            for index, (parameter, value) in enumerate(
                (("voltage_limit_V", 0.0), ("kp_V_per_rad_s", -0.1),
                 ("ki_V_per_rad", math.nan))
            ):
                slave = FMU3Slave(guid=model.guid, unzipDirectory=directory,
                                  modelIdentifier=model.coSimulation.modelIdentifier,
                                  instanceName=f"invalid_parameter_{index}")
                slave.instantiate()
                try:
                    with self.assertRaises(Exception) as invalid:
                        slave.setFloat64([refs[parameter]], [value])
                    self.assertEqual(type(invalid.exception).__name__, "FMICallException")
                finally:
                    slave.freeInstance()
            slave = FMU3Slave(guid=model.guid, unzipDirectory=directory,
                              modelIdentifier=model.coSimulation.modelIdentifier,
                              instanceName="invalid_step")
            slave.instantiate()
            try:
                slave.enterInitializationMode(startTime=0.0)
                slave.exitInitializationMode()
                with self.assertRaises(Exception) as invalid:
                    slave.doStep(currentCommunicationPoint=0.0, communicationStepSize=0.02)
                self.assertEqual(type(invalid.exception).__name__, "FMICallException")
            finally:
                slave.freeInstance()

    def test_closed_loop_trace_scene_and_repeatability(self):
        published = json.loads(
            (ROOT / "results/demo_C/report.json").read_text(encoding="utf-8")
        )
        self.assertEqual(published["status"], "passed")
        self.assertEqual(published["steps"], 1000)
        self.assertEqual(published["controller_fmu_sha256"],
                         hashlib.sha256(CONTROLLER.read_bytes()).hexdigest())
        self.assertEqual(published["plant_fmu_sha256"],
                         hashlib.sha256(PLANT.read_bytes()).hexdigest())
        self.assertEqual(published["trace_sha256"], hashlib.sha256(
            (ROOT / "results/demo_C/trace.csv").read_bytes()).hexdigest())
        scene_path = ROOT / "scenes/demo_C_closed_loop.usda"
        stage = Usd.Stage.Open(str(scene_path))
        self.assertTrue(stage)
        for path, expected in (("/World/FmuModels/Controller", 5),
                               ("/World/FmuModels/Plant", 4)):
            instance = stage.GetPrimAtPath(path)
            self.assertTrue(instance.IsValid())
            mappings = list(instance.GetChild("Signals").GetChildren())
            self.assertEqual(len(mappings), expected)
            self.assertEqual(len({item.GetAttribute("fmi:fmuAttribute").Get()
                                  for item in mappings}), expected)
        _, _, unresolved = UsdUtils.ComputeAllDependencies(str(scene_path))
        self.assertFalse(unresolved)
        visual = Usd.Stage.Open(str(ROOT / "scenes/demo_C_wheel_visual.usda"))
        self.assertTrue(visual)
        self.assertEqual(visual.GetPrimAtPath("/World/Visual/WheelAngle")
                         .GetAttribute("xformOp:rotateZ").GetNumTimeSamples(), 1001)
        with (ROOT / "results/demo_C/trace.csv").open(newline="", encoding="utf-8") as stream:
            trace_rows = list(csv.DictReader(stream))
        angle = visual.GetPrimAtPath("/World/Visual/WheelAngle").GetAttribute(
            "xformOp:rotateZ"
        )
        for frame in (0, 20, 320, 620, 720, 1000):
            self.assertAlmostEqual(
                angle.Get(Usd.TimeCode(frame)),
                math.degrees(float(trace_rows[frame]["theta_rad_ovstage"])),
                delta=1e-3,  # USD rotateZ op stores float32 degrees.
            )
        render = json.loads((ROOT / "results/demo_C/render_manifest.json")
                            .read_text(encoding="utf-8"))
        video = json.loads((ROOT / "results/demo_C/video_metadata.json")
                           .read_text(encoding="utf-8"))
        self.assertEqual(render["frames"], 61)
        self.assertEqual(render["trace_sha256"], published["trace_sha256"])
        self.assertEqual(render["scene_sha256"], hashlib.sha256(
            (ROOT / "scenes/demo_C_wheel_visual.usda").read_bytes()).hexdigest())
        self.assertEqual(video["scene_sha256"], render["scene_sha256"])
        self.assertEqual(video["sha256"], hashlib.sha256(
            (ROOT / "results/demo_C/demo_C_closed_loop.mp4").read_bytes()).hexdigest())
        with tempfile.TemporaryDirectory(prefix="sero_demo_c_") as directory:
            reproduced = run(Path(directory))
        self.assertEqual(reproduced["trace_sha256"], published["trace_sha256"])
        self.assertLess(max(reproduced["max_abs_ovfmi_vs_direct_fmpy_by_signal"].values()),
                        reproduced["declared_ovfmi_vs_fmpy_tolerance"])
        self.assertEqual(reproduced["max_abs_ovstage_vs_ovfmi"], 0.0)

    def test_scene_preflight_rejects_bad_connections_and_assets(self):
        original = (ROOT / "scenes/demo_C_closed_loop.usda").read_text(encoding="utf-8")
        portable = original.replace("@../fmus/", "@" + str(ROOT / "fmus") + "/")
        self.assertEqual(validate_scene()["status"], "passed")
        cases = (
            ("wrong_variable",
             'string fmi:fmuAttribute = "omega_ref_rad_s"',
             'string fmi:fmuAttribute = "omega_ref_TYPO"',
             "unknown or wrong FMU variable"),
            ("disconnected_target",
             'rel fmi:targets = </World/ControllerState>',
             'rel fmi:targets = </World/Disconnected>',
             "disconnected or wrong USD state target"),
            ("wrong_start",
             'float molonbot:omega_ref_rad_s = 0',
             'float molonbot:omega_ref_rad_s = 1',
             "must start at 0"),
            ("missing_fmu",
             'MolonbotWheelPIController.fmu@',
             'MissingController.fmu@',
             "FMU does not exist"),
        )
        with tempfile.TemporaryDirectory(prefix="sero_demo_c_bad_usd_") as directory:
            for name, before, after, error in cases:
                self.assertIn(before, portable)
                path = Path(directory) / f"{name}.usda"
                path.write_text(portable.replace(before, after, 1), encoding="utf-8")
                with self.subTest(name=name), self.assertRaisesRegex(ValueError, error):
                    validate_scene(path)


if __name__ == "__main__":
    unittest.main()
