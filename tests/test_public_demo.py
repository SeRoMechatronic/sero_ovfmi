"""Portable regression checks for the published ovfmi/FMUs and USD scenes."""

from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import fmpy
from pxr import Usd, UsdUtils

from scripts.run_wheel_demo import ROOT, run as run_single
from scripts.run_four_wheel_motion import run as run_four
from scripts.validate_demo_b import validate as validate_b


class PublicDemoTests(unittest.TestCase):
    def test_author_modeling_screenshots_and_gallery(self):
        folder = ROOT / "assets/opensero/screenshots"
        gallery = (ROOT / "docs/OPENSERO_MODELING.md").read_text(encoding="utf-8")
        entries = (folder / "SHA256SUMS").read_text(encoding="utf-8").splitlines()
        self.assertEqual(len(entries), 8)
        self.assertEqual(len(list(folder.glob("*.png"))), 8)
        for entry in entries:
            expected, filename = entry.split("  ", 1)
            path = folder / filename
            self.assertTrue(path.is_file())
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), expected)
            self.assertIn(filename, gallery)

    def test_fmu_metadata_and_validation(self):
        fmu = ROOT / "fmus/MolonbotWheelPlant.fmu"
        self.assertEqual(fmpy.validation.validate_fmu(str(fmu)), [])
        model = fmpy.read_model_description(str(fmu), validate=True)
        self.assertEqual(model.fmiVersion, "3.0")
        self.assertIsNotNone(model.coSimulation)
        variables = {item.name: item for item in model.modelVariables}
        self.assertEqual(variables["voltage_cmd_V"].causality, "input")
        for name in ("voltage_applied_V", "omega_rad_s", "theta_rad"):
            self.assertEqual(variables[name].causality, "output")
        self.assertEqual(
            hashlib.sha256(fmu.read_bytes()).hexdigest(),
            "5cdb7566ef9dcffbca455a31fad3588fde8063e535002ec8899454965d695f76",
        )

    def test_single_fmu_matches_direct_fmpy_and_repeats(self):
        scene = ROOT / "scenes/demo_A_wheel_validation.usda"
        validation = ROOT / "results/demo_A/validation"
        published = json.loads((validation / "report.json").read_text(encoding="utf-8"))
        self.assertEqual(published["scene"], str(scene.relative_to(ROOT)))
        self.assertEqual(published["scene_sha256"], hashlib.sha256(scene.read_bytes()).hexdigest())
        self.assertEqual(
            published["trace_sha256"],
            hashlib.sha256((validation / "trace.csv").read_bytes()).hexdigest(),
        )
        self.assertTrue((ROOT / "results/demo_A/demo_A_robot_only.mp4").is_file())
        self.assertFalse((ROOT / "results/demo_a").exists())
        with tempfile.TemporaryDirectory(prefix="sero_ovfmi_a_") as directory:
            first = run_single(scene, Path(directory) / "first")
            second = run_single(scene, Path(directory) / "second")
        self.assertEqual(first["status"], "passed")
        self.assertEqual(first["steps"], 500)
        self.assertEqual(first["trace_sha256"], second["trace_sha256"])
        self.assertLess(first["max_abs_ovfmi_vs_fmpy"], 5e-6)
        self.assertEqual(first["max_abs_ovstage_vs_ovfmi"], 0)

    def test_four_independent_fmus_and_synthetic_path(self):
        stage = Usd.Stage.Open(str(ROOT / "scenes/demo_movimiento_4ruedas.usda"))
        self.assertTrue(stage)
        self.assertEqual(len(stage.GetPrimAtPath("/World/DrivePlants").GetChildren()), 4)
        prior = json.loads((ROOT / "results/drive/report.json").read_text(encoding="utf-8"))
        report = run_four()
        self.assertEqual(report["steps"], 789)
        self.assertEqual(report["trajectory_sha256"], prior["trajectory_sha256"])
        self.assertLess(report["max_abs_ovfmi_vs_fmpy"], 5e-6)
        self.assertEqual(report["max_abs_front_back_same_side"], 0)
        self.assertGreater(
            report["path_validation"]["semantic_aabb_clearance_after_wheel_footprint_m"],
            0.5,
        )

    def test_demo_b_scene_validation_and_tamper_detection(self):
        report = validate_b()
        published = json.loads(
            (ROOT / "results/demo_B/validation/report.json").read_text(encoding="utf-8")
        )
        self.assertEqual(report, published)
        self.assertEqual(report["checked_pose_samples"], 790)
        self.assertEqual(report["max_position_error_m"], 0.0)
        self.assertEqual(report["max_heading_error_rad"], 0.0)
        self.assertEqual(report["fmu_instances"], 4)
        self.assertEqual(report["mapped_signals_per_instance"], 4)
        self.assertGreater(report["minimum_semantic_aabb_margin_m"], 0.1)
        self.assertGreater(report["minimum_wall_aabb_margin_m"], 0.1)
        with tempfile.TemporaryDirectory(prefix="sero_ovfmi_b_") as directory:
            altered = Path(directory) / "trajectory.csv"
            original = (ROOT / "results/drive/trajectory.csv").read_bytes()
            altered.write_bytes(original.replace(b"0.01,0.0", b"0.01,1.0", 1))
            with self.assertRaisesRegex(ValueError, "trace hash mismatch"):
                validate_b(trace_path=altered)

    def test_timeline_demos_and_local_dependencies(self):
        stages = []
        for name in ("demo_A_robot_only.usda", "demo_B_synthetic_room.usda"):
            path = ROOT / "scenes" / name
            stage = Usd.Stage.Open(str(path))
            self.assertTrue(stage)
            self.assertEqual(stage.GetEndTimeCode(), 789)
            self.assertEqual(stage.GetTimeCodesPerSecond(), 100)
            robot = stage.GetPrimAtPath("/World/jetauto")
            self.assertTrue(robot.IsValid())
            self.assertEqual(robot.GetAttribute("xformOp:translate").GetNumTimeSamples(), 790)
            layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path))
            self.assertFalse(unresolved)
            for dependency in (*layers, *assets):
                identifier = getattr(dependency, "identifier", str(dependency))
                self.assertTrue(Path(str(identifier)).resolve().is_relative_to(ROOT))
            stages.append(stage)
        self.assertFalse(stages[0].GetPrimAtPath("/World/structure").IsValid())
        self.assertTrue(stages[1].GetPrimAtPath("/World/structure").IsValid())
        for frame in (0, 100, 400, 789):
            positions = [
                stage.GetPrimAtPath("/World/jetauto")
                .GetAttribute("xformOp:translate").Get(Usd.TimeCode(frame))
                for stage in stages
            ]
            self.assertEqual(positions[0], positions[1])
        first = stages[0].GetPrimAtPath("/World/jetauto").GetAttribute(
            "xformOp:translate"
        ).Get(Usd.TimeCode(0))
        last = stages[0].GetPrimAtPath("/World/jetauto").GetAttribute(
            "xformOp:translate"
        ).Get(Usd.TimeCode(789))
        self.assertGreater(((last[0] - first[0]) ** 2 + (last[1] - first[1]) ** 2) ** .5, 2)


if __name__ == "__main__":
    unittest.main()
