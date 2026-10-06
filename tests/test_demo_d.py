"""Public regressions for Demo D's FMI contract and recorded live trace."""

from __future__ import annotations

import csv
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from pxr import Usd


ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / "scenes/demo_D_live_fmi_contract.usda"
TRACE = ROOT / "results/demo_D/trace.csv"
REPORT = ROOT / "results/demo_D/report.json"
REPLAY = ROOT / "results/demo_D/replay_validation.json"
TURN_TRACE = ROOT / "results/demo_D_turn/trace.csv"
TURN_REPORT = ROOT / "results/demo_D_turn/report.json"
TURN_REPLAY = ROOT / "results/demo_D_turn/replay_validation.json"
WHEELS = ("FL", "FR", "BL", "BR")


class DemoDTests(unittest.TestCase):
    def test_eight_distinct_mapped_fmu_instances(self):
        stage = Usd.Stage.Open(str(STAGE))
        instances = [prim for prim in stage.Traverse() if prim.GetTypeName() == "FmuInstance"]
        self.assertEqual(len(instances), 8)
        expected = {(kind, wheel) for kind in ("Controller", "ShadowPlant")
                    for wheel in WHEELS}
        found = set()
        for instance in instances:
            kind, wheel = instance.GetName().split("_", 1)
            found.add((kind, wheel))
            asset = instance.GetAttribute("fmi:fmu").Get().path
            self.assertTrue((STAGE.parent / asset).resolve().is_file())
            state_path = f"/World/{kind}States/{wheel}"
            connection = stage.GetPrimAtPath(str(instance.GetPath()) + "/Signals")
            self.assertEqual(str(connection.GetRelationship("fmi:targets").GetTargets()[0]),
                             state_path)
            mappings = [child for child in connection.GetChildren()
                        if child.GetTypeName() == "FmuMapping"]
            self.assertEqual(len(mappings), 5 if kind == "Controller" else 4)
            for mapping in mappings:
                signal = mapping.GetAttribute("fmi:fmuAttribute").Get()
                self.assertEqual(mapping.GetAttribute("fmi:usdAttribute").Get(),
                                 "molonbot:" + signal)
                expected_input = signal in (("omega_ref_rad_s", "omega_meas_rad_s")
                                            if kind == "Controller" else ("voltage_cmd_V",))
                self.assertEqual(mapping.GetAttribute("fmi:direction").Get(),
                                 "input" if expected_input else "output")
                self.assertTrue(stage.GetPrimAtPath(state_path).HasAttribute(
                    "molonbot:" + signal))
        self.assertEqual(found, expected)

    def test_recorded_trace_is_complete_and_replayable(self):
        report = json.loads(REPORT.read_text(encoding="utf-8"))
        replay = json.loads(REPLAY.read_text(encoding="utf-8"))
        with TRACE.open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["environment"], "single_ground")
        self.assertTrue(report["disabled_duplicate_room_floor_collision_in_session"])
        self.assertEqual((report["steps"], len(rows)), (320, 1280))
        digest = hashlib.sha256(TRACE.read_bytes()).hexdigest()
        self.assertEqual(report["trace_sha256"], digest)
        self.assertEqual(replay["trace_sha256"], digest)
        self.assertLess(replay["maximum_abs_fmpy_ovfmi_signal_difference"],
                        replay["comparison_tolerance"])
        # Recompute the independent FMU replay, rather than trusting only JSON.
        with tempfile.TemporaryDirectory(prefix="sero_demo_d_test_") as directory:
            result = subprocess.run(
                [sys.executable, str(ROOT / "scripts/validate_demo_d.py"), str(TRACE),
                 "--output", str(Path(directory) / "replay.json")],
                cwd=ROOT, capture_output=True, text=True, check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_replay_rejects_tampered_fmu_output(self):
        with tempfile.TemporaryDirectory(prefix="sero_demo_d_tamper_") as directory:
            directory = Path(directory)
            with TRACE.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            rows[100]["voltage_cmd_V"] = str(float(rows[100]["voltage_cmd_V"]) + 0.5)
            scratch = directory / "trace.csv"
            with scratch.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            report = json.loads(REPORT.read_text(encoding="utf-8"))
            report["trace_sha256"] = hashlib.sha256(scratch.read_bytes()).hexdigest()
            (directory / "report.json").write_text(json.dumps(report), encoding="utf-8")
            result = subprocess.run(
                [sys.executable, str(ROOT / "scripts/validate_demo_d.py"), str(scratch),
                 "--output", str(directory / "replay.json")],
                cwd=ROOT, capture_output=True, text=True, check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("Direct FMPy/ovfmi disagreement", result.stderr)

    def test_bounded_drive_turn_reverses_in_live_physx(self):
        report = json.loads(TURN_REPORT.read_text(encoding="utf-8"))
        replay = json.loads(TURN_REPLAY.read_text(encoding="utf-8"))
        self.assertEqual(report["status"], "passed")
        self.assertEqual(report["steps"], 500)
        self.assertEqual(report["motor_model"], "bounded_velocity_drive")
        self.assertEqual(report["profile"], "turn")
        self.assertEqual(report["environment"], "single_ground")
        self.assertEqual(len(report["kinematic_cabinet_bodies_in_session"]), 6)
        self.assertTrue(report["disabled_duplicate_room_floor_collision_in_session"])
        self.assertLess(report["maximum_base_displacement_from_initial_m"], 0.30)
        self.assertLess(report["terminal_max_abs_wheel_speed_rad_s"], 0.15)
        self.assertLess(report["terminal_heading_drift_last_50_steps_rad"], 0.01)
        self.assertLess(report["first_turn_yaw_change_rad"], -0.05)
        self.assertGreater(report["second_turn_yaw_change_rad"], 0.05)
        digest = hashlib.sha256(TURN_TRACE.read_bytes()).hexdigest()
        self.assertEqual(report["trace_sha256"], digest)
        self.assertEqual(replay["trace_sha256"], digest)
        with tempfile.TemporaryDirectory(prefix="sero_demo_d_turn_test_") as directory:
            result = subprocess.run(
                [sys.executable, str(ROOT / "scripts/validate_demo_d.py"), str(TURN_TRACE),
                 "--output", str(Path(directory) / "replay.json")],
                cwd=ROOT, capture_output=True, text=True, check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_bounded_drive_replay_rejects_tampered_motor_target(self):
        with tempfile.TemporaryDirectory(prefix="sero_demo_d_motor_tamper_") as directory:
            directory = Path(directory)
            with TURN_TRACE.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            rows[100]["motor_target_rad_s"] = str(float(rows[100]["motor_target_rad_s"]) + 0.5)
            scratch = directory / "trace.csv"
            with scratch.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            report = json.loads(TURN_REPORT.read_text(encoding="utf-8"))
            report["trace_sha256"] = hashlib.sha256(scratch.read_bytes()).hexdigest()
            (directory / "report.json").write_text(json.dumps(report), encoding="utf-8")
            result = subprocess.run(
                [sys.executable, str(ROOT / "scripts/validate_demo_d.py"), str(scratch),
                 "--output", str(directory / "replay.json")],
                cwd=ROOT, capture_output=True, text=True, check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("motor equation is inconsistent", result.stderr)


if __name__ == "__main__":
    unittest.main()
