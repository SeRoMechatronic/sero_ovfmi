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
            (directory / "report.json").write_bytes(REPORT.read_bytes())
            with TRACE.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            rows[100]["voltage_cmd_V"] = str(float(rows[100]["voltage_cmd_V"]) + 0.5)
            scratch = directory / "trace.csv"
            with scratch.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            result = subprocess.run(
                [sys.executable, str(ROOT / "scripts/validate_demo_d.py"), str(scratch),
                 "--output", str(directory / "replay.json")],
                cwd=ROOT, capture_output=True, text=True, check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("Direct FMPy/ovfmi disagreement", result.stderr)


if __name__ == "__main__":
    unittest.main()
