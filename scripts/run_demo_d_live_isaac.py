#!/usr/bin/env python3
"""Run four live PI FMUs against Isaac Sim/PhysX wheel joints, entirely offline.

The FMUs run in the repository's Python 3.10 environment; Isaac Sim 6.0.1
uses Python 3.12. A synchronous local stdio protocol connects the two without
importing ROS, accessing the Jetson, or commanding physical hardware.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import select
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LOCAL_SCENE = ROOT.parents[1] / "usd/scenes/molonbot_entorno.usd"
MASTER = ROOT / "scripts/demo_d_fmi_master.py"
CONTRACT = ROOT / "scenes/demo_D_live_fmi_contract.usda"
WHEELS = ("FL", "FR", "BL", "BR")
JOINTS = (
    "wheel_left_front_joint", "wheel_right_front_joint",
    "wheel_left_back_joint", "wheel_right_back_joint",
)
H = 0.01
TORQUE_GAIN = 0.004  # synthetic N m / V; not identified from Molonbot hardware
BACK_EMF = 0.20     # synthetic V / (rad/s)
TORQUE_LIMIT = 0.05  # synthetic N m


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reference(step: int, profile: str) -> list[float]:
    if profile == "forward_reverse":
        if 20 <= step < 100:
            return [4.0] * 4
        if 140 <= step < 220:
            return [-4.0] * 4
    elif profile == "turn":
        if 20 <= step < 100:
            return [-4.0, 4.0, -4.0, 4.0]
        if 140 <= step < 220:
            return [4.0, -4.0, 4.0, -4.0]
    else:
        raise ValueError(f"Unknown profile: {profile}")
    return [0.0] * 4


def exchange(process: subprocess.Popen, message: dict) -> dict:
    if process.poll() is not None:
        raise RuntimeError(f"FMU master exited unexpectedly: {process.returncode}")
    process.stdin.write(json.dumps(message, separators=(",", ":")) + "\n")
    process.stdin.flush()
    readable, _, _ = select.select([process.stdout], [], [], 10.0)
    if not readable:
        raise TimeoutError(f"FMU master did not answer step {message.get('step')}")
    line = process.stdout.readline()
    if not line:
        raise RuntimeError("FMU master closed stdout unexpectedly")
    response = json.loads(line)
    if response.get("status") == "error":
        raise RuntimeError(f"FMU master error: {response.get('error')}")
    return response


def yaw_from_quat_wxyz(quat) -> float:
    w, x, y, z = (float(value) for value in quat)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=LOCAL_SCENE)
    parser.add_argument("--output", type=Path, default=ROOT / "results/demo_D_local")
    parser.add_argument("--fmi-python", type=Path, default=ROOT / ".venv/bin/python")
    parser.add_argument("--profile", choices=("forward_reverse", "turn"),
                        default="forward_reverse")
    parser.add_argument("--environment", choices=("full", "clear"), default="full",
                        help="clear keeps the local robot and ground, but disables room geometry")
    parser.add_argument("--steps", type=int, default=320)
    parser.add_argument("--settle-steps", type=int, default=0,
                        help="diagnostic PhysX warm-up; nonzero values expose ovfmi 0.2's first-step input edge case")
    parser.add_argument("--capture-video", action="store_true",
                        help="capture live rendered frames to MP4, not replayed poses")
    parser.add_argument("--camera-height-m", type=float, default=4.0)
    parser.add_argument("--camera-focal-mm", type=float, default=15.0)
    args, kit_args = parser.parse_known_args()
    if args.steps < 320 or args.steps > 2000:
        raise ValueError("Choose between 320 and 2,000 10 ms steps")
    if args.settle_steps < 0 or args.settle_steps > 100:
        raise ValueError("Choose between 0 and 100 settle steps")
    if args.camera_height_m <= 0 or args.camera_focal_mm <= 0:
        raise ValueError("Camera height and focal length must be positive")
    for path in (args.scene, args.fmi_python, MASTER, CONTRACT):
        if not path.is_file():
            raise FileNotFoundError(path)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    (output / "run_status.json").write_text(
        json.dumps({"status": "running", "scene": str(args.scene.resolve())}, indent=2) + "\n",
        encoding="utf-8",
    )
    sys.argv = [sys.argv[0], *kit_args]

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    process = None
    writer = None
    try:
        import numpy as np
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.utils.types import ArticulationAction

        omni.usd.get_context().open_stage(str(args.scene.resolve()))
        for _ in range(20):
            app.update()
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError(f"Isaac Sim could not open {args.scene}")
        stage.SetEditTarget(stage.GetSessionLayer())
        # The local Molonbot scene includes historical ROS graphs. Disable
        # them in this session only, never in the saved source asset.
        disabled_graphs = []
        for path in ("/World/action_graphs", "/World/ActionGraph"):
            graph = stage.GetPrimAtPath(path)
            if graph.IsValid():
                graph.SetActive(False)
                disabled_graphs.append(path)
        disabled_room = []
        if args.environment == "clear":
            for path in ("/World/estructura_sala_actual", "/World/gemelo_semantico",
                         "/World/mapa_camara_actual", "/World/mapa_l2_actual"):
                prim = stage.GetPrimAtPath(path)
                if prim.IsValid():
                    prim.SetActive(False)
                    disabled_room.append(path)
        world = World(stage_units_in_meters=1.0, physics_dt=H, rendering_dt=H)
        robot = world.scene.add(Robot(prim_path="/World/jetauto", name="demo_d_molonbot"))
        world.reset()
        dof_names = list(robot.dof_names)
        if not all(name in dof_names for name in JOINTS):
            raise RuntimeError(f"Missing wheel joints: {set(JOINTS) - set(dof_names)}")
        indices = [dof_names.index(name) for name in JOINTS]
        controller = robot.get_articulation_controller()
        kps, kds = controller.get_gains()
        kps, kds = np.asarray(kps).copy(), np.asarray(kds).copy()
        kps[indices] = 0.0
        kds[indices] = 0.0
        controller.set_gains(kps=kps, kds=kds)
        if (np.any(np.asarray(controller.get_gains()[0])[indices] != 0)
                or np.any(np.asarray(controller.get_gains()[1])[indices] != 0)):
            raise RuntimeError("Wheel drives did not enter effort mode")
        for _ in range(args.settle_steps):
            robot.apply_action(ArticulationAction(
                joint_efforts=np.zeros(4, dtype=float), joint_indices=indices,
            ))
            world.step(render=False)

        captured_frames = 0
        if args.capture_video:
            import cv2
            from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

            ceiling = stage.GetPrimAtPath("/World/estructura_sala_actual/Ceiling")
            if ceiling.IsValid():
                ceiling.SetActive(False)  # camera access only, session layer
            initial_position, _ = robot.get_world_pose()
            camera = RtxCamera(
                "/World/DemoDOverheadCamera", tick_rate=10.0,
                translations=np.array([initial_position[0], initial_position[1],
                                       args.camera_height_m]),
                orientations=np.array([1.0, 0.0, 0.0, 0.0]),
            )
            camera.camera.set_focal_lengths(args.camera_focal_mm)
            camera.camera.set_clipping_ranges(0.01, 100.0)
            camera_sensor = CameraSensor(camera, resolution=(480, 640), annotators=["rgb"])
            writer = cv2.VideoWriter(
                str(output / "live_physx.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                10.0, (640, 480),
            )
            if not writer.isOpened():
                raise RuntimeError("OpenCV could not create the live Isaac MP4")
        with (output / "fmi_master.log").open("w", encoding="utf-8") as log:
            fmi_env = os.environ.copy()
            # Isaac's launcher injects its Python 3.12 stdlib into PYTHONPATH.
            # The FMU subprocess uses the repository's Python 3.10 environment.
            for name in ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP"):
                fmi_env.pop(name, None)
            process = subprocess.Popen(
                [str(args.fmi_python.absolute()), str(MASTER)], cwd=ROOT,
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log,
                text=True, bufsize=1, env=fmi_env,
            )
            readable, _, _ = select.select([process.stdout], [], [], 20.0)
            if not readable:
                raise TimeoutError("FMU master did not initialize")
            ready = json.loads(process.stdout.readline())
            if ready != {"status": "ready", "instances": 8, "h_s": H}:
                raise RuntimeError(f"Unexpected FMU master response: {ready}")
            columns = [
                "step", "t_s", "wheel", "reference_rad_s", "measured_before_rad_s",
                "measured_after_rad_s", "voltage_applied_to_physx_V",
                "voltage_cmd_V", "error_rad_s", "integral_V",
                "shadow_voltage_applied_V", "shadow_omega_rad_s", "shadow_theta_rad",
                "torque_Nm", "base_x_m", "base_y_m", "base_z_m", "base_yaw_rad",
            ]
            rows = []
            previous_voltage = np.zeros(4, dtype=float)
            initial_pose, _ = robot.get_world_pose()
            for step in range(args.steps):
                ref = reference(step, args.profile)
                measured_before = np.asarray(robot.get_joint_velocities()[indices], dtype=float)
                if not np.all(np.isfinite(measured_before)):
                    raise RuntimeError(f"Non-finite PhysX velocity at step {step}")
                answer = exchange(process, {
                    "step": step, "reference_rad_s": ref,
                    "measurement_rad_s": measured_before.tolist(),
                })
                if answer.get("status") != "ok" or answer.get("step") != step:
                    raise RuntimeError(f"Bad FMU sequence response: {answer}")
                voltage = np.asarray(
                    [answer["controller"][wheel]["voltage_cmd_V"] for wheel in WHEELS],
                    dtype=float,
                )
                torque = np.clip(TORQUE_GAIN * (previous_voltage - BACK_EMF * measured_before),
                                 -TORQUE_LIMIT, TORQUE_LIMIT)
                if not np.all(np.isfinite(torque)) or np.max(np.abs(torque)) > TORQUE_LIMIT:
                    raise RuntimeError(f"Invalid simulated motor effort at step {step}")
                robot.apply_action(ArticulationAction(
                    joint_efforts=torque, joint_indices=indices,
                ))
                render_frame = args.capture_video and step % 10 == 0
                world.step(render=render_frame)
                if render_frame:
                    rgb, _ = camera_sensor.get_data("rgb")
                    if rgb is not None:
                        rgb_numpy = rgb.numpy()
                        if rgb_numpy.shape[:2] != (480, 640):
                            raise RuntimeError(f"Unexpected camera image shape {rgb_numpy.shape}")
                        bgr = cv2.cvtColor(rgb_numpy[:, :, :3], cv2.COLOR_RGB2BGR)
                        live_speed = robot.get_joint_velocities()[indices]
                        live_position, _ = robot.get_world_pose()
                        cv2.rectangle(bgr, (0, 0), (639, 107), (12, 18, 24), -1)
                        overlay = (
                            "LIVE: openSeRo FMI 3.0 PI -> ovfmi -> Isaac PhysX",
                            f"t={(step + 1) * H:.2f}s  ref={ref[0]:+.1f} rad/s  "
                            f"wheel={float(live_speed[0]):+.2f} rad/s  V={voltage[0]:+.2f}",
                            f"base=({live_position[0]:+.2f}, {live_position[1]:+.2f}) m  "
                            f"mode={args.environment}  SIMULATION ONLY",
                        )
                        for line_index, label in enumerate(overlay):
                            cv2.putText(bgr, label, (9, 24 + 34 * line_index),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                                        (255, 255, 255), 1, cv2.LINE_AA)
                        writer.write(bgr)
                        if captured_frames == 0:
                            cv2.imwrite(str(output / "live_first_frame.png"), bgr)
                        captured_frames += 1
                measured_after = np.asarray(robot.get_joint_velocities()[indices], dtype=float)
                position, orientation = robot.get_world_pose()
                if not np.all(np.isfinite(measured_after)) or not np.all(np.isfinite(position)):
                    raise RuntimeError(f"Non-finite PhysX state at step {step}")
                yaw = yaw_from_quat_wxyz(orientation)
                for index, wheel in enumerate(WHEELS):
                    c = answer["controller"][wheel]
                    p = answer["shadow_plant"][wheel]
                    rows.append({
                        "step": step + 1, "t_s": (step + 1) * H, "wheel": wheel,
                        "reference_rad_s": ref[index],
                        "measured_before_rad_s": measured_before[index],
                        "measured_after_rad_s": measured_after[index],
                        "voltage_applied_to_physx_V": previous_voltage[index],
                        "voltage_cmd_V": c["voltage_cmd_V"],
                        "error_rad_s": c["error_rad_s"],
                        "integral_V": c["integral_V"],
                        "shadow_voltage_applied_V": p["voltage_applied_V"],
                        "shadow_omega_rad_s": p["omega_rad_s"],
                        "shadow_theta_rad": p["theta_rad"],
                        "torque_Nm": torque[index],
                        "base_x_m": position[0], "base_y_m": position[1],
                        "base_z_m": position[2], "base_yaw_rad": yaw,
                    })
                previous_voltage = voltage
                if step % 50 == 0 or step + 1 == args.steps:
                    print(f"LIVE step {step+1}/{args.steps}: "
                          f"base=({position[0]:.3f},{position[1]:.3f}) "
                          f"omega={[round(v, 2) for v in measured_after]} "
                          f"V={[round(v, 2) for v in voltage]}", flush=True)
            closed = exchange(process, {"op": "close"})
            if closed != {"status": "closed", "steps": args.steps}:
                raise RuntimeError(f"FMU master did not close cleanly: {closed}")
            process.wait(timeout=10)
            if process.returncode != 0:
                raise RuntimeError(f"FMU master exited with {process.returncode}")
            process = None
        if writer is not None:
            writer.release()
            writer = None
            if captured_frames < args.steps // 20:
                raise RuntimeError(f"Too few live frames captured: {captured_frames}")

        trace = output / "trace.csv"
        with trace.open("w", newline="", encoding="utf-8") as stream:
            csv_writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
            csv_writer.writeheader()
            csv_writer.writerows(rows)
        displacement = math.hypot(float(position[0] - initial_pose[0]),
                                  float(position[1] - initial_pose[1]))
        max_displacement = max(math.hypot(
            float(row["base_x_m"] - initial_pose[0]),
            float(row["base_y_m"] - initial_pose[1]),
        ) for row in rows)
        max_speed = max(abs(row["measured_after_rad_s"]) for row in rows)
        max_voltage = max(abs(row["voltage_cmd_V"]) for row in rows)
        max_yaw_change = max(abs(float(row["base_yaw_rad"] - rows[0]["base_yaw_rad"]))
                             for row in rows)
        max_wheel_speed_spread = max(
            max(float(rows[4 * step + index]["measured_after_rad_s"]) for index in range(4))
            - min(float(rows[4 * step + index]["measured_after_rad_s"]) for index in range(4))
            for step in range(args.steps)
        )
        signs = {wheel: {math.copysign(1, row["measured_after_rad_s"])
                         for row in rows if row["wheel"] == wheel
                         and abs(row["measured_after_rad_s"]) > 1.0}
                 for wheel in WHEELS}
        if max_speed < 1.0 or max_voltage > 12.00001:
            raise AssertionError("Live PhysX motion acceptance checks failed")
        if args.profile == "forward_reverse":
            if max_displacement < 0.02 or not all(len(value) == 2 for value in signs.values()):
                raise AssertionError("Forward/reverse profile did not move and reverse all four wheels")
        elif max_yaw_change < 0.05:
            raise AssertionError("Turn profile did not change the simulated robot heading")
        report = {
            "status": "passed", "scope": "live FMU-controlled Isaac/PhysX simulation; no ROS or physical robot",
            "profile": args.profile, "environment": args.environment,
            "steps": args.steps, "h_s": H, "uncommanded_settle_steps": args.settle_steps,
            "fmu_instances": 8, "controllers_in_feedback": 4,
            "plant_fmus_role": "shadow prediction only; PhysX is the controlled plant",
            "communication_delay_steps": 1,
            "live_rendered_frames": captured_frames,
            "live_video_sha256": sha256(output / "live_physx.mp4") if args.capture_video else None,
            "scene_sha256": sha256(args.scene.resolve()),
            "fmi_contract_sha256": sha256(CONTRACT),
            "controller_fmu_sha256": sha256(ROOT / "fmus/MolonbotWheelPIController.fmu"),
            "plant_fmu_sha256": sha256(ROOT / "fmus/MolonbotWheelPlant.fmu"),
            "trace_sha256": sha256(trace),
            "disabled_ros_graphs_in_session": disabled_graphs,
            "disabled_room_prims_in_session": disabled_room,
            "joint_names": list(JOINTS), "joint_indices": indices,
            "synthetic_motor": {
                "torque_gain_Nm_per_V": TORQUE_GAIN,
                "back_emf_V_per_rad_s": BACK_EMF,
                "torque_limit_Nm": TORQUE_LIMIT,
                "equation": "clip(gain * (previous_controller_voltage - back_emf * measured_wheel_speed), +/-torque_limit)",
            },
            "base_displacement_from_initial_m": displacement,
            "maximum_base_displacement_from_initial_m": max_displacement,
            "maximum_abs_measured_wheel_speed_rad_s": max_speed,
            "maximum_wheel_speed_spread_rad_s": max_wheel_speed_spread,
            "maximum_abs_base_yaw_change_rad": max_yaw_change,
            "maximum_abs_controller_voltage_V": max_voltage,
            "all_wheels_reversed_measured_rotation": all(len(value) == 2 for value in signs.values()),
            "limitations": [
                "Motor constants and wheel/ground friction are simulation assumptions, not physical calibration.",
                "The local Molonbot USD is not included in the public repository.",
                "The four shadow plant FMUs do not actuate PhysX.",
                "No real-time wall-clock guarantee, hardware control, or mecanum lateral-motion validation is claimed.",
            ],
        }
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({key: report[key] for key in (
            "status", "profile", "steps", "base_displacement_from_initial_m",
            "maximum_abs_measured_wheel_speed_rad_s",
            "maximum_abs_controller_voltage_V",
            "all_wheels_reversed_measured_rotation", "trace_sha256",
        )}, indent=2), flush=True)
        (output / "run_status.json").write_text(
            json.dumps({"status": "passed", "report_sha256": sha256(output / "report.json")},
                       indent=2) + "\n",
            encoding="utf-8",
        )
        return 0
    except BaseException as exc:
        (output / "run_status.json").write_text(
            json.dumps({"status": "failed", "error_type": type(exc).__name__,
                        "message": str(exc)}, indent=2) + "\n",
            encoding="utf-8",
        )
        raise
    finally:
        if writer is not None:
            writer.release()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
