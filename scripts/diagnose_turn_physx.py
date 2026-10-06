#!/usr/bin/env python3
"""Isolate Molonbot wheel/ground physics from FMI and the physical robot.

The private USD is only read. All diagnostic changes are made in a USD session
layer and disappear when Isaac Sim exits. Results are local by default.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LOCAL_SCENE = ROOT.parents[1] / "usd/scenes/molonbot_entorno.usd"
JOINTS = (
    "wheel_left_front_joint", "wheel_right_front_joint",
    "wheel_left_back_joint", "wheel_right_back_joint",
)
H = 0.01


def yaw(quat) -> float:
    w, x, y, z = (float(value) for value in quat)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=LOCAL_SCENE)
    parser.add_argument("--output", type=Path, default=ROOT.parent / "results/turn_diagnostic")
    parser.add_argument("--mode", choices=("velocity", "effort"), default="velocity")
    parser.add_argument("--profile", choices=("turn", "forward", "one_wheel", "front_back", "diagonal"), default="turn")
    parser.add_argument("--environment", choices=("clear", "single_ground"), default="clear")
    parser.add_argument("--wheel-shape", choices=("original", "sphere"), default="original",
                        help="session-only diagnostic replacement; sphere is not a mecanum roller model")
    parser.add_argument("--effort-nm", type=float, default=0.1)
    parser.add_argument("--speed-rad-s", type=float, default=4.0)
    parser.add_argument("--drive-max-effort-nm", type=float, default=None,
                        help="optional PhysX velocity-drive effort cap; session only")
    parser.add_argument("--drive-damping", type=float, default=None,
                        help="optional PhysX velocity-drive damping for controlled diagnostic")
    parser.add_argument("--steps", type=int, default=120)
    args, kit_args = parser.parse_known_args()
    if not args.scene.is_file():
        raise FileNotFoundError(args.scene)
    if not (0 < args.effort_nm <= 2 and 0 < args.speed_rad_s <= 20 and 10 <= args.steps <= 500):
        raise ValueError("Diagnostic parameters outside safe simulation bounds")
    if args.drive_max_effort_nm is not None and not (0 < args.drive_max_effort_nm <= 10):
        raise ValueError("Invalid diagnostic velocity-drive effort cap")
    if args.drive_damping is not None and not (0 < args.drive_damping <= 1000):
        raise ValueError("Invalid diagnostic velocity-drive damping")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    sys.argv = [sys.argv[0], *kit_args]

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import Usd, UsdGeom, UsdPhysics

        omni.usd.get_context().open_stage(str(args.scene.resolve()))
        for _ in range(20):
            app.update()
        stage = omni.usd.get_context().get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        for path in ("/World/action_graphs", "/World/ActionGraph"):
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                prim.SetActive(False)
        if args.environment == "clear":
            for path in ("/World/estructura_sala_actual", "/World/gemelo_semantico",
                         "/World/mapa_camara_actual", "/World/mapa_l2_actual"):
                prim = stage.GetPrimAtPath(path)
                if prim.IsValid():
                    prim.SetActive(False)
        else:
            floor = stage.GetPrimAtPath("/World/estructura_sala_actual/Floor")
            if floor.IsValid() and floor.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(floor).CreateCollisionEnabledAttr(False)
        if args.wheel_shape == "sphere":
            for side in ("left", "right"):
                for end in ("front", "back"):
                    link_path = f"/World/jetauto/wheel_{side}_{end}_link"
                    original = stage.GetPrimAtPath(f"{link_path}/collisions")
                    if not original.IsValid():
                        raise RuntimeError(f"Missing original wheel collision: {original.GetPath()}")
                    original.SetActive(False)
                    sphere = UsdGeom.Sphere.Define(stage, f"{link_path}/diagnostic_sphere")
                    sphere.CreateRadiusAttr(0.049)
                    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())

        collider_paths = []
        collider_geometry = []
        scene_colliders = []
        scene_rigid_bodies = []
        xforms = UsdGeom.XformCache()
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
            if prim.GetPath().pathString.startswith("/World/jetauto/") and prim.HasAPI(UsdPhysics.CollisionAPI):
                collider_paths.append(str(prim.GetPath()))
                position = xforms.GetLocalToWorldTransform(prim).ExtractTranslation()
                collider_geometry.append({"path": str(prim.GetPath()),
                                          "type": prim.GetTypeName(),
                                          "center_m": [float(position[i]) for i in range(3)],
                                          "radius_m": prim.GetAttribute("radius").Get() if prim.GetAttribute("radius") else None})
            if any(prim.GetPath().pathString.startswith(prefix) for prefix in
                   ("/World/gemelo_semantico", "/World/mapa_camara_actual", "/World/mapa_l2_actual")) and prim.HasAPI(UsdPhysics.CollisionAPI):
                position = xforms.GetLocalToWorldTransform(prim).ExtractTranslation()
                enabled = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
                bbox = bounds.ComputeWorldBound(prim).ComputeAlignedRange()
                scene_colliders.append({"path": str(prim.GetPath()), "type": prim.GetTypeName(),
                                        "center_m": [float(position[i]) for i in range(3)],
                                        "bbox_min_m": [float(bbox.GetMin()[i]) for i in range(3)],
                                        "bbox_max_m": [float(bbox.GetMax()[i]) for i in range(3)],
                                        "instance_proxy": prim.IsInstanceProxy(),
                                        "enabled": bool(enabled) if enabled is not None else True})
            if prim.GetPath().pathString.startswith("/World/gemelo_semantico/") and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                position = xforms.GetLocalToWorldTransform(prim).ExtractTranslation()
                scene_rigid_bodies.append({"path": str(prim.GetPath()),
                                           "center_m": [float(position[i]) for i in range(3)],
                                           "instance_proxy": prim.IsInstanceProxy(),
                                           "kinematic": bool(UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get())})
        joint_attributes = {}
        for name in JOINTS:
            prim = stage.GetPrimAtPath(f"/World/jetauto/joints/{name}")
            joint_attributes[name] = {attr.GetName(): str(attr.Get()) for attr in prim.GetAttributes()
                                      if any(word in attr.GetName().lower() for word in
                                             ("maxforce", "damping", "stiffness", "axis", "friction"))}
        root_body_attributes = {}
        for path in ("/World/jetauto", "/World/jetauto/base_link"):
            prim = stage.GetPrimAtPath(path)
            root_body_attributes[path] = {attr.GetName(): str(attr.Get()) for attr in prim.GetAttributes()
                                          if any(word in attr.GetName().lower() for word in
                                                 ("lock", "fix", "kinematic", "gravity", "mass", "velocity", "angular"))}
        all_joint_connections = []
        for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
            if prim.GetPath().pathString.startswith("/World/jetauto/") and prim.GetTypeName().endswith("Joint"):
                all_joint_connections.append({"path": str(prim.GetPath()), "type": prim.GetTypeName(),
                                              "body0": [str(path) for path in
                                                        prim.GetRelationship("physics:body0").GetTargets()] if prim.GetRelationship("physics:body0") else [],
                                              "body1": [str(path) for path in
                                                        prim.GetRelationship("physics:body1").GetTargets()] if prim.GetRelationship("physics:body1") else [],
                                              "axis": str(prim.GetAttribute("physics:axis").Get()) if prim.GetAttribute("physics:axis") else None})

        world = World(stage_units_in_meters=1.0, physics_dt=H, rendering_dt=H)
        robot = world.scene.add(Robot(prim_path="/World/jetauto", name="turn_diagnostic_molonbot"))
        world.reset()
        body_masses = None
        if hasattr(robot, "get_body_masses"):
            body_masses = np.asarray(robot.get_body_masses(), dtype=float).tolist()
        elif hasattr(robot, "_articulation_view") and hasattr(robot._articulation_view, "get_body_masses"):
            body_masses = np.asarray(robot._articulation_view.get_body_masses(), dtype=float).tolist()
        names = list(robot.dof_names)
        indices = [names.index(name) for name in JOINTS]
        all_kp, all_kd = robot.get_articulation_controller().get_gains()
        all_dof_gains = [{"name": name, "kp": float(all_kp[i]), "kd": float(all_kd[i])}
                         for i, name in enumerate(names)]
        max_efforts_before = np.asarray(robot._articulation_view.get_max_efforts(joint_indices=indices), dtype=float).tolist()
        controller = robot.get_articulation_controller()
        effort_modes = controller.get_effort_modes()
        kps, kds = controller.get_gains()
        gains_before = {"kp": np.asarray(kps)[indices].tolist(),
                        "kd": np.asarray(kds)[indices].tolist()}
        if args.mode == "effort":
            kps, kds = np.asarray(kps).copy(), np.asarray(kds).copy()
            kps[indices] = 0.0
            kds[indices] = 0.0
            controller.set_gains(kps=kps, kds=kds)
        else:
            if args.drive_damping is not None:
                kps, kds = np.asarray(kps).copy(), np.asarray(kds).copy()
                kps[indices] = 0.0
                kds[indices] = args.drive_damping
                controller.set_gains(kps=kps, kds=kds)
            if args.drive_max_effort_nm is not None:
                robot._articulation_view.set_max_efforts(
                    np.full((1, 4), args.drive_max_effort_nm, dtype=float), joint_indices=indices)
        max_efforts_after = np.asarray(robot._articulation_view.get_max_efforts(joint_indices=indices), dtype=float).tolist()
        initial_position, initial_orientation = robot.get_world_pose()
        initial_yaw = yaw(initial_orientation)
        rows = []
        for step in range(args.steps):
            if 20 <= step < 100:
                if args.profile == "turn":
                    signs = np.array([-1., 1., -1., 1.])
                elif args.profile == "forward":
                    signs = np.ones(4)
                elif args.profile == "front_back":
                    signs = np.array([1., 1., -1., -1.])
                elif args.profile == "diagonal":
                    signs = np.array([1., -1., -1., 1.])
                else:
                    signs = np.array([1., 0., 0., 0.])
            else:
                signs = np.zeros(4)
            command = signs * (args.effort_nm if args.mode == "effort" else args.speed_rad_s)
            if args.mode == "effort":
                action = ArticulationAction(joint_efforts=command, joint_indices=indices)
            else:
                action = ArticulationAction(joint_velocities=command, joint_indices=indices)
            robot.apply_action(action)
            world.step(render=False)
            position, orientation = robot.get_world_pose()
            velocities = np.asarray(robot.get_joint_velocities()[indices], dtype=float)
            measured_efforts = np.asarray(robot.get_measured_joint_efforts(joint_indices=indices), dtype=float)
            applied_efforts = np.asarray(robot.get_applied_joint_efforts(joint_indices=indices), dtype=float)
            rows.append({"step": step + 1, "t_s": (step + 1) * H,
                         "yaw_rad": yaw(orientation), "x_m": float(position[0]),
                         "y_m": float(position[1]), "z_m": float(position[2]),
                         **{f"{name}_rad_s": float(value) for name, value in zip(JOINTS, velocities)},
                         **{f"{name}_measured_effort_nm": float(value)
                            for name, value in zip(JOINTS, measured_efforts)},
                         **{f"{name}_applied_effort_nm": float(value)
                            for name, value in zip(JOINTS, applied_efforts)}})
        with (output / "trace.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
        report = {"scope": "Isaac Sim only; no FMI, ROS, or physical robot",
                  "mode": args.mode, "profile": args.profile, "environment": args.environment,
                  "wheel_shape": args.wheel_shape,
                  "command": args.effort_nm if args.mode == "effort" else args.speed_rad_s,
                  "drive_max_effort_nm": args.drive_max_effort_nm,
                  "drive_damping": args.drive_damping,
                  "steps": args.steps, "h_s": H, "gains_before": gains_before,
                  "max_efforts_before_nm": max_efforts_before,
                  "max_efforts_after_nm": max_efforts_after,
                  "initial_pose": {"position_m": np.asarray(initial_position).tolist(),
                                   "yaw_rad": initial_yaw},
                  "collider_count_with_instance_proxies": len(collider_paths),
                  "wheel_collider_paths": [path for path in collider_paths if "wheel_" in path],
                  "collider_geometry": collider_geometry,
                  "scene_colliders": scene_colliders,
                  "scene_rigid_bodies": scene_rigid_bodies,
                  "joint_attributes": joint_attributes,
                  "root_body_attributes": root_body_attributes,
                  "all_joint_connections": all_joint_connections,
                  "all_dof_gains": all_dof_gains,
                  "effort_modes": effort_modes,
                  "body_masses_kg": body_masses,
                  "maximum_abs_yaw_change_rad": max(abs(row["yaw_rad"] - initial_yaw) for row in rows),
                  "maximum_abs_wheel_speed_rad_s": max(abs(row[f"{name}_rad_s"])
                                                       for row in rows for name in JOINTS),
                  "maximum_abs_measured_effort_nm": max(abs(row[f"{name}_measured_effort_nm"])
                                                        for row in rows for name in JOINTS),
                  "maximum_abs_applied_effort_nm": max(abs(row[f"{name}_applied_effort_nm"])
                                                       for row in rows for name in JOINTS),
                  "maximum_base_displacement_m": max(math.hypot(row["x_m"] - initial_position[0],
                                                                row["y_m"] - initial_position[1])
                                                     for row in rows)}
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({key: report[key] for key in (
            "mode", "profile", "environment", "maximum_abs_yaw_change_rad",
            "maximum_abs_wheel_speed_rad_s", "maximum_base_displacement_m",
        )}, indent=2), flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
