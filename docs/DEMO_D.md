# Demo D — live openSeRo PI control of Isaac Sim/PhysX wheel joints

Demo D is the first experiment here in which the FMU output **acts on the simulated articulated robot during the physics run**. It is not another playback of precomputed USD poses. Four instances of the author's FMI 3.0 PI-controller FMU receive the four wheel velocities measured by PhysX, through `ovstage` and `ovfmi`, every 10 ms. An explicitly selected synthetic motor adapter applies their commands to the four revolute wheel joints. Four separate wheel-plant FMUs run in parallel as **shadow predictions**; their outputs do not actuate PhysX.

This experiment uses the private local Molonbot USD and deliberately does **not** publish its mesh, room geometry, or rendered video in this MIT repository. The [signal-contract stage](../scenes/demo_D_live_fmi_contract.usda), [live runner](../scripts/run_demo_d_live_isaac.py), [FMU-side master](../scripts/demo_d_fmi_master.py), original [forward trace](../results/demo_D/trace.csv), new [turn trace](../results/demo_D_turn/trace.csv), their reports and independent FMPy replays are public. The local videos were captured from the live camera stream during simulation, not reconstructed from CSV, but are held outside this repository pending an asset-publication decision.

## Architecture and explicit delay

Isaac Sim 6.0.1 uses Python 3.12, while the pinned ovstage/ovfmi environment here uses Python 3.10. The two processes exchange one JSON line per simulation step over local standard I/O. No ROS bridge, network socket, Jetson, or physical motor interface is imported by this demo. The Isaac process disables the legacy ROS action graph in a USD *session layer* without editing the source asset.

At step `k`, PhysX supplies the measured wheel speeds and the master writes those values and the desired references into the four controller USD state prims. It also feeds each shadow plant the controller voltage from step `k-1`. `FmiHost` reads the ovstage inputs, advances all eight FMU instances by 10 ms, and publishes their outputs. Isaac converts the **previous** controller voltage into a motor command, advances PhysX by 10 ms, and logs the newly measured velocities and robot pose. This is a one-step communication delay, not an instantaneous electromechanical coupling.

The original `direct_torque` law remains available: `torque = clip(0.004 * (previous_voltage - 0.20 * measured_omega), -0.05, +0.05)` in N·m. This law passed forward/reverse but failed turn-in-place. The new, opt-in `bounded_velocity_drive` maps voltage to a velocity target at `1 rad/s per V`, with PhysX drive damping `1 N·m/(rad/s)` and maximum drive effort `0.5 N·m` per wheel. It is an **inner PhysX velocity servo**, not a calibrated voltage-to-torque DC motor or proof that the real robot can deliver this effort. The outer PI FMUs still close the loop on measured PhysX wheel speed. All constants are chosen simulation values, not identified from Molonbot hardware.

The `single_ground` mode retains the local room geometry and turns off only the *duplicate reconstructed-floor collider* in a USD session layer; the pre-existing base ground collider remains active. The optional `--freeze-cabinets` holds the six dynamic pieces of two stationary semantic cabinets kinematic **in the session only**, preserving their visible meshes and colliders. Neither operation edits the source USD. `clear` removes room geometry for isolation; `full` retains both floor colliders as a historical diagnostic.

## Measured result and what it does *not* establish

The recorded `single_ground` run, with the room still loaded, completed 320 physics steps (3.2 s) and 1,280 wheel samples with a +4/0/−4/0 rad/s profile. All four measured wheel speeds changed sign; the maximum wheel speed was **2.934 rad/s**, the maximum command was **4.690 V**, and the base reached **0.152 m** from its initial location before moving back. Thirty frames were captured directly from the live Isaac camera. The [replay validator](../scripts/validate_demo_d.py) independently stepped eight FMU instances with FMPy using the logged references and measured PhysX speeds. Its maximum difference from the ovfmi-published controller and shadow-plant signals was **2.3842×10⁻⁷**, below the declared `1×10⁻⁴` tolerance. It also checked the one-step signal continuity, torque equation, 12 V command bound, and trace hash. This validates the FMI/ovfmi coupling numerically; it does **not** validate the motor model against physical data.

In a controlled comparison, the raw `full` room run with **both** coplanar floor colliders reached a wheel-speed spread of `16.502 rad/s`. Disabling only the reconstructed floor's collision reduced that spread to `0.097 rad/s`; the resulting trace matched the `clear` run. This single-change result strongly implicates duplicate ground contact in the instability, although no PhysX contact-event trace was recorded.

The original direct-torque turn still fails and remains a recorded **negative result**. An isolated direct velocity-drive diagnostic established that the articulation can change heading, while a bounded-effort velocity drive with `0.5 N·m` maximum per wheel passed the turn profile. With the complete semantic room loaded, a separate instability appeared after the second turn: the base drifted more than `0.30 m`. Disabling only the two semantic cabinets removed the drift; inspecting the USD found six unanchored dynamic rigid bodies inside them. Holding those bodies kinematic in a session layer produced the same clean trace while retaining the cabinets' visuals and colliders. This controlled ablation implicates the cabinet dynamics; a contact-event trace has not been captured, so the exact collision mechanism is not claimed.

The [new turn report](../results/demo_D_turn/report.json) records 500 steps, a first heading change of **−0.19957 rad**, a reversed change of **+0.10494 rad**, and maximum base displacement of **0.03612 m**. Each wheel exceeded **2.25 rad/s in its commanded direction** in both turn phases; peak measured wheel speed was **4.991 rad/s** and solver-reported joint effort **0.5023 N·m** (the configured drive limit is `0.5 N·m`; the small excess is a numerical solver reading). After the extended zero-reference interval, maximum wheel speed was **0.0719 rad/s** and heading drift over the final 50 steps was **0.00295 rad**. The [English plot](../results/demo_D_turn/turn.svg) shows the logged wheel speeds, heading, and planar drift without distributing the private robot mesh. The [independent replay](../results/demo_D_turn/replay_validation.json) found maximum FMU/ovfmi signal difference **2.3842×10⁻⁷**, zero one-step and motor-mapping error, and both opposite heading changes above the declared `0.05 rad` threshold. The local live camera captured 48 frames for a separate matching run. This validates **this one simulation profile**; it does not establish robust turning across environments, obstacle-aware navigation, mecanum lateral motion, real-time scheduling, hardware-in-the-loop, or real-robot safety.

## Reproduce what is public

The checked-in trace can be verified without Isaac Sim or any private asset:

```bash
.venv/bin/python scripts/validate_demo_d.py results/demo_D/trace.csv
.venv/bin/python scripts/validate_demo_d.py results/demo_D_turn/trace.csv
.venv/bin/python scripts/plot_demo_d_turn.py
.venv/bin/python -m unittest tests.test_demo_d -v
```

To rerun the live physics experiment, supply your own licensed, compatible USD stage with `/World/jetauto`, an actual PhysX articulation, a ground collider, and the four joint names documented in the report. With Isaac Sim 6.0.1 installed, use:

```bash
.venv/bin/python scripts/build_demo_d_fmi_scene.py
/path/to/isaacsim/python.sh scripts/run_demo_d_live_isaac.py \
  --scene /path/to/your/compatible_robot_scene.usd \
  --output /path/to/local_output \
  --environment single_ground --steps 320 --capture-video
.venv/bin/python scripts/validate_demo_d.py /path/to/local_output/trace.csv
```

For the tested turn profile in the local Molonbot room, select the bounded drive and cabinet stabilization explicitly:

```bash
/path/to/isaacsim/python.sh scripts/run_demo_d_live_isaac.py \
  --scene /path/to/local_molonbot_scene.usd \
  --output /path/to/local_turn_output \
  --profile turn --motor-model bounded_velocity_drive \
  --environment single_ground --freeze-cabinets --steps 500 --capture-video
.venv/bin/python scripts/validate_demo_d.py /path/to/local_turn_output/trace.csv
```

Confirm that `run_status.json` says `passed` before using a generated report; the runner writes `failed` there on a caught Isaac/Python exception. The MP4 in that local output is a live-camera artifact and is not part of the numerical acceptance criterion.

The room modes and `--freeze-cabinets` use the local Molonbot scene's named prims; another USD may require its own collision setup. The old direct-torque turn is still expected to fail, while the bounded-drive turn has passed only the documented profile and scene. If using `--settle-steps` above zero, note the [observed ovfmi 0.2 first-step input edge case](../results/first_step_probe.json); both published runs used zero settling steps and an initial zero-reference interval. The public traces and reports permit independent numerical replay, but full Isaac reproduction still requires a publishable articulated robot asset. That remains the portability gap.

The [standalone physics diagnostic](../scripts/diagnose_turn_physx.py) can isolate direct joint velocity commands from constant joint-effort commands, without running any FMU. `room_shell`, `no_semantic`, `no_maps`, and `no_cabinets` are controlled scene-ablation modes in the live runner; they are not alternative successful final scenes. They were used to identify the cabinet dynamics, while the final result uses the complete semantic scene with `--freeze-cabinets`.
