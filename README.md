# SeRo × ovfmi — reproducible FMI 3.0 / OpenUSD demonstration

This repository is a small, public, MIT-licensed experiment showing **actual `ovfmi` use**, not just a rendered digital twin. Wheel-plant and PI-controller FMUs made with **openSeRo** are declared in USD, discovered by `FmiHost`, advanced through `ovfmi`, and read back through `ovstage`. Every step is checked against an independent direct-FMPy run. The results include numerical/visual Demos A and B, a two-FMU closed-loop Demo C, and a live Isaac Sim/PhysX wheel-control experiment (Demo D).

The geometry is **original and synthetic**. No Molonbot robot mesh, observed room snapshot, private email, personal imagery, ROS bag, or third-party NVIDIA asset is included. The FMU is a didactic first-order model, not a calibrated physical motor. Nothing here commands a real robot.

## Authored in openSeRo / SeRo_MBE

The wheel-plant FMU was modeled and exported in the author's own tool. This original screenshot shows the voltage input, saturation, first-order wheel response, model parameters, and speed/angle outputs:

![Wheel-plant block model in the author's SeRo_MBE/openSeRo interface](assets/opensero/screenshots/01_modelo_MolonbotWheelPlant.png)

The [plant modeling gallery](docs/OPENSERO_MODELING.md) and [controller modeling gallery](docs/DEMO_C.md#authorship-opensero-model-and-bench) show function code, FMI 3.0 Co-Simulation export settings, and in-tool MIL-versus-FMU plots. The screenshots document the author's workflow; the independently checked numerical results below come from the included FMUs, `ovfmi`, and FMPy.

## Demo A: numerical validation and robot-only visualization

Demo A now groups both deliverables under one name: the [single-wheel signal validation](results/demo_A/validation/report.json) and the robot-only Isaac Sim video below. These are **complementary but separate runs**: the 500-step single-wheel test checks the USD/ovfmi/FMU signal route against FMPy, while the 789-step four-wheel run generates the trajectory used by the video. The single-wheel trace is not a direct measurement of the animated four-wheel run. See the [Demo A guide](docs/DEMO_A.md) for the exact relationship and files.

## Watch the three visual demos

| Demo | Video | Animated OpenUSD stage | Numerical validation |
|---|---|---|---|
| A — robot only | [MP4](results/demo_A/demo_A_robot_only.mp4) | [USD](scenes/demo_A_robot_only.usda) | [Single-wheel FMU mapping](docs/DEMO_A.md) |
| B — robot in a synthetic room | [MP4](results/demo_B/demo_B_synthetic_room.mp4) | [USD](scenes/demo_B_synthetic_room.usda) | [Four-wheel trace + scene/geometry check](docs/DEMO_B.md) |
| C — PI controller + wheel plant | [MP4](results/demo_C/demo_C_closed_loop.mp4) | [USD](scenes/demo_C_wheel_visual.usda) | [1,000-step two-FMU closed loop](docs/DEMO_C.md) |

Demos A and B contain the same **790 authored robot poses** over 7.89 s at 100 time codes/s. Their videos are English-labelled H.264 renders (96 frames, 960×540, 12 fps). Motion is computed from four FMU wheel-angle outputs using a declared no-slip differential-drive assumption. Demo C instead shows a wheel-angle indicator and two signal bars for a 10-second closed-loop trace (61 rendered frames). In A/B/C, Isaac Sim replays authored USD data; those videos do **not** solve wheel contacts. Demo D is different: FMU commands actuate four PhysX joints during the run, and measured wheel speed is fed back to the FMUs. No demo proves physical-robot behavior.

The [three-video page](results/index.html) is convenient for local playback after cloning.

## Numerical evidence

| Check | Recorded result |
|---|---:|
| Single-wheel USD → ovstage → ovfmi → FMU → ovstage run | 500 successful 10 ms steps |
| Maximum single-wheel `ovfmi` vs direct FMPy output difference | `9.484643150869942e-7` |
| Maximum `ovstage` vs `ovfmi` output difference | `0` |
| Four independent FMU instances / completed steps | 4 / 789 |
| Maximum four-wheel `ovfmi` vs direct FMPy output difference | `1.8920591244864227e-6` |
| Repeat-run trajectory hash | identical in the tested environment |
| Synthetic aisle travel / final heading error | `2.20149 m` / `-0.00605 rad` |
| Demo B USD poses matched to four-wheel trace | 790 / 790; maximum position and heading errors `0` |
| Demo B minimum synthetic object / wall / floor margins after chosen footprint | `0.6541 m` / `0.76 m` / `0.8 m` |
| Demo C two-FMU, 10 ms closed-loop steps | `1,000` |
| Demo C maximum ovfmi vs direct FMPy output difference | `1.9025629498514718e-6` |
| Demo C maximum controller voltage / +30 rad/s saturation time | `12 V` / `0.75 s` |
| Demo D live PhysX steps / four-wheel samples | `320` / `1,280` |
| Demo D maximum ovfmi vs independent FMPy replay difference | `2.384185791015625e-7` |
| Demo D single-ground room-run maximum wheel speed / excursion from start | `2.934 rad/s` / `0.152 m` |

The [Demo A one-wheel trace](results/demo_A/validation/trace.csv), [four-wheel trajectory](results/drive/trajectory.csv), [Demo B scene-validation report](results/demo_B/validation/report.json), [Demo C closed-loop trace and plot](docs/DEMO_C.md), and [methods/results note](docs/RESULTS.md) are included. Demo B reuses the **same** four-FMU/FMPy comparison that generates both A/B visual replays; Demo C is a new two-FMU simulation. The small `ovfmi`–FMPy differences are consistent with `ovfmi` 0.2 publishing ordinary outputs as float32, while the FMUs declare Float64. These are numerical correctness comparisons, **not** a controlled performance benchmark.

The [first-step probe](results/first_step_probe.json) documents a version-specific negative finding: with `ovfmi==0.2.0`, changing an input in ovstage after attachment but before the first step did not affect that first step in this configuration. The main profiles initialize at 0 V and change later, so they do not hide that behavior.

## Reproduce on Linux x86-64

The pinned software was tested with Python 3.10, `ovfmi==0.2.0`, `ovstage==0.2.0.377349`, `FMPy==0.3.25`, and `usd-core==26.8`. A Linux x86-64 FMU binary is included. Isaac Sim is **not** needed for the numerical tests.

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install -r requirements.lock
.venv/bin/python -m pip check
.venv/bin/python scripts/run_wheel_demo.py
.venv/bin/python scripts/plot_wheel_trace.py
.venv/bin/python scripts/probe_initial_input.py
.venv/bin/python scripts/run_four_wheel_motion.py
.venv/bin/python scripts/build_timeline_scenes.py
.venv/bin/python scripts/validate_demo_b.py
.venv/bin/python scripts/validate_demo_c_scene.py
.venv/bin/python scripts/run_demo_c.py
.venv/bin/python scripts/plot_demo_c.py
.venv/bin/python scripts/build_demo_c_visual.py
.venv/bin/python -m unittest discover -s tests -v
```

To regenerate the two videos, run the next commands with a working Isaac Sim 6.0.1 installation; replace `/path/to/isaacsim` with its actual location:

```bash
/path/to/isaacsim/python.sh scripts/render_timeline_demos_isaac.py --demo A
/path/to/isaacsim/python.sh scripts/render_timeline_demos_isaac.py --demo B
python3 scripts/encode_timeline_demos.py
```

For Demo C's offline signal visualization, run `/path/to/isaacsim/python.sh scripts/render_demo_c_isaac.py` then `python3 scripts/encode_demo_c.py` (system Python with OpenCV). Open any animated USD in Isaac Sim and press **Play** in the Timeline. For immediate A/B GUI playback with a framed camera, use `/path/to/isaacsim/python.sh scripts/open_timeline_demo_isaac.py --demo A` or `--demo B`. None of these commands connects to ROS or hardware.

The scene paths are relative. `scripts/inspect_drive_geometry.py` runs in a separate Python process because the tested `ovstage` and `usd-core` packages load different USD libraries. The checked wheel radius, track, obstacles, and walls are **chosen synthetic software-test values**, not measurements or safety margins for a robot.

## How the FMI mapping works

In [Demo A's minimal validation stage](scenes/demo_A_wheel_validation.usda), `FmuInstance` points to [the openSeRo FMU](fmus/MolonbotWheelPlant.fmu). A `FmuConnection` targets `/World/WheelState`; its four `FmuMapping` prims route `voltage_cmd_V` into the FMU and `voltage_applied_V`, `omega_rad_s`, and `theta_rad` back to USD. The [runner](scripts/run_wheel_demo.py) writes the 0/6/0 V profile into ovstage, calls `update_from_ovstage`, `step_sync(0.01)`, and `write_to_ovstage`, then reads every output group and compares it with direct FMPy and an analytical recurrence.

In [the four-wheel stage](scenes/demo_movimiento_4ruedas.usda), four separate `FmuInstance` prims map to FL/FR/BL/BR state prims. The [four-wheel runner](scripts/run_four_wheel_motion.py) checks every instance against its own direct FMPy reference, integrates the robot pose from published angle increments, and writes the trace. [The animation builder](scripts/build_timeline_scenes.py) then bakes those poses into a reusable USD layer shared by visual Demos A and B. This explicit adapter is why the robot moves; `ovfmi` does not automatically drive Isaac articulations.

## Demo C: closed-loop coupling

The new FMI 3.0 Co-Simulation [PI-controller FMU](fmus/MolonbotWheelPIController.fmu) and the existing wheel-plant FMU now run as two mapped instances in [one USD stage](scenes/demo_C_closed_loop.usda). The explicit one-step communication delay avoids an implicit algebraic loop. The [Demo C guide](docs/DEMO_C.md) separates the completed evidence, author modeling screenshots, limitations, and reproduction commands; the [original export contract](docs/DEMO_C_FMU_SPEC.md) remains available for comparison.

## Demo D: live Isaac Sim/PhysX feedback

Four PI-controller FMU instances command simulated joint efforts through a declared, **uncalibrated** voltage-to-torque adapter while Isaac Sim/PhysX measures all four wheel speeds every 10 ms. Four wheel-plant FMUs are separate shadow predictions, not the controlled physics plant. The [Demo D guide](docs/DEMO_D.md) provides the process boundary, code, public 320-step trace, independent FMPy replay, and the precise limitations. The local articulated robot and room assets, and the live video containing their imagery, are not included in this public repository. Forward/reverse was demonstrated with the room loaded after a session-only fix for its duplicate ground collider; turn-in-place did not pass. Nothing was sent to physical hardware.

## License and limits

The repository, including both author-owned openSeRo FMUs and the 16 author-provided modeling screenshots, is licensed under [MIT](LICENSE). The FMU archives contain generated C source and Linux/Windows x86-64 binaries; see [FMU and screenshot provenance](FMU_NOTICE.md). This repository does not claim real-time behavior, FMI 3 Scheduled Execution support, automatic PhysX coupling without the explicit Demo D adapter, SLAM, Nav2, calibrated digital-twin physics, or hardware safety. No part of it is intended for physical actuation.
