# SeRo × ovfmi — reproducible FMI 3.0 / OpenUSD demonstration

This repository is a small, public, MIT-licensed experiment showing **actual `ovfmi` use**, not just a rendered digital twin. A wheel FMU made with **openSeRo** is declared in USD, discovered by `FmiHost`, advanced through `ovfmi`, and read back through `ovstage`. Every step is checked against an independent direct-FMPy run. A four-instance extension produces an offline robot trajectory and two timeline-playable Isaac Sim scenes.

The geometry is **original and synthetic**. No Molonbot robot mesh, observed room snapshot, private email, personal imagery, ROS bag, or third-party NVIDIA asset is included. The FMU is a didactic first-order model, not a calibrated physical motor. Nothing here commands a real robot.

## Watch the two visual demos

| Demo | Video | Animated OpenUSD stage |
|---|---|---|
| A — robot only | [MP4](results/demo_A/demo_A_robot_only.mp4) | [USD](scenes/demo_A_robot_only.usda) |
| B — robot in a synthetic room | [MP4](results/demo_B/demo_B_synthetic_room.mp4) | [USD](scenes/demo_B_synthetic_room.usda) |

Both USD stages contain the same **790 authored robot poses** over 7.89 s at 100 time codes/s. Open either in Isaac Sim and press **Play** in the Timeline. The videos are English-labelled H.264 renders (96 frames, 960×540, 12 fps). Motion is computed from four FMU wheel-angle outputs using a declared no-slip differential-drive assumption; Isaac Sim renders the resulting USD animation. It does **not** solve wheel contacts or prove physical-robot behavior. The synthetic wheel meshes do not spin.

The [two-video page](results/index.html) is convenient for local playback after cloning.

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

The [one-wheel trace](results/demo_a/trace.csv), [signal plot](results/demo_a/trace.svg), [one-wheel report](results/demo_a/report.json), [four-wheel trajectory](results/drive/trajectory.csv), [four-wheel report](results/drive/report.json), and [methods/results note](docs/RESULTS.md) are included. The small `ovfmi`–FMPy differences are consistent with `ovfmi` 0.2 publishing ordinary outputs as float32, while the FMU declares Float64. These are numerical correctness comparisons, **not** a controlled performance benchmark.

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
.venv/bin/python -m unittest discover -s tests -v
```

To regenerate the two videos, run the next commands with a working Isaac Sim 6.0.1 installation; replace `/path/to/isaacsim` with its actual location:

```bash
/path/to/isaacsim/python.sh scripts/render_timeline_demos_isaac.py --demo A
/path/to/isaacsim/python.sh scripts/render_timeline_demos_isaac.py --demo B
python3 scripts/encode_timeline_demos.py
```

For immediate, looping GUI playback with a framed camera, use `/path/to/isaacsim/python.sh scripts/open_timeline_demo_isaac.py --demo A` or `--demo B`. This opens only the synthetic offline scene; it does not connect to ROS or hardware.

The scene paths are relative. `scripts/inspect_drive_geometry.py` runs in a separate Python process because the tested `ovstage` and `usd-core` packages load different USD libraries. The checked wheel radius, track, obstacles, and walls are **chosen synthetic software-test values**, not measurements or safety margins for a robot.

## How the FMI mapping works

In [the minimal stage](scenes/demo_a_wheel.usda), `FmuInstance` points to [the openSeRo FMU](fmus/MolonbotWheelPlant.fmu). A `FmuConnection` targets `/World/WheelState`; its four `FmuMapping` prims route `voltage_cmd_V` into the FMU and `voltage_applied_V`, `omega_rad_s`, and `theta_rad` back to USD. The [runner](scripts/run_wheel_demo.py) writes the 0/6/0 V profile into ovstage, calls `update_from_ovstage`, `step_sync(0.01)`, and `write_to_ovstage`, then reads every output group and compares it with direct FMPy and an analytical recurrence.

In [the four-wheel stage](scenes/demo_movimiento_4ruedas.usda), four separate `FmuInstance` prims map to FL/FR/BL/BR state prims. The [four-wheel runner](scripts/run_four_wheel_motion.py) checks every instance against its own direct FMPy reference, integrates the robot pose from published angle increments, and writes the trace. [The animation builder](scripts/build_timeline_scenes.py) then bakes those poses into a reusable USD layer shared by visual Demos A and B. This explicit adapter is why the robot moves; `ovfmi` does not automatically drive Isaac articulations.

## Next experiment: closed-loop Demo C

Only **one new FMU** is needed: an FMI 3.0 Co-Simulation wheel PI controller. Reuse the included wheel-plant FMU. The exact variable names, units, update equation, one-step coupling schedule, export requirements, and acceptance tests are in [the Demo C FMU specification](docs/DEMO_C_FMU_SPEC.md). It is a proposal, **not a completed result**.

## License and limits

The repository, including the author-owned openSeRo FMU, is licensed under [MIT](LICENSE). The FMU archive contains generated C source and Linux/Windows x86-64 binaries; see [FMU provenance](FMU_NOTICE.md). This repository does not claim real-time behavior, FMI 3 Scheduled Execution support, automatic PhysX coupling, SLAM, Nav2, calibrated digital-twin physics, or hardware safety. No part of it is intended for physical actuation.
