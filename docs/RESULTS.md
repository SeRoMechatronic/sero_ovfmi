# Methods, measurements, and limitations

Recorded 6 October 2026 on Linux x86-64 with Python 3.10, `ovfmi 0.2.0`, `ovstage 0.2.0.377349`, `FMPy 0.3.25`, and the included openSeRo-generated FMI 3.0 Co-Simulation FMU. The [lock file](../requirements.lock) lists exact Python packages. The experiment is offline and synthetic. The reported one-process wall times are exploratory, not a controlled benchmark.

## Model and mapping

The FMU ZIP has SHA-256 `5cdb7566ef9dcffbca455a31fad3588fde8063e535002ec8899454965d695f76` and contains Linux and Windows x86-64 binaries plus generated C source. Its relevant `modelDescription.xml` variables are scalar Float64:

| USD attribute | FMU variable | Direction | Unit |
|---|---|---|---|
| `molonbot:voltage_cmd_V` | `voltage_cmd_V` | USD → FMU | V |
| `molonbot:voltage_applied_V` | `voltage_applied_V` | FMU → USD | V |
| `molonbot:omega_rad_s` | `omega_rad_s` | FMU → USD | rad/s |
| `molonbot:theta_rad` | `theta_rad` | FMU → USD | rad |

The first-order teaching model has default speed gain 2 rad/s per V, 0.25 s time constant, and ±12 V saturation. For constant input over a 10 ms communication step, the independent reference integrates the exact first-order recurrence. The existing FMU declares named units but not explicit `<BaseUnit>` decompositions; that is an interoperability improvement for a future export, not a reason to claim this FMU is invalid. FMPy's FMU validator returned no issues in the tested environment.

## Demo A: one-wheel numerical validation

The [minimal USD](../scenes/demo_A_wheel_validation.usda) declares one `FmuInstance`, one state prim, and four explicit mapping prims. The [runner](../scripts/run_wheel_demo.py) supplies 0 V for 0–0.5 s, 6 V for 0.5–3 s, and 0 V for 3–5 s. At each of 500 10 ms steps it writes the command to ovstage, calls `FmiHost.update_from_ovstage`, `step_sync`, and `write_to_ovstage`, and reads the three published outputs. In parallel, the same FMU is stepped directly by FMPy; speed and angle are also compared with the analytical recurrence.

| Observation | Result |
|---|---:|
| Completed steps | 500 |
| Maximum absolute ovfmi vs direct FMPy output difference | `9.484643150869942e-7` |
| Maximum absolute ovstage vs ovfmi output difference | `0` |
| Maximum direct FMPy vs analytical speed/angle difference | `0` |
| `omega(0.75 s)` direct FMPy | `7.585446705942695 rad/s` |
| `omega(3.25 s)` direct FMPy | `4.414352873647819 rad/s` |
| SHA-256 of the 500-step CSV | `00afc3189fc92b7a6a4562146d9ba8a08470897ffe0fad4d13ae5d9286612ede` |

The direct FMU variables are Float64; ordinary ovfmi 0.2 output groups observed here are float32. The measured sub-micro-unit difference is consistent with that conversion. This is an observation of the installed version and route, not a claim about all FMI implementations. The [CSV](../results/demo_A/validation/trace.csv) and [JSON report](../results/demo_A/validation/report.json) give each sample and the exact scene/FMU hashes. A repeat run in the same environment produced the same CSV hash.

## First-step input edge case

The [isolated reproducer](../scripts/probe_initial_input.py) authors 0 V in USD, attaches the FMU, writes 6 V to ovstage before the first step, and invokes `update_from_ovstage` then one 10 ms `step_sync`. Direct FMU evaluation predicts `omega=0.4705267301721219 rad/s`; ovfmi 0.2 in this configuration returned `0`. The [JSON](../results/first_step_probe.json) records both numbers and `first_step_update_applied=false`. The main profiles deliberately start with 0 V, so later changes still test the mapping and do not disguise this edge case. We did not patch ovfmi or claim a general bug in versions we have not tested.

## Four independent FMUs and visual replay for Demos A and B

The [four-wheel USD](../scenes/demo_movimiento_4ruedas.usda) declares four independent `FmuInstance` prims (FL/FR/BL/BR) and four state prims. The [runner](../scripts/run_four_wheel_motion.py) reads every published output and compares each instance with a distinct direct-FMPy instance. Commands are 20 zero-input steps, 92 opposite-polarity turn steps at ±1.5 V, 150 settling steps, 377 forward steps at +6 V, and 150 final settling steps. Each wheel-angle increment contributes to an explicit no-slip differential-drive pose integrator.

The nominal wheel radius `0.048671331 m`, track `0.172250003 m`, initial pose `(0, 0, 0)`, and target heading `-1.553688281 rad` are **chosen software-test parameters**. They are not measured robot dimensions. The [original synthetic room](../scenes/synthetic_room.usda) uses USD primitive boxes; no private/reconstructed room data is present. Its obstacle AABB check is an internal software consistency check, not a hardware safety analysis.

| Observation | Result |
|---|---:|
| FMU instances / completed 10 ms steps | 4 / 789 |
| Maximum ovfmi vs independent FMPy output difference | `1.8920591244864227e-6` |
| Same-side front/rear output difference | `0` |
| Final projected synthetic aisle travel | `2.2014892365 m` |
| Final target-heading error | `-0.00605492175 rad` |
| Final lateral deviation | `-0.0132549354 m` |
| Minimum synthetic AABB clearance after chosen footprint | `0.6541453419 m` |
| SHA-256 of the 789-step trajectory CSV | `0e37b35c5a0824aa10b4f1fa01a2644dee1e535b6e32177d504d3052be05e592` |

The [trajectory CSV](../results/drive/trajectory.csv) and [JSON report](../results/drive/report.json) contain commands, angles, speeds, pose, geometry assumptions, hashes, and limitations. The [animation builder](../scripts/build_timeline_scenes.py) writes 790 time-sampled robot translations/orientations (including time zero) into `scenes/robot_motion.usda`. Demos [A](../scenes/demo_A_robot_only.usda) and [B](../scenes/demo_B_synthetic_room.usda) compose that same layer. Isaac Sim 6.0.1 rendered [A](../results/demo_A/demo_A_robot_only.mp4) and [B](../results/demo_B/demo_B_synthetic_room.mp4); the respective render/video JSON manifests verify frame count, resolution, source-scene hash, and trace hash. This four-wheel trajectory is a different run from Demo A's single-wheel validation above. No physical robot or ROS system was connected.

## Demo B: composed-scene numerical validation

The [Demo B validator](../scripts/validate_demo_b.py) checks the **composed B stage** against all 789 rows of the shared four-FMU trajectory plus the initial pose, not just selected frames. It verifies 4 FMU instances and 4 explicit signal mappings per instance, local USD dependencies, trace/scene/video hashes, and 2D distances from every pose to the synthetic objects, wall boxes, and floor edge using a declared 0.25 m circular footprint. The [B-specific JSON report](../results/demo_B/validation/report.json) records the results. The reported ovfmi–FMPy wheel-output difference is imported from the **same shared four-FMU run** used by the A/B visual replay; it is not presented as a new or independent FMU experiment.

| Demo B observation | Result |
|---|---:|
| Composed poses checked against the source CSV | 790 / 790 |
| Maximum position / heading difference | `0 m` / `0 rad` |
| Minimum semantic-object AABB margin after chosen footprint | `0.6541453419 m` |
| Minimum wall AABB margin after chosen footprint | `0.76 m` |
| Minimum floor-edge margin after chosen footprint | `0.8 m` |

These are deterministic software and coarse axis-aligned geometry checks on an **original synthetic room**. They do not show that the scene matches a real room, that PhysX contact is correct, or that a physical robot can traverse the path safely. See the [Demo B guide](DEMO_B.md) for the evidence chain.

## What these measurements do not establish

- They do not validate a real motor, battery, wheel radius, room geometry, localization estimate, obstacle avoidance, or Nav2 behavior.
- USD pose animation is not PhysX traction/contact simulation. The wheel meshes do not rotate in the videos.
- FMU numerical agreement is not a latency or throughput benchmark. The single-run `attach_ms`, median, and p95 fields in the JSON are exploratory only.
- No controller FMU, closed-loop Demo C, FMI Scheduled Execution, SSP packaging, or ovfmi 0.3 comparison has been tested here. See the [separate Demo C export contract](DEMO_C_FMU_SPEC.md).

Primary references: [ovfmi package page](https://pypi.org/project/ovfmi/) and [FMI 3.0.2 specification](https://fmi-standard.org/docs/3.0.2/).
