# Demo C — exact FMU contract for a closed-loop ovfmi experiment

Status: **specification only**. Demo C has not been run because the controller FMU does not yet exist. This document is the export contract for openSeRo and the acceptance plan for the subsequent experiment.

## How many FMUs?

Exactly **two distinct FMU models** are needed for the first closed-loop demonstration:

1. **Plant:** reuse the included `MolonbotWheelPlant.fmu`. No new plant FMU is required. It accepts a voltage command and reports applied voltage, angular speed, and angle.
2. **Controller:** create **one new** `MolonbotWheelPIController.fmu` in openSeRo, as specified below. It accepts a speed reference and measured speed and reports a bounded voltage command.

The minimal test uses **two instances total** (one controller, one plant). After it passes, a four-wheel extension can instantiate four copies of each model (**eight instances**, still only two FMU *files*). Do not build four different controller files unless wheel-specific dynamics justify them. An improved/calibrated plant would be a separate, optional third model—not a prerequisite for Demo C.

## Required export properties

- FMI **3.0 Co-Simulation** (`<CoSimulation>`), not only Model Exchange or Scheduled Execution. If openSeRo exports additional FMI modes, Co-Simulation must still be present and tested.
- A Linux `x86_64-linux` shared library in the FMU; a Windows `x86_64-windows` binary is useful but optional for our Linux test. The binary must match `modelIdentifier` in `modelDescription.xml`.
- All signals below are scalar `Float64`. Use unique value references, correct `causality`/`variability`, `<ModelStructure><Output ...>` entries, and a `DefaultExperiment` with `stepSize="0.01"`. Include `<BaseUnit>` definitions and clear declared units where supported by openSeRo.
- The controller must support repeated `fmi3DoStep(..., communicationStepSize=0.01)` calls from time 0 to at least 10 s; no external file, network service, Python process, ROS topic, or real robot is needed. Deterministic runs from identical initialization are expected.
- If only a fixed 10 ms step is supported, declare that honestly. Do not set `canHandleVariableCommunicationStepSize="true"` unless tested.

## Controller input/output contract

| Variable | FMI causality | Type / unit | Start | Meaning |
|---|---|---|---:|---|
| `omega_ref_rad_s` | input | Float64, rad/s | 0 | Desired wheel angular speed at the communication point. May be positive or negative. |
| `omega_meas_rad_s` | input | Float64, rad/s | 0 | Most recently published plant speed. |
| `voltage_cmd_V` | output | Float64, V | 0 | Signed, saturated command for the wheel plant. |
| `error_rad_s` | output | Float64, rad/s | 0 | `omega_ref_rad_s - omega_meas_rad_s`. |
| `integral_V` | output | Float64, V | 0 | Internal integral contribution, exposed for diagnosis. |

Recommended fixed or tunable `Float64` parameters, with these defaults:

| Parameter | Unit | Default | Constraint |
|---|---|---:|---|
| `kp_V_per_rad_s` | V/(rad/s) | `0.4` | `>= 0` |
| `ki_V_per_rad` | V/rad | `1.2` | `>= 0` |
| `voltage_limit_V` | V | `12.0` | `> 0` |
| `antiwindup_gain_per_s` | 1/s | `4.0` | `>= 0` |

The exact names above are part of the contract: do not rename them without updating the USD mappings and tests. `voltage_cmd_V` must be the controller output; it must **not** move a physical robot. A `saturation_flag` output is optional, but if provided, specify its FMI type and semantics. Do not add a clock or Scheduled Execution requirement to the first CS version.

## Unambiguous controller algorithm

At initialization, set integral state `I = 0 V`, `voltage_cmd_V = 0 V`, and both other outputs to zero. At every successful 10 ms Co-Simulation step, take the inputs held at that step's communication point and compute:

```text
h = 0.01 s
e = omega_ref_rad_s - omega_meas_rad_s
u_raw = kp_V_per_rad_s * e + I
u = clamp(u_raw, -voltage_limit_V, +voltage_limit_V)
I_next = clamp(I + ki_V_per_rad * h * e
                 + antiwindup_gain_per_s * h * (u - u_raw),
               -voltage_limit_V, +voltage_limit_V)
error_rad_s = e
voltage_cmd_V = u
integral_V = I_next
I = I_next
```

Use the pre-update `I` for `u_raw`; that ordering makes the expected first output reproducible. If the tool implements a different update ordering, tell us and document it before testing. Reject non-finite inputs/parameters explicitly; `voltage_cmd_V` must remain finite and within the voltage limit. A zero reference with zero measured speed must produce exactly zero command from the initialized state.

## How we will connect the two FMUs through ovfmi

Both FMUs will be declared as `FmuInstance` prims in one USD stage, with explicit `FmuMapping` entries to distinct USD state prims. A Python master will publish inputs in ovstage, call `update_from_ovstage()`, advance both FMUs with `step_sync(0.01)`, call `write_to_ovstage()`, and read back outputs. The loop has a **documented one-step communication delay** to avoid an implicit algebraic loop:

```text
At step k, before step_sync:
  controller.omega_ref  <- reference[k]
  controller.omega_meas <- plant.omega published after step k-1
  plant.voltage_cmd     <- controller.voltage published after step k-1
  (all values are 0 at initialization)
Step both FMUs over [t_k, t_(k+1)] in one host call.
Read new controller voltage and plant speed from ovstage for step k+1.
```

This schedule is deliberately discrete and reproducible. It does **not** claim instantaneous feedback or zero-delay physical control. It also avoids the [observed ovfmi 0.2 first-step input edge case](../results/first_step_probe.json) by starting at a zero reference and applying the first nonzero step only after explicit zero-input initialization steps. If ovfmi changes its initialization behavior in a later version, the exact schedule will still be tested rather than assumed.

## Planned evidence and acceptance tests

1. Validate each FMU independently with FMPy; check FMI mode, binary platform, variables, units, finite outputs, and hash. Verify the controller's first-step equation directly before involving ovfmi.
2. Run the same reference profile through (a) direct FMPy orchestration and (b) ovstage → ovfmi → both FMUs → ovstage. Compare every published signal at every step with a declared tolerance. Check the output mapping directions and ordinal ordering, not only the final value.
3. Profile: 0 rad/s for 0.2 s; +8 rad/s for 3 s; 0 rad/s for 1 s; -6 rad/s for 2 s; then +30 rad/s for 1 s to exercise saturation; finish at 0. Log at 10 ms resolution. Record `reference`, `measurement`, `error`, `command`, `applied voltage`, `angle`, `integral`, step status, and time.
4. Check command bounds `|voltage_cmd_V| <= 12 V`, sign changes, deterministic repeatability, and controller–plant/FMPy agreement. Assess settling time, overshoot, steady-state error, and saturation recovery from the actual trace; do **not** promise numerical limits before the exported controller is tested.
5. Negative tests: disconnected mapping, wrong variable name, missing FMU, invalid parameter, NaN input, wrong start value, and, if supported, an FMU step failure. Report the actual failure modes rather than silently replacing values.
6. Produce English CSV/JSON/SVG evidence, pinned environment, exact FMU hashes, a minimal USD scene, and a small Isaac Sim visualization. Separate FMU computation time from rendering; do not claim a performance advantage without repeated controlled measurements.

The result would demonstrate explicit closed-loop orchestration and a clear USD signal contract. It would still **not** validate Molonbot's real motors or justify deployment on hardware. A later Scheduled Execution or SSP experiment can be added as its own study after the Co-Simulation baseline passes.

## What to send us from openSeRo

Send the new `.fmu` and, if possible, its generating openSeRo model/project file, parameter values, export log, and a short note stating whether the algorithm and variable names above were followed exactly. The FMU alone is enough to begin automated inspection and direct FMPy tests; the model source improves reproducibility. Do not include passwords, private email, or real robot imagery in the upload.

Reference: [FMI 3.0.2 Co-Simulation specification](https://fmi-standard.org/docs/3.0.2/).
