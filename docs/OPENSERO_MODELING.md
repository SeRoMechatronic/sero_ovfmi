# Wheel-plant modeling in openSeRo / SeRo_MBE

These eight original screenshots were provided by the author in the `MolonbotWheelPlant.zip` archive and published here with the author's permission under the repository's [MIT License](../LICENSE). They show the author-owned SeRo_MBE/openSeRo interface used to build and export the included FMI 3.0 Co-Simulation wheel-plant FMU. The images are preserved unchanged; [SHA-256 values](../assets/opensero/screenshots/SHA256SUMS) let readers verify the copies. The private archive and editable modeling projects are not part of this public repository. See [FMU provenance](../FMU_NOTICE.md) for the matching archive and FMU hashes.

The screenshots explain the *modeling process*. They do not establish calibrated motor physics, real-robot performance, or independent numerical agreement. The [Demo A report](../results/demo_A/validation/report.json) and [four-wheel report](../results/drive/report.json) contain the separate, reproducible ovfmi-versus-FMPy checks.

## 1. Wheel-plant block model

The input voltage passes through a saturation block; the model then generates angular speed and angle using the declared gain and time constant.

![Block diagram of the authored wheel-plant model](../assets/opensero/screenshots/01_modelo_MolonbotWheelPlant.png)

## 2. Per-step motor response code

The model's C function implements the exact first-order response for a held input over one communication step and integrates wheel angle.

![C function implementing the exact wheel response](../assets/opensero/screenshots/02_codigo_motor_rueda_exacto.png)

## 3. Voltage saturation code

The separate limiter clamps the signed voltage to the configured magnitude before the wheel-response function.

![C function implementing signed voltage saturation](../assets/opensero/screenshots/03_codigo_limitar_tension.png)

## 4. Time-constant parameter

The screenshot records `tau_s = 0.25 s` as an FMI model parameter in the author's tool.

![Time-constant parameter editor with value 0.25 seconds](../assets/opensero/screenshots/04_parametro_tau_s.png)

## 5. FMI 3.0 Co-Simulation export

The export dialog shows FMI 3.0 Co-Simulation, a 0.01 s step, and Linux/Windows x64 binaries selected. The actual FMU metadata and binaries are also checked by the repository's automated tests.

![FMI 3.0 Co-Simulation export dialog](../assets/opensero/screenshots/05_exportar_fmu.png)

## 6. In-tool comparison bench

The bench connects the same voltage profile to a native model path and to the exported FMU, with scopes for speed, angle, and applied voltage.

![MIL versus FMU test-bench diagram in the author's tool](../assets/opensero/screenshots/06_banco_pruebas.png)

## 7. In-tool response plots

The speed, angle, and voltage traces are visually overlaid in the author's bench. This is useful context, not a substitute for the per-sample exported-FMU comparisons in the public CSV and JSON reports.

![In-tool speed, angle, and applied-voltage comparison plots](../assets/opensero/screenshots/07_banco_resultados.png)

## 8. Speed response cursors

The in-tool speed plot shows two cursor checkpoints near 0.75 s and 3.25 s. The repository's direct FMPy check records the numerical values separately.

![In-tool speed comparison with two cursor checkpoints](../assets/opensero/screenshots/08_omega_MIL_vs_FMU_cursores.png)
