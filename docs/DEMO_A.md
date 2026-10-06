# Demo A: numerical FMU validation and robot-only visualization

Demo A is one presentation package with two complementary experiments. It does **not** claim that the one-wheel numerical trace is the source of the robot video.

| Part | Purpose | Evidence |
|---|---|---|
| One-wheel validation | Test one openSeRo FMU's explicit USD → ovstage → ovfmi → FMU → ovstage mapping against direct FMPy and an analytical recurrence. | [USD stage](../scenes/demo_A_wheel_validation.usda), [500-step CSV](../results/demo_A/validation/trace.csv), [plot](../results/demo_A/validation/trace.svg), [JSON report](../results/demo_A/validation/report.json) |
| Robot-only replay | Show an offline robot trajectory derived from a **separate four-instance FMU run**. | [four-wheel trajectory](../results/drive/trajectory.csv), [four-wheel report](../results/drive/report.json), [animated USD](../scenes/demo_A_robot_only.usda), [Isaac Sim video](../results/demo_A/demo_A_robot_only.mp4), [render manifest](../results/demo_A/render_manifest.json) |

The one-wheel check runs 500 steps at 10 ms with a 0/6/0 V profile. The four-wheel drive run has 789 steps at 10 ms with turn, forward, and settling segments. Its trajectory is also reused by visual Demo B; this is why the shared four-wheel trace remains in `results/drive/` rather than being duplicated under Demo A. The video is a kinematic USD timeline render, not contact-physics or real-robot evidence.

Run `scripts/run_wheel_demo.py` to reproduce the one-wheel validation, then `scripts/run_four_wheel_motion.py` and `scripts/build_timeline_scenes.py` to reproduce the four-wheel trajectory and animation. See the [main README](../README.md) for pinned setup and Isaac Sim commands, and [methods/results](RESULTS.md) for measured values and limitations.

The plant FMU was created in the author's openSeRo/SeRo_MBE tool; see the [original modeling screenshots](OPENSERO_MODELING.md). These illustrate the model and its in-tool test bench. The numerical validation reported here is a separate check using the exported FMU, ovfmi, and direct FMPy.
