# Demo B: four-FMU motion in a synthetic room

Demo B combines a validated, offline four-wheel FMU trajectory with a separate check of the composed synthetic USD room. It is a numerical software demonstration and an Isaac Sim visualization, **not** a measured reconstruction of Molonbot's physical environment.

| Evidence | What it establishes |
|---|---|
| [Shared four-FMU report](../results/drive/report.json) and [trajectory CSV](../results/drive/trajectory.csv) | Four independent wheel-plant FMU instances were stepped for 789 × 10 ms and compared with direct FMPy. This is the same trajectory reused by Demo A's robot-only video. |
| [Demo B validation report](../results/demo_B/validation/report.json) | All 790 composed USD positions and headings match the source trajectory; four FMU prims and their mappings are present; coarse footprint-to-object/wall/floor margins are positive in the synthetic room. |
| [Animated USD](../scenes/demo_B_synthetic_room.usda) and [Isaac Sim video](../results/demo_B/demo_B_synthetic_room.mp4) | Offline kinematic replay of that trajectory within original synthetic room geometry. |

Reproduce the B-specific check with `.venv/bin/python scripts/validate_demo_b.py` after the pinned setup in the [main README](../README.md). The script checks hashes of the FMU, source trace, composed USD stage, and video against their published reports; it fails if they differ. It also checks every authored pose, each FMU mapping, and axis-aligned 2D clearances using the explicitly chosen 0.25 m circular footprint.

The minimum clearances are **software-model margins**, not real-world safety margins. The room is not sensor-derived, there is no contact dynamics or obstacle replanning, and the B validation does not constitute a second independent FMU run. Numerical methods and measurements are in [RESULTS.md](RESULTS.md).
