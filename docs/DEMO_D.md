# Demo D — live openSeRo PI control of Isaac Sim/PhysX wheel joints

Demo D is the first experiment here in which the FMU output **acts on the simulated articulated robot during the physics run**. It is not another playback of precomputed USD poses. Four instances of the author's FMI 3.0 PI-controller FMU receive the four wheel velocities measured by PhysX, through `ovstage` and `ovfmi`, every 10 ms. A declared synthetic voltage-to-torque adapter applies their commands to the four revolute wheel joints. Four separate wheel-plant FMUs run in parallel as **shadow predictions**; their outputs do not actuate PhysX.

This experiment uses the private local Molonbot USD and deliberately does **not** publish its mesh, room geometry, or rendered video in this MIT repository. The [signal-contract stage](../scenes/demo_D_live_fmi_contract.usda), [live runner](../scripts/run_demo_d_live_isaac.py), [FMU-side master](../scripts/demo_d_fmi_master.py), [320-step trace](../results/demo_D/trace.csv), [run report](../results/demo_D/report.json), and [independent FMPy replay result](../results/demo_D/replay_validation.json) are public. The local video was captured from the live camera stream during simulation, not reconstructed from the CSV, but is held outside this repository pending an asset-publication decision.

## Architecture and explicit delay

Isaac Sim 6.0.1 uses Python 3.12, while the pinned ovstage/ovfmi environment here uses Python 3.10. The two processes exchange one JSON line per simulation step over local standard I/O. No ROS bridge, network socket, Jetson, or physical motor interface is imported by this demo. The Isaac process disables the legacy ROS action graph in a USD *session layer* without editing the source asset.

At step `k`, PhysX supplies the measured wheel speeds and the master writes those values and the desired references into the four controller USD state prims. It also feeds each shadow plant the controller voltage from step `k-1`. `FmiHost` reads the ovstage inputs, advances all eight FMU instances by 10 ms, and publishes their outputs. Isaac converts the **previous** controller voltage into joint torque, advances PhysX by 10 ms, and logs the newly measured velocities and robot pose. This is a one-step communication delay, not an instantaneous electromechanical coupling.

The simulated motor law is `torque = clip(0.004 * (previous_voltage - 0.20 * measured_omega), -0.05, +0.05)` in N·m. These three constants are **chosen demonstration values**, not identified from Molonbot hardware. Wheel-drive stiffness and damping are set to zero in memory for effort control; the source USD is unchanged. The local room can be disabled in a session-only `clear` mode while retaining the ground and articulated robot, or retained in `full` mode.

## Measured result and what it does *not* establish

The recorded `clear` run completed 320 physics steps (3.2 s) and 1,280 wheel samples with a +4/0/−4/0 rad/s profile. All four measured wheel speeds changed sign; the maximum wheel speed was **2.934 rad/s**, the maximum command was **4.690 V**, and the base reached **0.152 m** from its initial location before moving back. Thirty frames were captured directly from the live Isaac camera. The [replay validator](../scripts/validate_demo_d.py) independently stepped eight FMU instances with FMPy using the logged references and measured PhysX speeds. Its maximum difference from the ovfmi-published controller and shadow-plant signals was **2.3842×10⁻⁷**, below the declared `1×10⁻⁴` tolerance. It also checked the one-step signal continuity, torque equation, 12 V command bound, and trace hash. This validates the FMI/ovfmi coupling numerically; it does **not** validate the motor model against physical data.

In a separate `full` room run, the maximum wheel-speed spread rose from `0.097` to `16.502 rad/s`. Nearby geometry is a plausible cause, but no contact-event trace was recorded, so that remains an **inference**. The turn-in-place diagnostic also **failed**: opposite left/right references produced controller voltages, but the simulated base did not turn appreciably. These are open model/contact issues, not demonstrated capabilities. The private attempted public surrogate was discarded after it exhibited unstable joints. No claim is made for obstacle-aware navigation, mecanum lateral motion, robust turning, real-time scheduling, hardware-in-the-loop, or real-robot safety.

## Reproduce what is public

The checked-in trace can be verified without Isaac Sim or any private asset:

```bash
.venv/bin/python scripts/validate_demo_d.py results/demo_D/trace.csv
.venv/bin/python -m unittest tests.test_demo_d -v
```

To rerun the live physics experiment, supply your own licensed, compatible USD stage with `/World/jetauto`, an actual PhysX articulation, a ground collider, and the four joint names documented in the report. With Isaac Sim 6.0.1 installed, use:

```bash
.venv/bin/python scripts/build_demo_d_fmi_scene.py
/path/to/isaacsim/python.sh scripts/run_demo_d_live_isaac.py \
  --scene /path/to/your/compatible_robot_scene.usd \
  --output /path/to/local_output \
  --environment clear --steps 320 --capture-video
.venv/bin/python scripts/validate_demo_d.py /path/to/local_output/trace.csv
```

Confirm that `run_status.json` says `passed` before using a generated report; the runner writes `failed` there on a caught Isaac/Python exception. The MP4 in that local output is a live-camera artifact and is not part of the numerical acceptance criterion.

`--environment clear` is specific to the local Molonbot scene's named room prims; other scenes simply retain their own geometry. The optional `--profile turn` is a **diagnostic that currently fails** on the tested local articulation. If using `--settle-steps` above zero, note the [observed ovfmi 0.2 first-step input edge case](../results/first_step_probe.json); the published run used zero settling steps and an initial zero-reference interval. The public trace and report permit independent numerical replay, but full Isaac reproduction still requires a publishable articulated robot asset. That is the remaining portability gap.
