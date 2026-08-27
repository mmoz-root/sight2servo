# Sight2Servo

Sight2Servo is a learning project about how visual information becomes robot
motion. The goal is to build a simple vision-guided robotic arm in MuJoCo while
understanding the geometry, kinematics, control, and simulation concepts behind
each step.

## Current status

Milestones 0-5 are complete.

Milestone 0 - MuJoCo model:

- Two-link planar arm with shoulder and elbow joints
- Torque motors, fixed target, and overhead camera
- Python state inspection, simulation stepping, and smoke tests

Milestone 1 - Forward kinematics:

- Manual two-link forward kinematics
- Known-angle tests and comparison against MuJoCo site positions

Milestone 2 - Inverse kinematics:

- Geometric reachability checks
- Both elbow configurations and singular-boundary handling
- IK verification through forward kinematics and MuJoCo

Milestone 3 - Feedback control:

- P and PD joint controllers
- Torque saturation at the actuator limits
- Two-joint control with dynamic coupling
- IK-driven PD control reaching a Cartesian target within 1 mm

Milestone 4 - Camera rendering:

- Reusable RGB rendering from the fixed MuJoCo overhead camera
- 640 x 480 frame extraction and saved-image verification
- Camera smoke test for frame shape, type, and content

Milestone 5 - Red-target detection:

- HSV color segmentation with both OpenCV red hue ranges
- Largest-contour centroid detection returning `(u, v)` or `None`
- Synthetic-image tests and MuJoCo camera integration test
- Saved detection image with the measured centroid marked

The test suite currently contains 27 passing tests.

Next milestones:

- Pixel-to-world coordinate mapping
- Closed vision-guided control loop
- Controlled experiments and final documentation

## Setup

```bash
uv sync
```

## Run

Inspect and step the simulation state:

```bash
uv run python src/sight2servo/simulation.py
```

Run the IK-driven PD control demo:

```bash
PYTHONPATH=src uv run python -m sight2servo.ik_pd_demo
```

Render and save an overhead camera frame:

```bash
PYTHONPATH=src uv run python -m sight2servo.camera_demo
```

Detect and mark the red target in a camera frame:

```bash
PYTHONPATH=src uv run python -m sight2servo.vision_demo
```

Open the MuJoCo viewer:

```bash
uv run python -m mujoco.viewer --mjcf=models/arm.xml
```

Run the tests:

```bash
uv run pytest -q
```
