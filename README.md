# Sight2Servo

Sight2Servo is a learning project about how visual information becomes robot
motion. The goal is to build a simple vision-guided robotic arm in MuJoCo while
understanding the geometry, kinematics, control, and simulation concepts behind
each step.

## Current status

Milestone 0 is complete:

- Two-link planar arm in the x-y plane
- Shoulder and elbow hinge joints
- Torque motors for both joints
- Fixed red target and overhead camera
- Python state inspection and simulation stepping
- Basic smoke tests

Not implemented yet:

- Forward kinematics
- Inverse kinematics
- Feedback control
- Camera-frame processing and target detection
- Closed perception-control loop

## Setup

```bash
uv sync
```

## Run

Inspect and step the simulation state:

```bash
uv run python src/sight2servo/simulation.py
```

Open the MuJoCo viewer:

```bash
uv run python -m mujoco.viewer --mjcf=models/arm.xml
```

Run the tests:

```bash
uv run pytest -q
```
