# Development Log

Sight2Servo was built incrementally so that each robotics concept could be
understood and verified before it became part of the complete system.

## Milestone 0: MuJoCo model

Created a two-link planar arm with shoulder and elbow hinge joints, torque
motors, a red target, an end-effector site, and a fixed overhead camera. Learned
the distinction between model configuration, simulation data, joint state,
and actuator torque.

## Milestone 1: Forward kinematics

Implemented the two-link forward-kinematics equations manually. Verified known
joint configurations and compared calculated end-effector positions with the
MuJoCo site position.

## Milestone 2: Inverse kinematics

Added annular-workspace reachability checks and both elbow configurations for
planar geometric IK. Verified every solution through forward kinematics and
MuJoCo.

## Milestone 3: Feedback control

Implemented P and PD torque control with actuator saturation. Observed dynamic
coupling between joints and used IK outputs as desired joint angles for the PD
controller.

## Milestone 4: Camera rendering

Rendered RGB arrays from the fixed overhead MuJoCo camera and saved frames for
visual inspection.

## Milestone 5: Target detection

Detected the red target using both HSV red hue ranges, selected the largest
contour, and calculated its pixel centroid. Verified detection on synthetic
images and rendered MuJoCo frames.

## Milestone 6: Pixel-to-world mapping

Mapped image pixels onto the known horizontal target plane using the camera's
field of view and position. Verified several target placements against their
known world coordinates.

## Milestone 7: Closed vision-guided loop

Connected camera rendering, detection, coordinate mapping, inverse kinematics,
PD control, and MuJoCo physics. Moved the target during the simulation and
verified that the robot responded using new camera observations rather than
the target's simulator coordinates. Observed self-occlusion when the arm hid
part of the target from the overhead camera.

## Milestone 8: Experiments

Compared P and PD control under identical initial conditions. PD substantially
reduced overshoot and oscillation and settled within the observation window.

Added bounded artificial pixel noise to repeated target observations. Larger
noise increased trial-to-trial variability, although it sometimes reduced
final error by counteracting the existing occlusion bias.

## Milestone 9: Finalization

Collected setup and run instructions, architecture, experiment reports, plots,
limitations, future directions, and a recording of the moving-target
demonstration.
