from pathlib import Path

import mujoco

from sight2servo.control import proportional_derivative_torque, limit_torque
from sight2servo.kinematics import inverse_kinematics

from math import hypot


MODEL_PATH = (
    Path(__file__).resolve().parents[2]
    / "models"
    / "arm.xml"
)

model = mujoco.MjModel.from_xml_path(
    str(MODEL_PATH)
)

data = mujoco.MjData(model)

target_x_m = 0.25
target_y_m = 0.18
link_1_m = 0.25
link_2_m = 0.18

solutions = inverse_kinematics(
    target_x_m,
    target_y_m,
    link_1_m,
    link_2_m,
)

desired_angles_rad = solutions[0]

print("IK solutions:", solutions)
print("Selected solution:", desired_angles_rad)
joint_names = ("shoulder", "elbow")
kp_nm_per_rad = 0.2
kd_nm_s_per_rad = 0.05
max_torque_nm = 0.2

print(
    "step | time_s | joint    | target_rad | position_rad | error_rad | "
    "velocity_rad_s | P_Nm    | D_Nm    | requested_Nm | applied_Nm"
)

for step in range(2000):
    for joint_index, desired_angle_rad in enumerate(desired_angles_rad):
        current_angle_rad = data.qpos[joint_index]
        current_velocity_rad_s = data.qvel[joint_index]
        error_rad = desired_angle_rad - current_angle_rad
        p_torque_nm = kp_nm_per_rad * error_rad
        d_torque_nm = -kd_nm_s_per_rad * current_velocity_rad_s

        requested_torque_nm = proportional_derivative_torque(
            desired_angle_rad,
            current_angle_rad,
            current_velocity_rad_s,
            kp_nm_per_rad,
            kd_nm_s_per_rad,
        )

        applied_torque_nm = limit_torque(
            requested_torque_nm,
            max_torque_nm,
        )
        data.ctrl[joint_index] = applied_torque_nm

        if step % 200 == 0:
            print(
                f"{step:4d} | {data.time:6.3f} | "
                f"{joint_names[joint_index]:8s} | "
                f"{desired_angle_rad:10.4f} | "
                f"{current_angle_rad:12.4f} | "
                f"{error_rad:9.4f} | "
                f"{current_velocity_rad_s:14.4f} | "
                f"{p_torque_nm:7.4f} | "
                f"{d_torque_nm:7.4f} | "
                f"{requested_torque_nm:12.4f} | "
                f"{applied_torque_nm:10.4f}"
            )

    mujoco.mj_step(model, data)

final_x_m, final_y_m = data.site("end_effector").xpos[:2]
print(f"Target (x, y): {target_x_m, target_y_m}")
print(f"Final (x, y): {final_x_m, final_y_m}")

x_error_m = target_x_m - final_x_m
y_error_m = target_y_m - final_y_m

position_error_m = hypot(x_error_m, y_error_m)

print(
    f"Position error: {position_error_m:.6f} m "
    f"({position_error_m * 1000:.3f} mm)"
)
