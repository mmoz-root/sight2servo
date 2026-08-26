from pathlib import Path

import mujoco

from sight2servo.control import proportional_derivative_torque


MODEL_PATH = (
    Path(__file__).resolve().parents[2]
    / "models"
    / "arm.xml"
)

model = mujoco.MjModel.from_xml_path(
    str(MODEL_PATH)
)

data = mujoco.MjData(model)

desired_shoulder_angle_rad = 0.5
kp_nm_per_rad = 0.2
kd_nm_s_per_rad = 0.05

print(
    "step | time_s | target_rad | position_rad | error_rad | "
    "velocity_rad_s | P_Nm    | D_Nm    | total_Nm"
)

for step in range(1000):
    current_shoulder_angle_rad = data.qpos[0]
    current_shoulder_velocity_rad_s = data.qvel[0]
    error_rad = desired_shoulder_angle_rad - current_shoulder_angle_rad
    p_torque_nm = kp_nm_per_rad * error_rad
    d_torque_nm = -kd_nm_s_per_rad * current_shoulder_velocity_rad_s

    shoulder_torque_nm = proportional_derivative_torque(
        desired_shoulder_angle_rad,
        current_shoulder_angle_rad,
        current_shoulder_velocity_rad_s,
        kp_nm_per_rad,
        kd_nm_s_per_rad,
    )

    data.ctrl[0] = shoulder_torque_nm

    if step % 100 == 0:
        print(
            f"{step:4d} | {data.time:6.3f} | "
            f"{desired_shoulder_angle_rad:10.4f} | "
            f"{current_shoulder_angle_rad:12.4f} | "
            f"{error_rad:9.4f} | "
            f"{current_shoulder_velocity_rad_s:14.4f} | "
            f"{p_torque_nm:7.4f} | "
            f"{d_torque_nm:7.4f} | "
            f"{shoulder_torque_nm:8.4f}"
        )

    mujoco.mj_step(model, data)
