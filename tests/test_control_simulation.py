from pathlib import Path
import pytest

import mujoco
from sight2servo.kinematics import inverse_kinematics
from sight2servo.control import (
    limit_torque,
    proportional_derivative_torque,
)


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "arm.xml"
)

model = mujoco.MjModel.from_xml_path(
    str(MODEL_PATH)
)


LINK_1_M = 0.25
LINK_2_M = 0.18
KP_NM_PER_RAD = 0.2
KD_NM_S_PER_RAD = 0.05
MAX_TORQUE_NM = 0.2


def test_control_simulation():
    data = mujoco.MjData(model)
    solutions = inverse_kinematics(
            0.25, 0.18,
            LINK_1_M, LINK_2_M
        )

    desired_angles_rad = solutions[0]

    for _ in range(2000):
        for joint_index, desired_angle_rad in enumerate(desired_angles_rad):
            requested_torque_nm = proportional_derivative_torque(
                desired_angle_rad,
                data.qpos[joint_index],
                data.qvel[joint_index],
                KP_NM_PER_RAD,
                KD_NM_S_PER_RAD,
            )

            data.ctrl[joint_index] = limit_torque(
                requested_torque_nm,
                MAX_TORQUE_NM,
            )

        mujoco.mj_step(model, data)
         
    final_xy = data.site("end_effector").xpos[:2]

    assert final_xy == pytest.approx(
        (0.25, 0.18),
        abs=0.001,
    )

    assert data.qvel == pytest.approx(
        (0.0, 0.0),
        abs=0.05,
    )