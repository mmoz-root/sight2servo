import pytest
from math import pi
from sight2servo.kinematics import forward_kinematics, target_is_reachable, inverse_kinematics
from pathlib import Path
import mujoco

MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "arm.xml"
)

LINK_1_M = 0.25
LINK_2_M = 0.18


def test_forward_kinematics_with_both_angles_zero():
    position = forward_kinematics(
        0.0,
        0.0,
        LINK_1_M,
        LINK_2_M,
    )

    assert position == pytest.approx((0.43, 0.0))


def test_forward_kinematics_with_shoulder_at_ninety_degrees():
    position = forward_kinematics(
        pi / 2,
        0.0,
        LINK_1_M,
        LINK_2_M,
    )

    assert position == pytest.approx((0.0, 0.43))

def test_forward_kinematics_with_elbow_at_ninety_degrees():
    position = forward_kinematics(
        0.0,
        pi / 2,
        LINK_1_M,
        LINK_2_M,
    )

    assert position == pytest.approx((0.25, 0.18))

def test_nonzero_config():
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )
    data = mujoco.MjData(model)

    data.qpos[:] = [pi/4, -pi/6]

    mujoco.mj_forward(model, data)

    manual_xy = forward_kinematics(
        pi/4,
        -pi/6,
        LINK_1_M,
        LINK_2_M
    )

    mujoco_xy = data.site(
        "end_effector"
    ).xpos[:2]

    assert manual_xy == pytest.approx(mujoco_xy)

@pytest.mark.parametrize(
    ("target_x_m", "target_y_m", "expected"),
    [
        (0.25, 0.18, True),
        (0.50, 0.00, False),
        (0.03, 0.00, False),
    ],
)
def test_target_reachability(
    target_x_m,
    target_y_m,
    expected,
):
    result = target_is_reachable(
        target_x_m,
        target_y_m,
        LINK_1_M,
        LINK_2_M,
    )

    assert result is expected


TARGET_X_M = 0.25
TARGET_Y_M = 0.18

def test_inverse_kinematics():
    solutions = inverse_kinematics(
        TARGET_X_M, TARGET_Y_M,
        LINK_1_M, LINK_2_M
    )

    assert len(solutions) == 2

    for theta_1_rad, theta_2_rad in solutions:
        reconstructed_position = forward_kinematics(
            theta_1_rad,
            theta_2_rad,
            LINK_1_M,
            LINK_2_M,
        )

        assert reconstructed_position == pytest.approx((TARGET_X_M, TARGET_Y_M))

def test_boundarycase_ik():
    solutions = inverse_kinematics(
        0.43, 0.0,
        LINK_1_M, LINK_2_M
    )
    assert len(solutions) == 2

    for theta_1_rad, theta_2_rad in solutions:
        reconstructed_position = forward_kinematics(
            theta_1_rad,
            theta_2_rad,
            LINK_1_M,
            LINK_2_M,
        )

    assert reconstructed_position == pytest.approx((0.43, 0.0))

@pytest.mark.parametrize(
    ("target_x_m", "target_y_m"),
    [
        (0.50, 0.00),  # Too far
        (0.03, 0.00),  # Too close
    ],
)
def test_inverse_kinematics_returns_no_solution_for_unreachable_target(
    target_x_m,
    target_y_m,
):
    solutions = inverse_kinematics(
        target_x_m,
        target_y_m,
        LINK_1_M,
        LINK_2_M,
    )

    assert solutions == []

def test_ik_manual():
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )
    data = mujoco.MjData(model)

    solutions = inverse_kinematics(
        0.25, 0.18,
        LINK_1_M, LINK_2_M
    )

    assert len(solutions) == 2

    for theta_1_rad, theta_2_rad in solutions:
        data.qpos[:] = (theta_1_rad, theta_2_rad)

        mujoco.mj_forward(model, data)

        mujoco_xy = data.site(
            "end_effector"
        ).xpos[:2]

        assert mujoco_xy == pytest.approx((0.25, 0.18))