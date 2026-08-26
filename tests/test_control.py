import pytest

from sight2servo.control import proportional_torque, proportional_derivative_torque, limit_torque


@pytest.mark.parametrize(
    ("desired_angle_rad", "current_angle_rad", "kp_nm_per_rad", "expected"),
    [
        (1.0, 0.25, 0.4, 0.3),   # Below target: positive torque
        (0.5, 0.5, 0.4, 0.0),    # At target: zero torque
        (0.5, 0.75, 0.4, -0.1),  # Past target: negative torque
    ]
)

def test_simple_p_torque(
    desired_angle_rad,
    current_angle_rad,
    kp_nm_per_rad,
    expected
):
    torque = proportional_torque(desired_angle_rad, current_angle_rad, kp_nm_per_rad)

    assert torque == pytest.approx(expected)

def test_pd_torque():
    torque = proportional_derivative_torque(
        desired_angle_rad=1.0,
        current_angle_rad=0.25,
        current_velocity_rad_s=0.5,
        kp_nm_per_rad=0.4,
        kd_nm_s_per_rad=0.1,
    )

    assert torque == pytest.approx(0.25)



@pytest.mark.parametrize(
    ("torque_nm", "max_torque_nm", "expected"),
        [
        (0.30, 0.20, 0.20),    # Positive saturation
        (-0.30, 0.20, -0.20),  # Negative saturation
        (0.05, 0.20, 0.05),    # Already within limits
    ]
)
def test_limit_torque(
    torque_nm,
    max_torque_nm,
    expected
):
    limited_torque_nm = limit_torque(torque_nm, max_torque_nm)

    assert limited_torque_nm == pytest.approx(expected)
