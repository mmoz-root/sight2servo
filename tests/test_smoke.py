from pathlib import Path

import mujoco


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "arm.xml"
)


def test_model_loads_with_expected_structure():
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )

    assert model.njnt == 2
    assert model.nu == 2
    assert model.ncam == 1


def test_shoulder_motor_changes_joint_state():
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )

    data = mujoco.MjData(model)
    initial_shoulder_position = float(data.qpos[0])
    data.ctrl[0] = 0.01

    for _ in range(100):
        mujoco.mj_step(model, data)

    position_change = abs(
        float(data.qpos[0]) - initial_shoulder_position
    )

    assert data.time > 0.0
    assert position_change > 1e-16

