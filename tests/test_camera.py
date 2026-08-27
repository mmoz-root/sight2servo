from pathlib import Path

import mujoco
import numpy as np

from sight2servo.camera import render_rgb_frame


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "arm.xml"
)

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

renderer = mujoco.Renderer(
    model,
    height=120,
    width=160
)

def test_render_rgb_frame():
    try:
        frame_rgb = render_rgb_frame(
            renderer,
            data,
            "overhead"
        )
    finally:
        renderer.close()

    assert frame_rgb.shape == (120, 160, 3)
    assert frame_rgb.dtype == np.uint8
    assert frame_rgb.min() < frame_rgb.max()