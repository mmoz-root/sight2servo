import numpy as np

from sight2servo .vision import create_red_mask, detect_red_target_centroid

from pathlib import Path

import mujoco
import pytest

from sight2servo.camera import render_rgb_frame

def test_create_red_mask_selects_red_pixels():
    frame_rgb = np.zeros(
        (100, 120, 3),
        dtype=np.uint8
    )

    frame_rgb[30:51, 70:91] = (255, 0, 0)

    frame_rgb[10:21, 10:21] = (0, 0, 255)

    red_mask = create_red_mask(frame_rgb)

    assert red_mask.shape == (100, 120)
    assert red_mask.dtype == np.uint8
    assert red_mask[40, 80] == 255
    assert red_mask[15, 15] == 0
    assert red_mask[0, 0] == 0


def test_detect_red_target_centroid_uses_largest_red_region():
    frame_rgb = np.zeros(
        (100, 120, 3),
        dtype=np.uint8,
    )

    # Main target centered at (u=80, v=40).
    frame_rgb[30:51, 70:91] = (255, 0, 0)

    # Smaller red distractor.
    frame_rgb[10:15, 10:15] = (255, 0, 0)

    centroid_px = detect_red_target_centroid(frame_rgb)

    assert centroid_px == (80, 40)

def test_detect_red_target_centroid_returns_none_without_red():
    frame_rgb = np.zeros(
        (100, 120, 3),
        dtype=np.uint8,
    )

    centroid_px = detect_red_target_centroid(frame_rgb)

    assert centroid_px is None


def test_one_simulation_camera_integration():
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
            width=160,
    )

    try:
        frame_rgb = render_rgb_frame(renderer, data, "overhead")
    finally:
        renderer.close()

    centroid_px = detect_red_target_centroid(frame_rgb)

    if centroid_px is None:
        raise RuntimeError("Red target was not detected")


    assert centroid_px is not None
    assert centroid_px == pytest.approx(
        (105, 37),
        abs=2
    )
