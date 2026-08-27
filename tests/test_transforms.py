import pytest

from sight2servo.transforms import overhead_pixel_to_world_xy
from sight2servo.vision import detect_red_target_centroid
from sight2servo.camera import render_rgb_frame

import mujoco

from pathlib import Path


def test_image_center_maps_directly_below_camera():
    world_xy_m = overhead_pixel_to_world_xy(
        pixel_u_px=319.5,
        pixel_v_px=239.5,
        image_width_px=640,
        image_height_px=480,
        camera_x_m=0.10,
        camera_y_m=-0.20,
        camera_z_m=1.20,
        plane_z_m=0.06,
        vertical_fov_deg=50.0,
    )

    assert world_xy_m == pytest.approx(
        (0.10, -0.20)
    )


def test_detected_target_pixel_maps_to_known_world_position():
    world_xy_m = overhead_pixel_to_world_xy(
        pixel_u_px=419,
        pixel_v_px=149,
        image_width_px=640,
        image_height_px=480,
        camera_x_m=0.0,
        camera_y_m=0.0,
        camera_z_m=1.20,
        plane_z_m=0.06,
        vertical_fov_deg=50.0,
    )

    assert world_xy_m == pytest.approx(
        (0.22, 0.20),
        abs=0.003,
    )


@pytest.mark.parametrize(
    ("target_x_m", "target_y_m"),
    [
        (0.22, 0.20),
        (-0.20, 0.15),
        (0.15, -0.18),
    ],
)
def test_camera_detection_maps_known_target_positions(
    target_x_m,
    target_y_m
):
    MODEL_PATH = (
        Path(__file__).resolve().parents[1]
        / "models"
        / "arm.xml"
    )
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )
    model.body("target").pos[:2] = (
        target_x_m,
        target_y_m,
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(
            model,
            height=240,
            width=320,
    )

    try:
        frame_rgb = render_rgb_frame(renderer, data, "overhead")
    finally:
        renderer.close()

    centroid_px = detect_red_target_centroid(frame_rgb)

    assert centroid_px is not None
    centroid_u_px, centroid_v_px = centroid_px

    estimated_xy_m = overhead_pixel_to_world_xy(
        pixel_u_px=centroid_u_px,
        pixel_v_px=centroid_v_px,
        image_width_px=320,
        image_height_px=240,
        camera_x_m=0.0,
        camera_y_m=0.0,
        camera_z_m=1.20,
        plane_z_m=0.06,
        vertical_fov_deg=50.0,
    )

    assert estimated_xy_m == pytest.approx(
        (target_x_m, target_y_m),
        abs=0.006,
    )