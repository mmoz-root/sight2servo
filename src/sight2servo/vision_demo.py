from pathlib import Path

import mujoco

from sight2servo.camera import render_rgb_frame
from sight2servo.vision import detect_red_target_centroid
from sight2servo.transforms import overhead_pixel_to_world_xy

import cv2

MODEL_PATH = (
    Path(__file__).resolve().parents[2]
    / "models"
    / "arm.xml"
)

model = mujoco.MjModel.from_xml_path(
    str(MODEL_PATH)
)

# model.body("target").pos[:2] = (-0.20, 0.15)
# model.body("target").pos[:2] = (0.15, -0.18)


data = mujoco.MjData(model)

mujoco.mj_forward(model, data)

image_width_px = 640
image_height_px = 480

try:
    renderer = mujoco.Renderer(
        model,
        height=image_height_px,
        width=image_width_px,
    )
    frame_rgb = render_rgb_frame(renderer, data, "overhead")
finally:
    renderer.close()



centroid_px = detect_red_target_centroid(frame_rgb)

if centroid_px is None:
    raise RuntimeError("Red target was not detected")

centroid_u_px, centroid_v_px = centroid_px

print("Detected centroid (u, v):", centroid_px)
print("Horizontal pixel u:", centroid_u_px)
print("Vertical pixel v:", centroid_v_px)

estimated_target_xy_m = overhead_pixel_to_world_xy(
    pixel_u_px=centroid_u_px,
    pixel_v_px=centroid_v_px,
    image_width_px=image_width_px,
    image_height_px=image_height_px,
    camera_x_m=0.0,
    camera_y_m=0.0,
    camera_z_m=1.20,
    plane_z_m=0.06,
    vertical_fov_deg=50.0,
)

print(
    "Estimated target world (x, y):",
    estimated_target_xy_m,
)

frame_bgr = cv2.cvtColor(
    frame_rgb,
    cv2.COLOR_RGB2BGR,
)

cv2.drawMarker(
    frame_bgr,
    centroid_px,
    color=(0, 255, 0),
    markerType=cv2.MARKER_CROSS,
    markerSize=20,
    thickness=2,
)

cv2.circle(
    frame_bgr,
    centroid_px,
    radius=12,
    color=(0, 255, 0),
    thickness=2,
)

output_path = (
    Path(__file__).resolve().parents[2]
    / "assets"
    / "images"
    / "target_detection.png"
)

output_path.parent.mkdir(parents=True, exist_ok=True)

image_was_saved = cv2.imwrite(
    str(output_path),
    frame_bgr
)

if not image_was_saved:
    raise RuntimeError(
        f"Failed to save detection image: {output_path}"
    )