from pathlib import Path

import mujoco

import cv2

from sight2servo.camera import render_rgb_frame

MODEL_PATH = (
    Path(__file__).resolve().parents[2]
    / "models"
    / "arm.xml"
)

model = mujoco.MjModel.from_xml_path(
    str(MODEL_PATH)
)

data = mujoco.MjData(model)

mujoco.mj_forward(model, data)


image_width_px = 640
image_height_px = 480

renderer = mujoco.Renderer(
    model,
    height=image_height_px,
    width=image_width_px,
)

frame_rgb = render_rgb_frame(renderer, data, "overhead")


print("Shape:", frame_rgb.shape)
print("Data type:", frame_rgb.dtype)
print("Value range:", frame_rgb.min(), frame_rgb.max())

renderer.close()


output_path = (
    Path(__file__).resolve().parents[2]
    / "assets"
    / "images"
    / "overhead_frame.png"
)

output_path.parent.mkdir(parents=True, exist_ok=True)

frame_bgr = cv2.cvtColor(
    frame_rgb,
    cv2.COLOR_RGB2BGR,
)

image_was_saved = cv2.imwrite(
    str(output_path),
    frame_bgr
)

if not image_was_saved:
    raise RuntimeError(f"Failed to save camera frame from: {output_path}")

print("Saved frame:", output_path)