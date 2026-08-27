import mujoco
import numpy as np
from numpy.typing import NDArray


def render_rgb_frame(
    renderer: mujoco.Renderer,
    data: mujoco.MjData,
    camera_name: str,
) -> NDArray[np.uint8]:
    renderer.update_scene(data, camera=camera_name)
    return renderer.render().copy()