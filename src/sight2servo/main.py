from pathlib import Path

import mujoco
import mujoco.viewer

import time

from math import hypot

from sight2servo.control import (
    limit_torque,
    proportional_derivative_torque,
)
from sight2servo.camera import render_rgb_frame
from sight2servo.kinematics import inverse_kinematics
from sight2servo.transforms import overhead_pixel_to_world_xy
from sight2servo.vision import detect_red_target_centroid


MODEL_PATH = (
    Path(__file__).resolve().parents[2]
    / "models"
    / "arm.xml"
)


def main() -> None:
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    perception_hz = 25.0

    perception_interval_steps = round(
        1.0 / (
            perception_hz
            * model.opt.timestep
        )
    )

    print("Physics timestep:", model.opt.timestep)
    print(
        "Perception interval:",
        perception_interval_steps,
    )
    image_width_px = 320
    image_height_px = 240
    link_1_m = 0.25
    link_2_m = 0.18

    kp_nm_per_rad = 0.2
    kd_nm_s_per_rad = 0.05
    max_torque_nm = 0.2

    desired_angles_rad: tuple[float, float] | None = None
    target_xy_m: tuple[float, float] | None = None
    centroid_px: tuple[int, int] | None = None

    renderer = mujoco.Renderer(
        model,
        height=image_height_px,
        width=image_width_px,
    )

    try:
        with mujoco.viewer.launch_passive(
            model,
            data,
        ) as viewer:
            viewer.cam.type = (
                mujoco.mjtCamera.mjCAMERA_FIXED
            )
            viewer.cam.fixedcamid = model.camera(
                "overhead"
            ).id

            for step in range(2000):
                if step == 1000:
                    model.body("target").pos[:2] = (
                        0.16,
                        -0.18,
                    )
                mujoco.mj_forward(model, data)
                if step % perception_interval_steps == 0:
                    frame_rgb = render_rgb_frame(
                        renderer,
                        data,
                        "overhead",
                    )

                    centroid_px = detect_red_target_centroid(
                        frame_rgb
                    )

                    if centroid_px is not None:
                        detected_target_xy_m = (
                            overhead_pixel_to_world_xy(
                                pixel_u_px=centroid_px[0],
                                pixel_v_px=centroid_px[1],
                                image_width_px=image_width_px,
                                image_height_px=image_height_px,
                                camera_x_m=0.0,
                                camera_y_m=0.0,
                                camera_z_m=1.20,
                                plane_z_m=0.06,
                                vertical_fov_deg=50.0,
                            )
                        )

                        ik_solutions = inverse_kinematics(
                            detected_target_xy_m[0],
                            detected_target_xy_m[1],
                            link_1_m,
                            link_2_m,
                        )

                        if ik_solutions:
                            target_xy_m = detected_target_xy_m
                            desired_angles_rad = ik_solutions[0]

                    if step % 200 == 0:
                        print(
                            f"step={step}",
                            f"centroid={centroid_px}",
                            f"target={target_xy_m}",
                            f"desired={desired_angles_rad}",
                        )

                if desired_angles_rad is None:
                    data.ctrl[:] = 0.0
                else:
                    for joint_index, desired_angle_rad in enumerate(
                        desired_angles_rad
                    ):
                        requested_torque_nm = (
                            proportional_derivative_torque(
                                desired_angle_rad,
                                data.qpos[joint_index],
                                data.qvel[joint_index],
                                kp_nm_per_rad,
                                kd_nm_s_per_rad,
                            )
                        )

                        data.ctrl[joint_index] = limit_torque(
                            requested_torque_nm,
                            max_torque_nm,
                        )

                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(model.opt.timestep)
    finally:
        renderer.close()

    if target_xy_m is None:
        raise RuntimeError(
            "No valid camera target was obtained"
        )

    final_x_m, final_y_m = data.site(
        "end_effector"
    ).xpos[:2]

    position_error_m = hypot(
        target_xy_m[0] - final_x_m,
        target_xy_m[1] - final_y_m,
    )

    print("Final end effector:", (final_x_m, final_y_m))
    print(
        f"Camera-target error: "
        f"{position_error_m * 1000:.3f} mm"
    )
if __name__ == "__main__":
    main()
