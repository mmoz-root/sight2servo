from math import hypot
from pathlib import Path
from random import Random
from statistics import mean, stdev
import matplotlib.pyplot as plt
import mujoco

from sight2servo.camera import render_rgb_frame
from sight2servo.control import (
    limit_torque,
    proportional_derivative_torque,
)
from sight2servo.kinematics import inverse_kinematics
from sight2servo.transforms import (
    overhead_pixel_to_world_xy,
)
from sight2servo.vision import (
    detect_red_target_centroid,
)


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "arm.xml"
)

IMAGE_WIDTH_PX = 320
IMAGE_HEIGHT_PX = 240

CAMERA_X_M = 0.0
CAMERA_Y_M = 0.0
CAMERA_Z_M = 1.20
TARGET_PLANE_Z_M = 0.06
CAMERA_VERTICAL_FOV_DEG = 50.0

LINK_1_M = 0.25
LINK_2_M = 0.18

KP_NM_PER_RAD = 0.2
KD_NM_S_PER_RAD = 0.05
MAX_TORQUE_NM = 0.2

PERCEPTION_HZ = 25.0
SIMULATION_STEPS = 2000

NOISE_LEVELS_PX = (0, 1, 5, 10)
TRIALS_PER_LEVEL = 20
BASE_RANDOM_SEED = 42


def add_pixel_noise(
    centroid_px: tuple[int, int],
    noise_bound_px: int,
    random_generator: Random,
) -> tuple[int, int]:
    noise_u_px = random_generator.randint(
        -noise_bound_px,
        noise_bound_px,
    )
    noise_v_px = random_generator.randint(
        -noise_bound_px,
        noise_bound_px,
    )

    noisy_u_px = centroid_px[0] + noise_u_px
    noisy_v_px = centroid_px[1] + noise_v_px

    return noisy_u_px, noisy_v_px

def perceive_target_xy_m(
    renderer: mujoco.Renderer,
    data: mujoco.MjData,
    noise_bound_px: int,
    random_generator: Random,
) -> tuple[float, float] | None:
    frame_rgb = render_rgb_frame(
        renderer,
        data,
        "overhead",
    )

    centroid_px = detect_red_target_centroid(
        frame_rgb
    )

    if centroid_px is None:
        return None

    noisy_centroid_px = add_pixel_noise(
        centroid_px,
        noise_bound_px,
        random_generator,
    )

    return overhead_pixel_to_world_xy(
        pixel_u_px=noisy_centroid_px[0],
        pixel_v_px=noisy_centroid_px[1],
        image_width_px=IMAGE_WIDTH_PX,
        image_height_px=IMAGE_HEIGHT_PX,
        camera_x_m=CAMERA_X_M,
        camera_y_m=CAMERA_Y_M,
        camera_z_m=CAMERA_Z_M,
        plane_z_m=TARGET_PLANE_Z_M,
        vertical_fov_deg=CAMERA_VERTICAL_FOV_DEG,
    )

def apply_pd_control(
    data: mujoco.MjData,
    desired_angles_rad: tuple[float, float] | None,
) -> None:
    if desired_angles_rad is None:
        data.ctrl[:] = 0.0
        return

    for joint_index, desired_angle_rad in enumerate(
        desired_angles_rad
    ):
        requested_torque_nm = (
            proportional_derivative_torque(
                desired_angle_rad,
                data.qpos[joint_index],
                data.qvel[joint_index],
                KP_NM_PER_RAD,
                KD_NM_S_PER_RAD,
            )
        )

        data.ctrl[joint_index] = limit_torque(
            requested_torque_nm,
            MAX_TORQUE_NM,
        )

def run_trial(
    noise_bound_px: int,
    random_seed: int,
) -> float:
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(
        model,
        height=IMAGE_HEIGHT_PX,
        width=IMAGE_WIDTH_PX,
    )

    random_generator = Random(random_seed)

    perception_interval_steps = round(
        1.0
        / (
            PERCEPTION_HZ
            * model.opt.timestep
        )
    )

    desired_angles_rad: (
        tuple[float, float] | None
    ) = None

    try:
        for step in range(SIMULATION_STEPS):
            if step % perception_interval_steps == 0:
                estimated_target_xy_m = (
                    perceive_target_xy_m(
                        renderer,
                        data,
                        noise_bound_px,
                        random_generator,
                    )
                )

                if estimated_target_xy_m is not None:
                    ik_solutions = inverse_kinematics(
                        estimated_target_xy_m[0],
                        estimated_target_xy_m[1],
                        LINK_1_M,
                        LINK_2_M,
                    )

                    if ik_solutions:
                        desired_angles_rad = ik_solutions[0]

            apply_pd_control(
                data,
                desired_angles_rad,
            )

            mujoco.mj_step(model, data)
    finally:
        renderer.close()

    if desired_angles_rad is None:
        raise RuntimeError(
            "No valid target observation was obtained"
        )

    ground_truth_target_xy_m = data.body(
        "target"
    ).xpos[:2]

    final_end_effector_xy_m = data.site(
        "end_effector"
    ).xpos[:2]

    return hypot(
        ground_truth_target_xy_m[0]
        - final_end_effector_xy_m[0],
        ground_truth_target_xy_m[1]
        - final_end_effector_xy_m[1],
    )

errors_by_noise_m: dict[int, list[float]] = {}

for noise_bound_px in NOISE_LEVELS_PX:
    trial_errors_m = []

    for trial_index in range(TRIALS_PER_LEVEL):
        random_seed = (
            BASE_RANDOM_SEED
            + trial_index
        )

        error_m = run_trial(
            noise_bound_px,
            random_seed,
        )

        trial_errors_m.append(error_m)

        print(
            f"noise=±{noise_bound_px:2d} px | "
            f"trial={trial_index + 1} | "
            f"error={error_m * 1000:.3f} mm"
        )

    errors_by_noise_m[noise_bound_px] = (
        trial_errors_m
    )

    print(
        f"noise=±{noise_bound_px:2d} px | "
        f"mean={mean(trial_errors_m) * 1000:.3f} mm | "
        f"std={stdev(trial_errors_m) * 1000:.3f} mm"
    )

noise_labels = [
    f"±{noise_bound_px} px"
    for noise_bound_px in NOISE_LEVELS_PX
]

error_groups_mm = [
    [
        error_m * 1000
        for error_m in errors_by_noise_m[
            noise_bound_px
        ]
    ]
    for noise_bound_px in NOISE_LEVELS_PX
]

figure, axis = plt.subplots(
    figsize=(8, 5)
)

axis.boxplot(
    error_groups_mm,
    tick_labels=noise_labels,
    showmeans=True,
)

axis.set_title(
    "Effect of Pixel Noise on Final Position Error"
)
axis.set_xlabel("Artificial centroid noise")
axis.set_ylabel("Final Cartesian error (mm)")
axis.grid(
    axis="y",
    alpha=0.3,
)

figure.tight_layout()

plot_path = (
    Path(__file__).resolve().parents[1]
    / "assets"
    / "plots"
    / "perception_noise.png"
)

plot_path.parent.mkdir(
    parents=True,
    exist_ok=True,
)

figure.savefig(
    plot_path,
    dpi=150,
)

plt.close(figure)

print("Saved plot:", plot_path)