from collections import deque
from pathlib import Path

import mujoco

from math import cos, pi, hypot, sqrt

from sight2servo.camera import render_rgb_frame
from sight2servo.transforms import overhead_pixel_to_world_xy
from sight2servo.vision import detect_red_target_centroid
from sight2servo.control import (
    limit_torque,
    proportional_derivative_torque,
)
from sight2servo.kinematics import inverse_kinematics

import csv

import matplotlib.pyplot as plt


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "arm.xml"
)

LATENCIES_MS = (0, 10, 20, 50, 100, 200)

PERCEPTION_HZ = 25.0

KP_NM_PER_RAD = 0.2
KD_NM_S_PER_RAD = 0.05
MAX_TORQUE_NM = 0.2

IMAGE_WIDTH_PX = 320
IMAGE_HEIGHT_PX = 240

INITIAL_TARGET_X_M = 0.22
INITIAL_TARGET_Y_M = 0.20

WARMUP_DURATION_S = 4.0
MOTION_DURATION_S = 8.0
SIMULATION_DURATION_S = (
    WARMUP_DURATION_S + MOTION_DURATION_S
)

MOTION_HALF_RANGE_M = 0.04
MOTION_PERIOD_S = 8.0

LINK_1_M = 0.25
LINK_2_M = 0.18


def target_position_xy_m(
    time_s: float,
) -> tuple[float, float]:
    
    if time_s <= WARMUP_DURATION_S:
        return INITIAL_TARGET_X_M, INITIAL_TARGET_Y_M

    motion_time_s = time_s - WARMUP_DURATION_S

    phase_rad = (
        2.0 * pi * motion_time_s / MOTION_PERIOD_S
    )

    target_x_m = (
        INITIAL_TARGET_X_M
        + MOTION_HALF_RANGE_M * (1.0 - cos(phase_rad))
    )

    return target_x_m, INITIAL_TARGET_Y_M

def receive_ready_target(
    pending_observations: deque[
        tuple[int, tuple[float, float]]
    ],
    current_step: int,
    previous_target_xy_m: tuple[float, float] | None,
) -> tuple[float, float] | None:
    latest_target_xy_m = previous_target_xy_m

    while (
        pending_observations
        and pending_observations[0][0] <= current_step
    ):
        delivery_step, target_xy_m = (
            pending_observations.popleft()
        )
        latest_target_xy_m = target_xy_m

    return latest_target_xy_m


def capture_target_xy_m(
    renderer: mujoco.Renderer,
    data: mujoco.MjData,
) -> tuple[float, float] | None:
    frame_rgb = render_rgb_frame(
        renderer, data, "overhead"
    )

    centroid_px = detect_red_target_centroid(frame_rgb)

    if centroid_px is None:
        return None

    return overhead_pixel_to_world_xy(
        pixel_u_px=centroid_px[0],
        pixel_v_px=centroid_px[1],
        image_width_px=IMAGE_WIDTH_PX,
        image_height_px=IMAGE_HEIGHT_PX,
        camera_x_m=0.0,
        camera_y_m=0.0,
        camera_z_m=1.20,
        plane_z_m=0.06,
        vertical_fov_deg=50.0,
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
        requested_torque_nm = proportional_derivative_torque(
            desired_angle_rad,
            data.qpos[joint_index],
            data.qvel[joint_index],
            KP_NM_PER_RAD,
            KD_NM_S_PER_RAD,
        )

        data.ctrl[joint_index] = limit_torque(
            requested_torque_nm,
            MAX_TORQUE_NM,
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
        requested_torque_nm = proportional_derivative_torque(
            desired_angle_rad,
            data.qpos[joint_index],
            data.qvel[joint_index],
            KP_NM_PER_RAD,
            KD_NM_S_PER_RAD,
        )

        data.ctrl[joint_index] = limit_torque(
            requested_torque_nm,
            MAX_TORQUE_NM,
        )

def run_trial(
    latency_ms: int,
) -> tuple[list[float], list[float], int]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    timestep_s = model.opt.timestep

    simulation_steps = round(
        SIMULATION_DURATION_S / timestep_s
    )
    warmup_steps = round(
        WARMUP_DURATION_S / timestep_s
    )
    perception_interval_steps = round(
        1.0 / (PERCEPTION_HZ * timestep_s)
    )
    latency_steps = round(
        (latency_ms / 1000.0) / timestep_s
    )

    pending_observations = deque()
    received_target_xy_m = None
    desired_angles_rad = None

    times_s = []
    errors_m = []
    missed_detections = 0

    renderer = mujoco.Renderer(
        model,
        height=IMAGE_HEIGHT_PX,
        width=IMAGE_WIDTH_PX,
    )

    try:
        for step in range(simulation_steps + 1):
            time_s = step * timestep_s

            # Move the environment's target and refresh world positions.
            model.body("target").pos[:2] = (
                target_position_xy_m(time_s)
            )
            mujoco.mj_forward(model, data)

            # Ground truth is used only to measure tracking error.
            if step >= warmup_steps:
                actual_target_xy_m = data.body("target").xpos[:2]
                end_effector_xy_m = data.site("end_effector").xpos[:2]

                errors_m.append(
                    hypot(
                        actual_target_xy_m[0] - end_effector_xy_m[0],
                        actual_target_xy_m[1] - end_effector_xy_m[1],
                    )
                )
                times_s.append(time_s)

            # Record the final state without taking an extra physics step.
            if step == simulation_steps:
                break

            if step % perception_interval_steps == 0:
                captured_target_xy_m = capture_target_xy_m(
                    renderer, data
                )

                if captured_target_xy_m is None:
                    missed_detections += 1
                else:
                    pending_observations.append(
                        (
                            step + latency_steps,
                            captured_target_xy_m,
                        )
                    )

            received_target_xy_m = receive_ready_target(
                pending_observations,
                step,
                received_target_xy_m,
            )

            if received_target_xy_m is not None:
                solutions = inverse_kinematics(
                    received_target_xy_m[0],
                    received_target_xy_m[1],
                    LINK_1_M,
                    LINK_2_M,
                )

                if solutions:
                    desired_angles_rad = solutions[0]

            apply_pd_control(data, desired_angles_rad)
            mujoco.mj_step(model, data)

    finally:
        renderer.close()

    if desired_angles_rad is None:
        raise RuntimeError(
            "No delivered observation produced a valid IK solution"
        )

    return times_s, errors_m, missed_detections

def tracking_rmse_m(
    errors_m: list[float],
) -> float:
    return sqrt(
        sum(error_m**2 for error_m in errors_m)
        / len(errors_m)
    )

times_s, errors_m, missed_detections = run_trial(
    latency_ms=0
)

print("Measured interval (s):", times_s[0], "to", times_s[-1])
print("Samples:", len(times_s))
print("Missed detections:", missed_detections)
print(f"Final tracking error: {errors_m[-1] * 1000:.3f} mm")

print(
    f"Tracking RMSE: "
    f"{tracking_rmse_m(errors_m) * 1000:.3f} mm"
)
print(
    f"Peak tracking error: "
    f"{max(errors_m) * 1000:.3f} mm"
)

histories_by_latency = {}

print(
    "latency_ms | rmse_mm | peak_mm | final_mm | missed_detections"
)

for latency_ms in LATENCIES_MS:
    times_s, errors_m, missed_detections = run_trial(
        latency_ms
    )

    histories_by_latency[latency_ms] = (
        times_s,
        errors_m,
        missed_detections,
    )

    print(
        f"{latency_ms:10d} | "
        f"{tracking_rmse_m(errors_m) * 1000:7.3f} | "
        f"{max(errors_m) * 1000:7.3f} | "
        f"{errors_m[-1] * 1000:8.3f} | "
        f"{missed_detections:17d}"
    )

results_dir = (
    Path(__file__).resolve().parents[1]
    / "assets"
    / "results"
)
results_dir.mkdir(parents=True, exist_ok=True)

summary_path = results_dir / "perception_latency.csv"
history_path = results_dir / "perception_latency_history.csv"

with (
    summary_path.open("w", newline="") as summary_file,
    history_path.open("w", newline="") as history_file,
):
    summary_writer = csv.writer(summary_file)
    history_writer = csv.writer(history_file)

    summary_writer.writerow([
        "latency_ms",
        "measurement_start_s",
        "measurement_end_s",
        "samples",
        "tracking_rmse_m",
        "peak_error_m",
        "final_error_m",
        "missed_detections_full_run",
    ])

    history_writer.writerow([
        "latency_ms",
        "time_s",
        "tracking_error_m",
    ])

    for latency_ms, (
        times_s,
        errors_m,
        missed_detections,
    ) in histories_by_latency.items():
        summary_writer.writerow([
            latency_ms,
            times_s[0],
            times_s[-1],
            len(times_s),
            tracking_rmse_m(errors_m),
            max(errors_m),
            errors_m[-1],
            missed_detections,
        ])

        for time_s, error_m in zip(times_s, errors_m):
            history_writer.writerow([
                latency_ms,
                time_s,
                error_m,
            ])

print("Saved summary:", summary_path)
print("Saved histories:", history_path)

figure, axis = plt.subplots(figsize=(10, 5))

for latency_ms, (times_s, errors_m, _) in (
    histories_by_latency.items()
):
    motion_times_s = [
        time_s - WARMUP_DURATION_S
        for time_s in times_s
    ]
    errors_mm = [
        error_m * 1000
        for error_m in errors_m
    ]

    axis.plot(
        motion_times_s,
        errors_mm,
        label=f"{latency_ms} ms",
        linewidth=1.2,
    )

axis.axvline(
    MOTION_PERIOD_S / 2.0,
    color="gray",
    linestyle="--",
    label="Target reverses direction",
)

axis.set_title("Effect of Perception Latency on Tracking Error")
axis.set_xlabel("Time since target motion began (s)")
axis.set_ylabel("Cartesian tracking error (mm)")
axis.grid(alpha=0.3)
axis.legend(ncol=3, fontsize=8)

figure.tight_layout()

plot_path = (
    Path(__file__).resolve().parents[1]
    / "assets"
    / "plots"
    / "perception_latency.png"
)
plot_path.parent.mkdir(parents=True, exist_ok=True)

figure.savefig(plot_path, dpi=150)
plt.close(figure)

print("Saved plot:", plot_path)