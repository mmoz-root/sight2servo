from pathlib import Path

import mujoco

from sight2servo.control import (
    limit_torque,
    proportional_derivative_torque,
)

from math import hypot, sqrt

import csv

import matplotlib.pyplot as plt




MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "arm.xml"
)

CONTROL_FREQUENCIES_HZ = (
    10, 25, 50, 100, 250, 500,
)

TARGET_CONFIGURATIONS_RAD = {
    "baseline": (0.5, -0.4),
    "smaller_motion": (0.25, -0.2),
    "positive_elbow": (0.5, 0.4),
}

KP_NM_PER_RAD = 0.2
KD_NM_S_PER_RAD = 0.05
MAX_TORQUE_NM = 0.2

SIMULATION_DURATION_S = 8.0
SETTLING_TOLERANCE_RAD = 0.02

def joint_overshoot_rad(
    positions_rad: list[tuple[float, float]],
    joint_index: int,
    desired_angles_rad: tuple[float, float],
) -> float:
    initial_rad = positions_rad[0][joint_index]
    desired_rad = desired_angles_rad[joint_index]

    direction = (
        1.0 if desired_rad >= initial_rad else -1.0
    )

    return max(
        0.0,
        max(
            direction * (position[joint_index] - desired_rad)
            for position in positions_rad
        ),
    )


def run_trial(
    control_frequency_hz: float,
    desired_angles_rad: tuple[float, float],
) -> tuple[list[float], list[tuple[float, float]]]:
    
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH),
    )

    data = mujoco.MjData(model)

    control_interval_steps = round(
        1.0
        / (
            control_frequency_hz
            * model.opt.timestep
        )
    )
    simulation_steps = round(
        SIMULATION_DURATION_S / model.opt.timestep
    )

    times_s = [float(data.time)]
    positions_rad = [
        (float(data.qpos[0]), float(data.qpos[1]))
    ]

    for step in range(simulation_steps):
        if step % control_interval_steps == 0:
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

        mujoco.mj_step(model, data)

        times_s.append(float(data.time))
        positions_rad.append(
            (
                float(data.qpos[0]),
                float(data.qpos[1])
            )
        )
    
    return times_s, positions_rad

def joint_rmse_rad(
    positions_rad: list[tuple[float, float]],
    desired_angles_rad: tuple[float, float],
) -> float:
    squared_error_sum = 0.0

    for position_rad in positions_rad:
        for joint_index, desired_angle_rad in enumerate(
            desired_angles_rad
        ):
            error_rad = (
                desired_angle_rad
                - position_rad[joint_index]
            )
            squared_error_sum += error_rad**2

    number_of_values = len(positions_rad) * 2

    return sqrt(
        squared_error_sum / number_of_values
    )

def settling_time_s(
    times_s: list[float],
    positions_rad: list[tuple[float, float]],
    desired_angles_rad: tuple[float, float],
) -> float | None:
    last_outside_index = None

    for sample_index, position_rad in enumerate(
        positions_rad
    ):
        if any(
            abs(position_rad[j] - desired_angles_rad[j])
            > SETTLING_TOLERANCE_RAD
            for j in range(2)
        ):
            last_outside_index = sample_index

    if last_outside_index is None:
        return times_s[0]

    if last_outside_index == len(positions_rad) - 1:
        return None

    return times_s[last_outside_index + 1]

def final_joint_error_rad(
    positions_rad: list[tuple[float, float]],
    desired_angles_rad: tuple[float, float],
) -> float:
    final_shoulder_rad, final_elbow_rad = positions_rad[-1]

    return hypot(
        desired_angles_rad[0] - final_shoulder_rad,
        desired_angles_rad[1] - final_elbow_rad,
    )


def main() -> None:
    histories_by_configuration = {}
    metric_rows = []

    print(
        "configuration | control_hz | joint_rmse_rad | final_error_rad | "
        "shoulder_overshoot_rad | elbow_overshoot_rad | settling_s"
    )

    for configuration, desired_angles_rad in TARGET_CONFIGURATIONS_RAD.items():
        histories_by_frequency = {}

        for frequency_hz in CONTROL_FREQUENCIES_HZ:
            times_s, positions_rad = run_trial(
                frequency_hz, desired_angles_rad
            )
            histories_by_frequency[frequency_hz] = (times_s, positions_rad)

            rmse_rad = joint_rmse_rad(positions_rad, desired_angles_rad)
            final_error_rad = final_joint_error_rad(
                positions_rad, desired_angles_rad
            )
            shoulder_overshoot_rad = joint_overshoot_rad(
                positions_rad, 0, desired_angles_rad
            )
            elbow_overshoot_rad = joint_overshoot_rad(
                positions_rad, 1, desired_angles_rad
            )
            settling_s = settling_time_s(
                times_s, positions_rad, desired_angles_rad
            )

            metric_rows.append([
                configuration,
                desired_angles_rad[0],
                desired_angles_rad[1],
                frequency_hz,
                times_s[-1],
                rmse_rad,
                final_error_rad,
                shoulder_overshoot_rad,
                elbow_overshoot_rad,
                settling_s if settling_s is not None else "",
            ])

            settling_label = (
                f"{settling_s:.3f}"
                if settling_s is not None
                else f"Not settled within {SIMULATION_DURATION_S:g} s"
            )
            print(
                f"{configuration:14s} | {frequency_hz:10d} | "
                f"{rmse_rad:14.6f} | {final_error_rad:15.6f} | "
                f"{shoulder_overshoot_rad:22.6f} | "
                f"{elbow_overshoot_rad:19.6f} | {settling_label}"
            )

        histories_by_configuration[configuration] = histories_by_frequency

    # Keep the original single-target CSV and plot as the baseline record.
    results_path = (
        MODEL_PATH.parent.parent
        / "assets" / "results" / "control_frequency_configurations.csv"
    )
    results_path.parent.mkdir(parents=True, exist_ok=True)

    with results_path.open("w", newline="") as results_file:
        writer = csv.writer(results_file)
        writer.writerow([
            "configuration",
            "desired_shoulder_rad",
            "desired_elbow_rad",
            "control_hz",
            "duration_s",
            "joint_rmse_rad",
            "final_joint_error_rad",
            "shoulder_overshoot_rad",
            "elbow_overshoot_rad",
            "settling_time_s",
        ])
        writer.writerows(metric_rows)

    print("Saved metrics:", results_path)

    for configuration, histories_by_frequency in (
        histories_by_configuration.items()
    ):
        desired_angles_rad = TARGET_CONFIGURATIONS_RAD[configuration]
        figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

        for joint_index, joint_name in enumerate(("Shoulder", "Elbow")):
            axis = axes[joint_index]

            for frequency_hz, (times_s, positions_rad) in (
                histories_by_frequency.items()
            ):
                axis.plot(
                    times_s,
                    [position[joint_index] for position in positions_rad],
                    label=f"{frequency_hz} Hz",
                    linewidth=1.2,
                )

            desired_rad = desired_angles_rad[joint_index]
            axis.axhspan(
                desired_rad - SETTLING_TOLERANCE_RAD,
                desired_rad + SETTLING_TOLERANCE_RAD,
                color="gray",
                alpha=0.15,
                label="Settling tolerance",
            )
            axis.axhline(
                desired_rad, color="black", linestyle="--", label="Desired"
            )
            axis.set_title(joint_name)
            axis.set_ylabel("Angle (rad)")
            axis.grid(alpha=0.3)

        axes[0].legend(ncol=4, fontsize=8)
        axes[-1].set_xlabel("Time (s)")
        figure.suptitle(
            f"Controller Update Frequency: {configuration}\n"
            f"Desired joints: {desired_angles_rad} rad"
        )
        figure.tight_layout()

        plot_path = (
            MODEL_PATH.parent.parent
            / "assets" / "plots" / f"control_frequency_{configuration}.png"
        )
        plot_path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(plot_path, dpi=150)
        plt.close(figure)
        print("Saved plot:", plot_path)


if __name__ == "__main__":
    main()
