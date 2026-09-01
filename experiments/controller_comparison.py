from pathlib import Path

import mujoco
from math import hypot


from sight2servo.control import (
    limit_torque,
    proportional_derivative_torque,
)

import matplotlib.pyplot as plt


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "arm.xml"
)

DESIRED_ANGLES_RAD = (0.5, -0.4)

KP_NM_PER_RAD = 0.2
P_KD_NM_S_PER_RAD = 0.0
PD_KD_NM_S_PER_RAD = 0.05

MAX_TORQUE_NM = 0.2
SIMULATION_STEPS = 2000

SETTLING_TOLERANCE_RAD = 0.02



def run_controller(
    kd_nm_s_per_rad: float,
) -> tuple[
    list[float],
    list[tuple[float, float]],
]:
    model = mujoco.MjModel.from_xml_path(
        str(MODEL_PATH)
    )
    data = mujoco.MjData(model)

    time_history_s = [float(data.time)]
    position_history_rad = [
        (
            float(data.qpos[0]),
            float(data.qpos[1]),
        )
    ]

    for _ in range(SIMULATION_STEPS):
        for joint_index, desired_angle_rad in enumerate(
            DESIRED_ANGLES_RAD
        ):
            requested_torque_nm = (
                proportional_derivative_torque(
                    desired_angle_rad,
                    data.qpos[joint_index],
                    data.qvel[joint_index],
                    KP_NM_PER_RAD,
                    kd_nm_s_per_rad,
                )
            )

            data.ctrl[joint_index] = limit_torque(
                requested_torque_nm,
                MAX_TORQUE_NM,
            )

        mujoco.mj_step(model, data)

        time_history_s.append(float(data.time))
        position_history_rad.append(
            (
                float(data.qpos[0]),
                float(data.qpos[1]),
            )
        )

    return time_history_s, position_history_rad

def joint_overshoot_rad(
    positions_rad: list[tuple[float, float]],
    joint_index: int,
) -> float:
    initial_angle_rad = positions_rad[0][joint_index]
    desired_angle_rad = DESIRED_ANGLES_RAD[joint_index]

    direction = (
        1.0
        if desired_angle_rad >= initial_angle_rad
        else -1.0
    )

    overshoots_rad = [
        direction
        * (
            position_rad[joint_index]
            - desired_angle_rad
        )
        for position_rad in positions_rad
    ]

    return max(0.0, max(overshoots_rad))


def final_joint_error_rad(
    positions_rad: list[tuple[float, float]],
) -> float:
    final_shoulder_rad, final_elbow_rad = (
        positions_rad[-1]
    )

    return hypot(
        DESIRED_ANGLES_RAD[0] - final_shoulder_rad,
        DESIRED_ANGLES_RAD[1] - final_elbow_rad,
    )

def settling_time_s(
    times_s: list[float],
    positions_rad: list[tuple[float, float]],
) -> float | None:
    last_outside_index = None

    for sample_index, position_rad in enumerate(
        positions_rad
    ):
        outside_tolerance = any(
            abs(
                DESIRED_ANGLES_RAD[joint_index]
                - position_rad[joint_index]
            )
            > SETTLING_TOLERANCE_RAD
            for joint_index in range(2)
        )

        if outside_tolerance:
            last_outside_index = sample_index

    if last_outside_index is None:
        return 0.0

    if last_outside_index == len(positions_rad) - 1:
        return None

    return times_s[last_outside_index + 1]

def target_crossings(
    positions_rad: list[tuple[float, float]],
    joint_index: int,
) -> int:
    desired_angle_rad = DESIRED_ANGLES_RAD[joint_index]

    previous_side = None
    crossing_count = 0

    for position_rad in positions_rad:
        error_rad = (
            position_rad[joint_index]
            - desired_angle_rad
        )

        if error_rad == 0.0:
            continue

        current_side = 1 if error_rad > 0.0 else -1

        if (
            previous_side is not None
            and current_side != previous_side
        ):
            crossing_count += 1

        previous_side = current_side

    return crossing_count


p_times_s, p_positions_rad = run_controller(
    P_KD_NM_S_PER_RAD
)

pd_times_s, pd_positions_rad = run_controller(
    PD_KD_NM_S_PER_RAD
)

print("Desired joint positions:", DESIRED_ANGLES_RAD)
print("P final joint positions:", p_positions_rad[-1])
print("PD final joint positions:", pd_positions_rad[-1])

print(
    "P final error:",
    final_joint_error_rad(p_positions_rad),
)
print(
    "PD final error:",
    final_joint_error_rad(pd_positions_rad),
)

for joint_index, joint_name in enumerate(
    ("shoulder", "elbow")
):
    print(
        f"P {joint_name} overshoot:",
        joint_overshoot_rad(
            p_positions_rad,
            joint_index,
        ),
    )
    print(
        f"PD {joint_name} overshoot:",
        joint_overshoot_rad(
            pd_positions_rad,
            joint_index,
        ),
    )

print(
    "P settling time:",
    settling_time_s(p_times_s, p_positions_rad),
)
print(
    "PD settling time:",
    settling_time_s(pd_times_s, pd_positions_rad),
)

for joint_index, joint_name in enumerate(
    ("shoulder", "elbow")
):
    print(
        f"P {joint_name} target crossings:",
        target_crossings(
            p_positions_rad,
            joint_index,
        ),
    )
    print(
        f"PD {joint_name} target crossings:",
        target_crossings(
            pd_positions_rad,
            joint_index,
        ),
    )

figure, axes = plt.subplots(
    2,
    1,
    figsize=(10, 7),
    sharex=True,
)

for joint_index, joint_name in enumerate(
    ("Shoulder", "Elbow")
):
    p_joint_positions_rad = [
        position[joint_index]
        for position in p_positions_rad
    ]
    pd_joint_positions_rad = [
        position[joint_index]
        for position in pd_positions_rad
    ]

    axes[joint_index].plot(
        p_times_s,
        p_joint_positions_rad,
        label="P",
    )
    axes[joint_index].plot(
        pd_times_s,
        pd_joint_positions_rad,
        label="PD",
    )
    axes[joint_index].axhline(
        DESIRED_ANGLES_RAD[joint_index],
        color="black",
        linestyle="--",
        label="Desired",
    )

    axes[joint_index].set_title(joint_name)
    axes[joint_index].set_ylabel("Angle (rad)")
    axes[joint_index].grid(alpha=0.3)
    axes[joint_index].legend()

axes[-1].set_xlabel("Time (s)")

figure.suptitle("P vs PD Joint Control")
figure.tight_layout()

plot_path = (
    Path(__file__).resolve().parents[1]
    / "assets"
    / "plots"
    / "controller_comparison.png"
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