from math import cos, sin, pi, hypot, acos, atan2, isclose

def forward_kinematics(
    theta_1_rad: float,
    theta_2_rad: float,
    link_1_m: float,
    link_2_m: float,
):
    elbow_x_m = link_1_m * cos(theta_1_rad)
    elbow_y_m = link_1_m * sin(theta_1_rad)

    link_2_world_angle_rad = theta_1_rad + theta_2_rad

    end_effector_x_m = elbow_x_m + link_2_m*cos(link_2_world_angle_rad)
    end_effector_y_m = elbow_y_m + link_2_m*sin(link_2_world_angle_rad)

    return end_effector_x_m, end_effector_y_m

def target_is_reachable(
    target_x_m: float,
    target_y_m: float,
    link_1_m: float,
    link_2_m: float
) -> bool:
    target_distance_m = hypot(target_x_m, target_y_m)

    minimum_reach_m = abs(link_1_m - link_2_m)
    maximum_reach_m = link_1_m + link_2_m

    return minimum_reach_m <= target_distance_m <= maximum_reach_m

def inverse_kinematics(
    target_x_m: float,
    target_y_m: float,
    link_1_m: float,
    link_2_m: float,
) -> list[tuple[float, float]]:

    if not target_is_reachable(target_x_m, target_y_m, link_1_m, link_2_m):
        return []
    
    cosine_theta_2 = (
        target_x_m**2 + target_y_m**2 
        - link_1_m**2 - link_2_m**2
    ) / (2.0*link_1_m*link_2_m)

    cosine_theta_2 = max(
        -1.0,
        min(1.0, cosine_theta_2)
    )

    theta_2_magnitude_rad = acos(cosine_theta_2)

    if (
        isclose(
            theta_2_magnitude_rad,
            0.0,
            abs_tol=1e-12,
        )
        or isclose(
            theta_2_magnitude_rad,
            pi,
            abs_tol=1e-12,
        )
    ):
        theta_2_candidates_rad = (
            theta_2_magnitude_rad,
        )
    else:
        theta_2_candidates_rad = (
            theta_2_magnitude_rad,
            -theta_2_magnitude_rad,
        )

    solutions = []

    target_direction_rad = atan2(
        target_y_m,
        target_x_m,
    )

    for theta_2_rad in theta_2_candidates_rad:
        shoulder_offset_rad = atan2(
            link_2_m * sin(theta_2_rad),
            link_1_m + link_2_m * cos(theta_2_rad),
        )

        theta_1_rad = (
            target_direction_rad
            - shoulder_offset_rad
        )

        solutions.append(
            (theta_1_rad, theta_2_rad)
        )
    
    return solutions