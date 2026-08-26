def proportional_torque(
    desired_angle_rad: float,
    current_angle_rad: float,
    kp_nm_per_rad: float,
) -> float:
    
    error = desired_angle_rad - current_angle_rad

    p_torque = kp_nm_per_rad * error

    return p_torque


def proportional_derivative_torque(
    desired_angle_rad: float,
    current_angle_rad: float,
    current_velocity_rad_s: float,
    kp_nm_per_rad: float,
    kd_nm_s_per_rad: float,
) -> float:
    error = desired_angle_rad - current_angle_rad 

    p_torque = kp_nm_per_rad * error

    d_torque = - kd_nm_s_per_rad * current_velocity_rad_s

    torque = p_torque + d_torque

    return torque

def limit_torque(
    torque_nm: float,
    max_torque_nm: float
) -> float:

    if torque_nm > max_torque_nm:
        return max_torque_nm
        
    if torque_nm < -max_torque_nm:
        return -max_torque_nm

    return torque_nm