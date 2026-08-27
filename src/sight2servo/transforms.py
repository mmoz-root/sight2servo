from math import radians, tan

def overhead_pixel_to_world_xy(
    pixel_u_px: float,
    pixel_v_px: float,
    image_width_px: int,
    image_height_px: int,
    camera_x_m: float,
    camera_y_m: float,
    camera_z_m: float,
    plane_z_m: float,
    vertical_fov_deg: float,
) -> tuple[float, float]:

    camera_to_plane_distance_m = camera_z_m - plane_z_m

    visible_world_height_m = (
        2.0
        * camera_to_plane_distance_m
        * tan(radians(vertical_fov_deg) / 2.0)
    )

    meters_per_pixel = (
        visible_world_height_m / image_height_px
    )

    center_u_px = (image_width_px - 1) / 2.0
    center_v_px = (image_height_px - 1) / 2.0

    offset_u_px = pixel_u_px - center_u_px
    offset_v_px = pixel_v_px - center_v_px

    world_x_m = camera_x_m + offset_u_px * meters_per_pixel
    world_y_m = camera_y_m - offset_v_px * meters_per_pixel


    return world_x_m, world_y_m