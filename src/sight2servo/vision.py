
import numpy as np
from numpy.typing import NDArray

import cv2



def create_red_mask(
    frame_rgb: NDArray[np.uint8]
) -> NDArray[np.uint8]:
    frame_hsv = cv2.cvtColor(
        frame_rgb,
        cv2.COLOR_RGB2HSV,
    )

    lower_red_1 = np.array([0, 100, 50], dtype=np.uint8)
    upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)

    lower_red_2 = np.array([170, 100, 50], dtype=np.uint8)
    upper_red_2 = np.array([179, 255, 255], dtype=np.uint8)

    mask_1 = cv2.inRange(
        frame_hsv,
        lower_red_1,
        upper_red_1
    )

    mask_2 = cv2.inRange(
        frame_hsv,
        lower_red_2,
        upper_red_2
    )

    red_mask = cv2.bitwise_or(mask_1, mask_2)

    return red_mask

def detect_red_target_centroid(
    frame_rgb: NDArray[np.uint8],
) -> tuple[int, int] | None:
    
    red_mask = create_red_mask(frame_rgb)

    contours, _ = cv2.findContours(
        red_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None


    largest_contour = max(
        contours,
        key=cv2.contourArea
    )

    moments = cv2.moments(largest_contour)

    if moments["m00"] == 0:
        return None
    
    centroid_u_px = round(
        moments["m10"] / moments["m00"]
    )
    centroid_v_px = round(
        moments["m01"] / moments["m00"]
    )

    return centroid_u_px, centroid_v_px