
"""ArUco detector for the Lab 7 camera."""

from __future__ import annotations

from typing import TYPE_CHECKING

import cv2

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray


def detect_markers(
    image: NDArray[np.uint8],
    dictionary_name: str,
) -> tuple[list[int], list[NDArray[np.float32]]]:

    # 1. Load the configured dictionary
    aruco = cv2.aruco
    dictionary_id = getattr(aruco, dictionary_name)
    dictionary = aruco.getPredefinedDictionary(dictionary_id)

    # 2. Convert BGR image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 3. Detect markers with compatible OpenCV API
    if hasattr(aruco, "ArucoDetector"):
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

    else:
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(
            gray,
            dictionary,
            parameters=parameters,
        )

    # 4. Handle empty detection
    if ids is None or len(ids) == 0:
        return [], []

    # 5. Keep IDs and corners paired
    pairs = [
        (int(marker_id), corner)
        for marker_id, corner in zip(ids.flatten(), corners)
    ]

    # 6. Sort by marker ID
    pairs.sort(key=lambda pair: pair[0])

    return (
        [marker_id for marker_id, _ in pairs],
        [corner for _, corner in pairs],
    )
