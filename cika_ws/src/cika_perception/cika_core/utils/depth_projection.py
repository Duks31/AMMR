"""
depth_projection.py
────────────────────────────────────────────────────────────────────────────
Utility: project a 2D bounding box center through the depth image to get
a 3D point in the camera optical frame, then hand it back for TF transform
to base_link in the calling node.
"""

import numpy as np
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PointStamped


def bbox_center(bbox: list[float]) -> tuple[int, int]:
    """Return integer pixel (u, v) for the center of [x1, y1, x2, y2]."""
    u = int((bbox[0] + bbox[2]) / 2.0)
    v = int((bbox[1] + bbox[3]) / 2.0)
    return u, v


def sample_depth(depth_image: np.ndarray, u: int, v: int, window: int = 5) -> float:
    """
    Sample depth at (u, v) using a small window median to reduce noise.
    Returns depth in metres. Returns NaN if all samples are invalid.

    Handles both float32 (metres) and uint16 (millimetres) encoding from
    Gazebo depth plugin and real OAK-D hardware.
    """
    h, w = depth_image.shape[:2]

    v0, v1 = max(0, v - window), min(h, v + window + 1)
    u0, u1 = max(0, u - window), min(w, u + window + 1)

    patch = depth_image[v0:v1, u0:u1].astype(np.float32)

    # Millimetre encoding (uint16 from Gazebo or OAK-D hardware)
    if patch.max() > 100.0:
        patch = patch / 1000.0

    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return float("nan")

    return float(np.median(valid))


def deproject_pixel(
    camera_model: PinholeCameraModel,
    u: int,
    v: int,
    depth_m: float,
) -> tuple[float, float, float] | None:
    """
    Back-project pixel (u, v) + depth into a 3D point in the camera
    optical frame using the pinhole camera model intrinsics.

    Returns (x, y, z) in metres, or None if depth is invalid.
    """
    if not np.isfinite(depth_m) or depth_m <= 0.0:
        return None

    ray = camera_model.projectPixelTo3dRay((u, v))  # unit vector in optical frame
    x = ray[0] * depth_m
    y = ray[1] * depth_m
    z = ray[2] * depth_m
    return x, y, z


def make_point_stamped(
    xyz: tuple[float, float, float],
    frame_id: str,
    stamp,
) -> PointStamped:
    """Wrap (x, y, z) into a PointStamped ready for tf2 transform."""
    ps = PointStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = stamp
    ps.point.x = float(xyz[0])
    ps.point.y = float(xyz[1])
    ps.point.z = float(xyz[2])
    return ps