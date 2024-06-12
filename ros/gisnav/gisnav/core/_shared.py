"""Shared static functions and constants for core nodes"""
from typing import Final, Optional, Tuple

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo

# TODO: make error model and generate covariance matrix dynamically
# Create dummy covariance matrix
_covariance_matrix = np.zeros((6, 6))
np.fill_diagonal(_covariance_matrix, 9)  # 3 meter SD = 9 variance
_covariance_matrix[3, 3] = np.radians(5**2)  # angle error should be set quite small
_covariance_matrix[4, 4] = _covariance_matrix[3, 3]
_covariance_matrix[5, 5] = _covariance_matrix[3, 3]
COVARIANCE_LIST: Final = _covariance_matrix.flatten().tolist()


def visualize_matches_and_pose(
    camera_info: CameraInfo,
    qry: np.ndarray,
    ref: np.ndarray,
    mkp_qry: np.ndarray,
    mkp_ref: np.ndarray,
    r: np.ndarray,
    t: np.ndarray,
) -> np.ndarray:
    """Visualizes matches and projected FOV"""

    def _project_fov(img, h_matrix):
        """Projects FOV on reference image"""
        height, width = img.shape[0:2]
        src_pts = np.float32(
            [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]]
        ).reshape(-1, 1, 2)
        try:
            return cv2.perspectiveTransform(src_pts, np.linalg.inv(h_matrix))
        except np.linalg.LinAlgError:
            return src_pts

    k = camera_info.k.reshape((3, 3))
    h_matrix = k @ np.delete(np.hstack((r, t)), 2, 1)
    projected_fov = _project_fov(qry, h_matrix)

    projected_fov[:, :, 1] = projected_fov[:, :, 1]
    img_with_fov = cv2.polylines(
        ref, [np.int32(projected_fov)], True, 255, 3, cv2.LINE_AA
    )

    mkp_qry = [cv2.KeyPoint(x[0], x[1], 1) for x in mkp_qry]
    mkp_ref = [cv2.KeyPoint(x[0], x[1], 1) for x in mkp_ref]

    matches = [cv2.DMatch(i, i, 0) for i in range(len(mkp_qry))]

    # Publish keypoint match image to ROS for debugging
    match_img = cv2.drawMatches(
        img_with_fov,
        mkp_ref,
        qry,
        mkp_qry,
        matches,
        None,
        matchColor=(0, 255, 0),
        flags=2,
    )
    match_img = cv2.cvtColor(match_img, cv2.COLOR_BGR2GRAY)
    return match_img


def compute_pose(
    camera_info: CameraInfo,
    mkp_qry: np.ndarray,
    mkp_ref: np.ndarray,
    elevation: np.ndarray,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    def _compute_3d_points(mkp_ref: np.ndarray, elevation: np.ndarray) -> np.ndarray:
        """Computes 3D points from matches"""
        if elevation is None:
            return np.hstack((mkp_ref, np.zeros((len(mkp_ref), 1))))

        x, y = np.transpose(np.floor(mkp_ref).astype(int))
        z_values = elevation[y, x].reshape(-1, 1)
        return np.hstack((mkp_ref, z_values))

    def _solve_pnp(
        mkp2_3d: np.ndarray, mkp_qry: np.ndarray, k_matrix: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Computes :term:`pose` using :func:`cv2.solvePnPRansac`"""
        dist_coeffs = np.zeros((4, 1))
        _, r, t, _ = cv2.solvePnPRansac(
            mkp2_3d,
            mkp_qry,
            k_matrix,
            dist_coeffs,
            useExtrinsicGuess=False,
            iterationsCount=10,
        )
        r_matrix, _ = cv2.Rodrigues(r)

        return r_matrix, t

    mkp2_3d = _compute_3d_points(mkp_ref, elevation)
    k_matrix = camera_info.k.reshape((3, 3))
    r, t = _solve_pnp(mkp2_3d, mkp_qry, k_matrix)

    return r, t
