import numpy as np

from typing import Tuple
from numpy.typing import NDArray


def rot2(theta: float) -> NDArray:
    """Return a float ndarray of shape (2, 2) for scalar theta."""

    # Define the rotation matrix
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    return rotation_matrix


# Helper function to create composed 3x3 matrices
def compose_pose(r: NDArray, t: NDArray) -> NDArray:
    """r is the rotation matrix and t is the 2-component (x, y) translation vector."""
    # Create additional components
    zeros = np.array([[0, 0]])
    position_vector = np.array([[t[0], t[1], 1]])

    # Create the pose matrix from the components (including the rotation matrix r)
    pose_matrix = np.concatenate((r, zeros))
    pose_matrix = np.concatenate((pose_matrix, position_vector.T), axis=1)

    return pose_matrix


# Helper function to extract pose components from 3x3 matrix
def extract_pose_components(pose: NDArray) -> Tuple[NDArray, NDArray]:

    rotation_matrix = pose[:2, :2]
    translation_vector = pose[:2, 2]

    return rotation_matrix, translation_vector


def make_pose(x: float, y: float, theta: float) -> NDArray:
    """Return T_A_B, shape (3, 3); (x,y) is B's origin expressed in A."""

    # Generate the rotation matrix from the given theta and combine x, y to the translation vector
    rotation_matrix = rot2(theta)
    translation_vector = np.array([x, y])

    # Create the final pose matrix T_A_B using our helper compose function
    T_A_B = compose_pose(rotation_matrix, translation_vector)

    return T_A_B


def invert_pose(T_A_B: NDArray) -> NDArray:
    """Return T_B_A using blocks; do not call np.linalg.inv/solve here."""

    rotation_matrix, translation_vector = extract_pose_components(T_A_B)
    inverse_translation_vector = np.matmul(-rotation_matrix.T, translation_vector)

    T_B_A = compose_pose(rotation_matrix.T, inverse_translation_vector)

    return T_B_A


def transform_points(T_A_B: NDArray, points_B: NDArray) -> NDArray:
    """Return points_A, shape (N,2). Empty (0,2) input stays (0,2)."""

    # Handle empty case
    if np.shape(points_B) == (0, 2):
        return points_B

    # Helper array
    new_points = []

    # In all other cases
    for points in points_B:
        transformed_points = np.matmul(T_A_B, np.array([points[0], points[1], 1]).T)[:2]
        new_points.append(list(transformed_points))

    return np.array(new_points)


def get_translation_vector(xy: NDArray) -> NDArray:
    """Return translation vector (x,y)."""
    translation_vector = np.array([xy[0], xy[1], 1])

    return translation_vector