"""Example paths; replace these with your own array of [x, y] points."""

import numpy as np


def make_example_path() -> np.ndarray:
    # An open S-shaped path with a gentle start and finish.
    horizontal_positions = np.linspace(0.0, 12.0, 121)
    vertical_positions = 1.5 * np.sin(2 * np.pi * horizontal_positions / 12.0)
    return np.column_stack((horizontal_positions, vertical_positions))
