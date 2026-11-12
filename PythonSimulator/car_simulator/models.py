"""Physical vehicle state: metres, seconds, and radians; position is the rear axle."""
from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class VehicleState:
    position: NDArray[np.float64]
    heading: float
    velocity: float
