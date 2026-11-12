"""Map loading boundary: replace a loader when the real map format is settled.

A map supplies display geometry and named example routes. The simulator loads
the map; it publishes a selected route separately, like the ROS path publisher.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from .messages import PathMessage
from .paths import make_example_path


@dataclass(frozen=True)
class SimulationMap:
    frame_id: str
    routes: dict[str, PathMessage]
    boundary_lines: tuple[np.ndarray, ...] = ()

    def __post_init__(self) -> None:
        if not self.frame_id:
            raise ValueError("A map needs a frame name.")
        for boundary in self.boundary_lines:
            boundary_array = np.asarray(boundary, dtype=float)
            if (boundary_array.ndim != 2 or boundary_array.shape[1] != 2
                    or not np.isfinite(boundary_array).all()):
                raise ValueError("Map boundaries must contain finite [x, y] points.")


class MapLoader(ABC):
    @abstractmethod
    def load(self, source: Path | None = None) -> SimulationMap:
        """Convert a future map file/source into the simulator's map values."""


class DummyMapLoader(MapLoader):
    def __init__(self, frame_id: str = "map", lane_width: float = 0.8):
        if not np.isfinite(lane_width) or lane_width <= 0:
            raise ValueError("lane_width must be positive.")
        self.frame_id = frame_id
        self.lane_width = lane_width

    def load(self, source: Path | None = None) -> SimulationMap:
        if source is not None:
            raise ValueError("DummyMapLoader generates a map; it does not read a file format.")
        path_positions = make_example_path()
        tangent_vectors = np.gradient(path_positions, axis=0)
        normal_vectors = np.column_stack((-tangent_vectors[:, 1], tangent_vectors[:, 0]))
        normal_vectors /= np.linalg.norm(normal_vectors, axis=1, keepdims=True)
        boundary_offset = self.lane_width / 2 * normal_vectors
        return SimulationMap(
            frame_id=self.frame_id,
            routes={"example": PathMessage.from_positions(path_positions, self.frame_id)},
            boundary_lines=(path_positions + boundary_offset, path_positions - boundary_offset))
