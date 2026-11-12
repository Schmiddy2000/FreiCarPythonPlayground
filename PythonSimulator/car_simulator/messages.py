"""ROS-like message values without ROS. Pose angles are radians; command steering is DEGREES."""
from dataclasses import dataclass, field
import numpy as np
from numpy.typing import ArrayLike, NDArray


@dataclass(frozen=True)
class Header:
    stamp: float = 0.0  # Simulation seconds, never wall-clock time.
    frame_id: str = "map"


@dataclass(frozen=True)
class Pose2D:
    position: NDArray[np.float64]
    heading: float = 0.0

    def __post_init__(self) -> None:
        position = np.asarray(self.position, dtype=float).copy()
        if position.shape != (2,) or not np.isfinite(position).all() or not np.isfinite(self.heading):
            raise ValueError("Pose2D requires a finite [x, y] position and heading.")
        position.setflags(write=False)
        object.__setattr__(self, "position", position)


@dataclass(frozen=True)
class PathMessage:
    header: Header
    poses: tuple[Pose2D, ...]

    @classmethod
    def from_positions(cls, positions: ArrayLike, frame_id: str = "map",
                       stamp: float = 0.0) -> "PathMessage":
        positions_array = np.asarray(positions, dtype=float)
        if positions_array.size == 0:
            positions_array = np.empty((0, 2))
        if positions_array.ndim != 2 or positions_array.shape[1] != 2:
            raise ValueError("Path positions must have shape (N, 2).")
        return cls(Header(stamp, frame_id), tuple(Pose2D(position) for position in positions_array))


@dataclass(frozen=True)
class OdometryMessage:
    header: Header
    child_frame_id: str
    pose: Pose2D
    # Body-frame velocity: linear_velocity[0] corresponds to odom.twist.linear.x.
    linear_velocity: NDArray[np.float64]


@dataclass(frozen=True)
class ControlCommand:
    header: Header = field(default_factory=Header)
    steering: float = 0.0  # Degrees at the message boundary.
    throttle: float = 0.0
    brake: float = 0.0
    hand_brake: bool = False
    # Symbolic local value; the actual ROS enum integer was not supplied.
    throttle_mode: str = "DUTY"


@dataclass(frozen=True)
class GoalMessage:
    data: bool


@dataclass(frozen=True)
class TargetTransform:
    header: Header
    child_frame_id: str
    pose: Pose2D
