"""Planar tf2 counterparts. lookup_transform(target, source) is target_from_source."""
from collections import deque
from dataclasses import dataclass, field
import numpy as np
from numpy.typing import NDArray
from PythonSimulator.car_simulator.messages import Pose2D


class TransformLookupError(LookupError):
    pass


@dataclass(frozen=True)
class Transform2D:
    translation: NDArray[np.float64] = field(default_factory=lambda: np.zeros(2))
    heading: float = 0.0

    def __post_init__(self) -> None:
        validated_pose = Pose2D(self.translation, self.heading)
        object.__setattr__(self, "translation", validated_pose.position)

    @property
    def rotation(self) -> NDArray[np.float64]:
        heading_cosine, heading_sine = np.cos(self.heading), np.sin(self.heading)
        return np.array([[heading_cosine, -heading_sine], [heading_sine, heading_cosine]])

    def transform_position(self, position: NDArray[np.float64]) -> NDArray[np.float64]:
        return self.rotation @ position + self.translation

    def transform_pose(self, pose: Pose2D) -> Pose2D:
        return Pose2D(self.transform_position(pose.position), pose.heading + self.heading)

    def inverse(self) -> "Transform2D":
        return Transform2D(-self.rotation.T @ self.translation, -self.heading)

    def compose(self, source_transform: "Transform2D") -> "Transform2D":
        # target_from_middle.compose(middle_from_source) -> target_from_source.
        return Transform2D(self.transform_position(source_transform.translation),
                           self.heading + source_transform.heading)


class TransformBuffer:
    def __init__(self):
        self.transforms: dict[tuple[str, str], Transform2D] = {}

    def set_transform(self, target_frame: str, source_frame: str,
                      target_from_source: Transform2D) -> None:
        if not target_frame or not source_frame or target_frame == source_frame:
            raise ValueError("A transform needs two different, nonempty frame names.")
        self.transforms[target_frame, source_frame] = target_from_source
        self.transforms[source_frame, target_frame] = target_from_source.inverse()

    def lookup_transform(self, target_frame: str, source_frame: str) -> Transform2D:
        if target_frame == source_frame:
            return Transform2D()
        pending_frames = deque([(source_frame, Transform2D())])
        visited_frames = {source_frame}
        while pending_frames:
            current_frame, current_from_source = pending_frames.popleft()
            for (next_frame, stored_source), next_from_current in self.transforms.items():
                if stored_source != current_frame or next_frame in visited_frames:
                    continue
                next_from_source = next_from_current.compose(current_from_source)
                if next_frame == target_frame:
                    return next_from_source
                visited_frames.add(next_frame)
                pending_frames.append((next_frame, next_from_source))
        raise TransformLookupError(f"Cannot transform {source_frame!r} into {target_frame!r}.")
