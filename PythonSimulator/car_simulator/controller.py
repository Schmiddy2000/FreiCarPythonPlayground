"""Counterpart of the supplied ROS Controller node, with planar poses and local publishers."""
from collections.abc import Callable
from dataclasses import dataclass, fields
import logging
import numpy as np
from PythonSimulator.car_simulator.messages import (ControlCommand, GoalMessage, Header, PathMessage, Pose2D,
                                                    OdometryMessage, TargetTransform)
from PythonSimulator.car_simulator.pid_controller import PID
from PythonSimulator.car_simulator.transforms import TransformBuffer, TransformLookupError
from PythonSimulator.car_simulator.transport import Publisher


@dataclass(frozen=True)
class ControllerParameters:
    map_frame_id: str = "map"
    robot_frame_id: str = "freicar_1"
    target_frame_id: str | None = None
    wheelbase: float = 0.36
    position_tolerance: float = 0.1
    steering_angle_limit: float = 1.22173
    maximum_velocity: float = 5.0
    throttle_limit: float = 0.25
    curvature_velocity_limit_factor: float = 0.44
    steering_velocity_limit_factor: float = 2.0
    distance_velocity_limit_factor: float = 0.3
    minimum_throttle_limit: float = 0.0
    desired_velocity: float = 0.5
    pid_p: float = 1.25
    pid_i: float = 0.01
    pid_d: float = 0.01
    integral_decay_factor: float = 0.0
    # The corresponding clamp is commented out in the supplied C++.
    apply_throttle_limit: bool = False

    def __post_init__(self) -> None:
        numerical_parameters = [getattr(self, parameter.name) for parameter in fields(self)
                                if isinstance(getattr(self, parameter.name), (int, float))]
        if not np.isfinite(numerical_parameters).all():
            raise ValueError("Controller parameters must be finite.")
        if (self.wheelbase <= 0 or self.position_tolerance <= 0
                or not 0 < self.steering_angle_limit < np.pi / 2
                or self.desired_velocity < 0 or self.maximum_velocity <= 0
                or self.throttle_limit < 0 or not 0 <= self.integral_decay_factor <= 1):
            raise ValueError("Invalid controller parameters.")
        if not self.map_frame_id or not self.robot_frame_id:
            raise ValueError("Map and robot frame names cannot be empty.")


class Controller:
    def __init__(self, parameters: ControllerParameters | None = None,
                 transform_buffer: TransformBuffer | None = None):
        self.parameters = parameters or ControllerParameters()
        self.transform_buffer = transform_buffer or TransformBuffer()
        self.logger = logging.getLogger(type(self).__name__)
        self.map_frame_id = self.parameters.map_frame_id
        self.tracker_frame_id = self.parameters.robot_frame_id
        self.front_axis_frame_id = self.tracker_frame_id + "/front_axis"
        self.rear_axis_frame_id = self.tracker_frame_id + "/rear_axis"
        self.handle_frame_id = self.tracker_frame_id + "/base_link"
        self.target_frame_id = self.parameters.target_frame_id or self.tracker_frame_id + "/lookahead"
        self.wheelbase = self.parameters.wheelbase
        self.position_tolerance = self.parameters.position_tolerance
        self.steering_angle_maximum = self.parameters.steering_angle_limit
        self.desired_velocity = self.parameters.desired_velocity
        self.velocity_pid = PID(self.parameters.pid_p, self.parameters.pid_i, self.parameters.pid_d,
                                decay_factor=self.parameters.integral_decay_factor)

        # Counterparts of pub_acker_, pub_goal_reached_, and the target broadcaster.
        self.control_publisher: Publisher[ControlCommand] = Publisher()
        self.goal_reached_publisher: Publisher[GoalMessage] = Publisher()
        self.target_publisher: Publisher[TargetTransform] = Publisher()
        self.clock: Callable[[], float] = lambda: self.current_time
        self.reset()

    def reset(self) -> None:
        # Simulator restart = new node lifecycle. receive_path() deliberately
        # does NOT reset PID history: the supplied C++ does not do so either.
        self.path: list[Pose2D] = []
        self.path_index = 0
        self.goal_reached = True
        self.completion_advertised = False
        self.current_time = 0.0
        self.current_throttle = 0.0
        self.current_steering_angle = 0.0
        self.control_command = ControlCommand()
        self.target_transform = TargetTransform(
            Header(0.0, self.map_frame_id), self.target_frame_id, Pose2D(np.zeros(2)))
        self.last_error: str | None = None
        self.velocity_pid.reset()

    @property
    def path_positions(self) -> np.ndarray:
        return np.asarray([pose.position for pose in self.path], dtype=float).reshape(-1, 2)

    def transform_path(self, path: PathMessage, target_frame: str) -> list[Pose2D]:
        frame_transform = self.transform_buffer.lookup_transform(target_frame, path.header.frame_id)
        return [frame_transform.transform_pose(pose) for pose in path.poses]

    def receive_path(self, new_path: PathMessage) -> bool:
        if new_path.poses:
            try:
                transformed_path = self.transform_path(new_path, self.map_frame_id)
            except TransformLookupError as exception:
                self.logger.warning("Cannot transform path: %s", exception)
                return False
            self.path = transformed_path
            self.path_index = 0
            self.goal_reached = False
            self.completion_advertised = False
            self.last_error = None
            # Do not change desired_velocity or reset the PID here.
        else:
            self.path.clear()
            self.goal_reached = True
            self.completion_advertised = True
            self.last_error = None
        return True

    def send_goal_msg(self, reached: bool) -> None:
        if not self.completion_advertised:
            self.completion_advertised = True
            self.goal_reached_publisher.publish(GoalMessage(reached))

    def publish_control_command(self, throttle: float, steering_degrees: float,
                                brake: float = 0.0, hand_brake: bool = False) -> None:
        self.control_command = ControlCommand(
            header=Header(self.clock(), self.tracker_frame_id),
            steering=steering_degrees, throttle=throttle, brake=brake,
            hand_brake=hand_brake, throttle_mode="DUTY")
        self.control_publisher.publish(self.control_command)

    def publish_stop_command(self) -> None:
        self.publish_control_command(0.0, 0.0, brake=1.0, hand_brake=True)

    def make_step(self, odometry: OdometryMessage) -> None:
        self.logger.warning("No controller action implemented")
