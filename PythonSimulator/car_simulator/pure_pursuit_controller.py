"""Port of the supplied WIP PurePursuit, keeping its algorithmic structure."""
from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray
from PythonSimulator.car_simulator.controller import Controller, ControllerParameters
from .messages import Header, OdometryMessage, Pose2D, TargetTransform
from PythonSimulator.car_simulator.transforms import Transform2D, TransformBuffer


@dataclass(frozen=True)
class PurePursuitParameters:
    look_ahead_distance: float = 0.7
    curvature_look_ahead_distance: float = 1.0
    log_velocity_diagnostics: bool = False

    def __post_init__(self) -> None:
        radii = (self.look_ahead_distance, self.curvature_look_ahead_distance)
        if not np.isfinite(radii).all() or min(radii) <= 0:
            raise ValueError("Look-ahead distances must be finite and positive.")


class PurePursuit(Controller):
    def __init__(self, parameters: ControllerParameters | None = None,
                 pursuit_parameters: PurePursuitParameters | None = None,
                 transform_buffer: TransformBuffer | None = None):
        super().__init__(parameters, transform_buffer)
        self.pursuit_parameters = pursuit_parameters or PurePursuitParameters()
        self.look_ahead_distance = self.pursuit_parameters.look_ahead_distance
        # Stored but intentionally unused, as ld_curvature is in this C++ version.
        self.curvature_look_ahead_distance = self.pursuit_parameters.curvature_look_ahead_distance
        self.previous_diagnostic_time = -np.inf

    def reset(self) -> None:
        super().reset()
        self.previous_diagnostic_time = -np.inf

    @staticmethod
    def get_distance(first: NDArray[np.float64], second: NDArray[np.float64]) -> float:
        return float(np.hypot(first[0] - second[0], first[1] - second[1]))

    def reached_goal(self, car_position: NDArray[np.float64]) -> bool:
        return bool(self.path and self.get_distance(car_position, self.path[-1].position)
                    < self.position_tolerance)

    def get_intersection_tuple_indices(self, center: NDArray[np.float64],
                                       radius: float) -> tuple[int, int]:
        if len(self.path) < 2:
            raise ValueError("An intersection tuple needs at least two path poses.")
        was_inside = self.get_distance(center, self.path[self.path_index].position) < radius
        for path_index in range(self.path_index + 1, len(self.path)):
            rear_axle_distance = self.get_distance(center, self.path[path_index].position)
            if rear_axle_distance < radius:
                if not was_inside:
                    self.path_index = path_index
                was_inside = True
            if rear_axle_distance > radius and was_inside:
                return path_index - 1, path_index
        # Retain the WIP fallback. Its LINE may intersect outside the segment.
        return len(self.path) - 2, len(self.path) - 1

    @staticmethod
    def get_eta(first_endpoint: NDArray[np.float64], second_endpoint: NDArray[np.float64],
                radius: float) -> float:
        segment_direction = second_endpoint - first_endpoint
        quadratic_a = float(np.dot(segment_direction, segment_direction))
        half_quadratic_b = float(np.dot(first_endpoint, segment_direction))
        quadratic_q = float(np.dot(first_endpoint, first_endpoint) - radius * radius)
        discriminant = half_quadratic_b ** 2 - quadratic_a * quadratic_q
        if quadratic_a <= 1e-24:
            raise ValueError("The selected path endpoints coincide.")
        if discriminant < -1e-12:
            raise ValueError("The selected path line does not intersect the look-ahead circle.")
        # Same '+' root as getEta(). Do not clamp the fraction to [0, 1].
        intersection_fraction = (
            -half_quadratic_b + np.sqrt(max(0.0, discriminant))) / quadratic_a
        intersection = first_endpoint + intersection_fraction * segment_direction
        return float(np.arctan2(intersection[1], intersection[0]))

    def log_velocity_diagnostics(self, odometry: OdometryMessage) -> None:
        if odometry.header.stamp - self.previous_diagnostic_time < 0.5:
            return
        self.previous_diagnostic_time = odometry.header.stamp
        velocity = odometry.linear_velocity
        self.logger.info(
            "target=%.3f vx=%.3f vy=%.3f speed_xy=%.3f throttle=%.3f",
            self.desired_velocity, velocity[0], velocity[1], np.hypot(*velocity), self.current_throttle)

    def make_step(self, odometry: OdometryMessage) -> None:
        self.current_time = odometry.header.stamp
        self.last_error = None

        # 0. PID first: longitudinal BODY velocity and the odometry timestamp.
        self.current_throttle = self.velocity_pid.step(
            self.desired_velocity - odometry.linear_velocity[0], odometry.header.stamp)
        if self.parameters.apply_throttle_limit:
            self.current_throttle = float(np.clip(
                self.current_throttle, 0.0, self.parameters.throttle_limit))
        if self.pursuit_parameters.log_velocity_diagnostics:
            self.log_velocity_diagnostics(odometry)

        # 1. As in C++, the odometry pose is assumed to be map_from_rear.
        if odometry.header.frame_id != self.map_frame_id:
            self.last_error = "Odometry pose must be expressed in the configured map frame."
            self.publish_stop_command()
            return
        map_from_rear = Transform2D(odometry.pose.position, odometry.pose.heading)
        rear_axle_position = map_from_rear.translation

        # Unlike the C++ early return, publish a stop for an empty/short route
        # so a previously published throttle does not stay active.
        if len(self.path) < 2:
            self.publish_stop_command()
            return

        # Check the goal before geometry so an initially reached goal can brake
        # even if the WIP final-pair geometry would not have a valid intersection.
        if self.reached_goal(rear_axle_position) or self.goal_reached:
            self.goal_reached = True
            self.velocity_pid.reset_integral()
            self.send_goal_msg(True)
            self.current_steering_angle = 0.0
            self.publish_stop_command()
            return

        # 2. Index pair, with the same mutable idx_ progression as C++.
        first_index, second_index = self.get_intersection_tuple_indices(
            rear_axle_position, self.look_ahead_distance)

        # 3. Invert map_from_rear and transform just these two endpoints.
        rear_from_map = map_from_rear.inverse()
        first_endpoint_rear = rear_from_map.transform_position(self.path[first_index].position)
        second_endpoint_rear = rear_from_map.transform_position(self.path[second_index].position)

        # 4. Calculate eta and steering using the supplied formulas.
        try:
            eta = self.get_eta(first_endpoint_rear, second_endpoint_rear, self.look_ahead_distance)
        except ValueError as exception:
            self.last_error = str(exception)
            self.logger.warning("Cannot compute pursuit target: %s", exception)
            self.publish_stop_command()
            return
        steering_angle = np.arctan(
            2 * self.wheelbase * np.sin(eta) / self.look_ahead_distance)
        self.current_steering_angle = float(steering_angle)

        # Visualization instrumentation only: it does not change control output.
        target_rear = self.look_ahead_distance * np.array([np.cos(eta), np.sin(eta)])
        target_map = map_from_rear.transform_position(target_rear)
        self.target_transform = TargetTransform(
            Header(self.clock(), self.map_frame_id), self.target_frame_id, Pose2D(target_map))
        self.target_publisher.publish(self.target_transform)

        # 5. Degrees at the message boundary; no extra speed policy or clamp.
        self.publish_control_command(self.current_throttle, float(np.rad2deg(steering_angle)))
