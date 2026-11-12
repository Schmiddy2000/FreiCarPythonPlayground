"""Rear-axle bicycle physics and the adapter from ROS-like commands to motion."""
import numpy as np
from PythonSimulator.car_simulator.messages import ControlCommand, Header, OdometryMessage, Pose2D
from PythonSimulator.car_simulator.models import VehicleState


class BicycleVehicle:
    def __init__(self, initial_state: VehicleState, wheelbase: float = 0.36,
                 acceleration_gain: float = 1.0, linear_drag: float = 0.0,
                 maximum_steering_angle: float = 1.22173,
                 maximum_throttle: float | None = None,
                 brake_deceleration: float = 3.0, hand_brake_deceleration: float = 6.0):
        parameters = (wheelbase, acceleration_gain, linear_drag, maximum_steering_angle,
                      brake_deceleration, hand_brake_deceleration)
        if (not np.isfinite(parameters).all() or wheelbase <= 0 or acceleration_gain <= 0
                or linear_drag < 0 or not 0 < maximum_steering_angle < np.pi / 2
                or min(brake_deceleration, hand_brake_deceleration) <= 0):
            raise ValueError("Invalid bicycle model parameters.")
        if maximum_throttle is not None and (
                not np.isfinite(maximum_throttle) or maximum_throttle <= 0):
            raise ValueError("maximum_throttle must be positive or None.")
        initial_position = np.asarray(initial_state.position, dtype=float)
        if (initial_position.shape != (2,) or not np.isfinite(initial_position).all()
                or not np.isfinite([initial_state.heading, initial_state.velocity]).all()
                or initial_state.velocity < 0):
            raise ValueError("Initial pose must be finite and velocity nonnegative.")
        self.initial_state = VehicleState(initial_position.copy(), float(initial_state.heading),
                                          float(initial_state.velocity))
        self.wheelbase = wheelbase
        self.acceleration_gain = acceleration_gain
        self.linear_drag = linear_drag
        self.maximum_steering_angle = maximum_steering_angle
        self.maximum_throttle = maximum_throttle
        self.brake_deceleration = brake_deceleration
        self.hand_brake_deceleration = hand_brake_deceleration
        self.reset()

    def reset(self) -> None:
        self.state = VehicleState(self.initial_state.position.copy(), self.initial_state.heading,
                                  self.initial_state.velocity)

    def get_odometry(self, stamp: float, map_frame_id: str,
                     tracker_frame_id: str) -> OdometryMessage:
        # Pose is in map coordinates, but twist.linear is in the vehicle frame.
        # A car pointing along world +y still reports body-frame vx = speed.
        return OdometryMessage(
            Header(stamp, map_frame_id), tracker_frame_id,
            Pose2D(self.state.position, self.state.heading),
            np.array([self.state.velocity, 0.0]))

    def apply_limits(self, command: ControlCommand) -> ControlCommand:
        if not np.isfinite([command.throttle, command.steering, command.brake]).all():
            raise ValueError("Control commands must be finite.")
        if not 0 <= command.brake <= 1 or command.throttle_mode != "DUTY":
            raise ValueError("Expected brake in [0, 1] and throttle_mode DUTY.")
        throttle = command.throttle
        if self.maximum_throttle is not None:
            throttle = float(np.clip(throttle, -self.maximum_throttle, self.maximum_throttle))
        steering_limit_degrees = np.rad2deg(self.maximum_steering_angle)
        return ControlCommand(
            command.header, float(np.clip(command.steering, -steering_limit_degrees,
                                          steering_limit_degrees)),
            throttle, command.brake, command.hand_brake, command.throttle_mode)

    def step(self, command: ControlCommand, dt: float) -> VehicleState:
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive.")
        command = self.apply_limits(command)
        # Convert DEGREES to radians exactly once, at the physics boundary.
        steering_angle = np.deg2rad(command.steering)
        previous_velocity = self.state.velocity
        braking_acceleration = command.brake * self.brake_deceleration
        if command.hand_brake:
            braking_acceleration = max(braking_acceleration, self.hand_brake_deceleration)
        # In this toy backend signed throttle can also decelerate. This is a
        # configurable model assumption, not a reconstruction of the CARLA adapter.
        acceleration = (self.acceleration_gain * command.throttle
                        - self.linear_drag * previous_velocity - braking_acceleration)
        updated_velocity = max(0.0, previous_velocity + acceleration * dt)
        moving_time = dt
        if acceleration < 0 and previous_velocity + acceleration * dt < 0:
            moving_time = previous_velocity / -acceleration
        travelled_distance = max(
            0.0, previous_velocity * moving_time + 0.5 * acceleration * moving_time ** 2)

        # Hold steering and acceleration constant; integrate the resulting arc
        # using the travelled distance, including partial-interval braking.
        curvature = np.tan(steering_angle) / self.wheelbase
        heading_change = curvature * travelled_distance
        middle_heading = self.state.heading + heading_change / 2
        displacement_length = travelled_distance * np.sinc(heading_change / (2 * np.pi))
        updated_position = self.state.position + displacement_length * np.array([
            np.cos(middle_heading), np.sin(middle_heading)])
        updated_heading = np.arctan2(np.sin(self.state.heading + heading_change),
                                     np.cos(self.state.heading + heading_change))
        self.state = VehicleState(updated_position, float(updated_heading), float(updated_velocity))
        return self.state
