"""Odometry -> controller callback -> published command -> simulated vehicle."""
from copy import deepcopy
from pathlib import Path
import numpy as np
from PythonSimulator.car_simulator.controller import Controller
from PythonSimulator.car_simulator.maps import MapLoader, SimulationMap
from PythonSimulator.car_simulator.messages import ControlCommand, GoalMessage, OdometryMessage, PathMessage, TargetTransform
from .transforms import Transform2D
from .transport import Publisher
from .vehicle import BicycleVehicle


class Simulator:
    def __init__(self, controller: Controller, vehicle: BicycleVehicle,
                 dt: float = 0.02, maximum_time: float = 60.0,
                 stopped_velocity_tolerance: float = 0.001):
        if (not np.isfinite([dt, maximum_time, stopped_velocity_tolerance]).all()
                or min(dt, maximum_time, stopped_velocity_tolerance) <= 0):
            raise ValueError("Time intervals and stopped velocity tolerance must be positive.")
        self.controller = controller
        self.vehicle = vehicle
        self.dt = dt
        self.maximum_time = maximum_time
        self.stopped_velocity_tolerance = stopped_velocity_tolerance
        self.simulation_map: SimulationMap | None = None
        self.active_path_message: PathMessage | None = None
        self.path_publisher: Publisher[PathMessage] = Publisher(transient_local=True)
        self.odometry_publisher: Publisher[OdometryMessage] = Publisher()
        self.path_publisher.subscribe(self._deliver_path)
        self.odometry_publisher.subscribe(self.controller.make_step)
        self.controller.control_publisher.subscribe(self.receive_control_command)
        self.controller.goal_reached_publisher.subscribe(self.receive_goal_message)
        self.controller.target_publisher.subscribe(self.receive_target_transform)
        self.controller.clock = lambda: self.current_time
        self.reset()

    def reset(self) -> None:
        saved_path = deepcopy(self.active_path_message)
        self.controller.reset()
        self.vehicle.reset()
        self.current_time = 0.0
        # With no published control yet, the backend holds the vehicle stopped.
        self.control_command = ControlCommand(brake=1.0, hand_brake=True)
        self.target_transform: TargetTransform | None = None
        self.stop_reason: str | None = None
        self.goal_event_received = False
        self.odometry_messages: list[OdometryMessage] = []
        self.goal_messages: list[GoalMessage] = []
        self.history = {
            "time": [0.0], "position": [self.vehicle.state.position.copy()],
            "heading": [self.vehicle.state.heading], "velocity": [self.vehicle.state.velocity],
            "control_time": [], "throttle": [], "requested_throttle": [],
            "steering_angle": [], "requested_steering_angle": [],
            "steering_degrees": [], "brake": [], "hand_brake": [],
            "target_position": [], "target_valid": [], "desired_velocity": [], "integral": [],
            "path_index": [], "goal_time": [], "goal_reached": []}
        self._update_vehicle_transforms()
        if saved_path is not None:
            self.path_publisher.publish(saved_path)

    def load_map(self, loader: MapLoader, source: Path | None = None) -> SimulationMap:
        loaded_map = loader.load(source)
        if loaded_map.frame_id != self.controller.map_frame_id:
            raise ValueError("Map display geometry must be in the configured map frame.")
        self.simulation_map = loaded_map
        # Loading a map does not choose a route or alter the controller.
        return loaded_map

    def publish_route(self, route_name: str) -> None:
        if self.simulation_map is None:
            raise RuntimeError("Load a map before choosing one of its routes.")
        self.publish_path(self.simulation_map.routes[route_name])

    def publish_path(self, path: PathMessage) -> None:
        self.path_publisher.publish(path)

    def _deliver_path(self, path: PathMessage) -> None:
        if self.controller.receive_path(path):
            self.active_path_message = deepcopy(path)
            self.target_transform = None
            self.goal_event_received = False
            self.stop_reason = None

    def receive_control_command(self, command: ControlCommand) -> None:
        self.control_command = command

    def receive_goal_message(self, message: GoalMessage) -> None:
        self.goal_messages.append(message)
        self.history["goal_time"].append(self.current_time)
        self.history["goal_reached"].append(message.data)
        self.goal_event_received = message.data

    def receive_target_transform(self, target: TargetTransform) -> None:
        self.target_transform = target
        self.controller.transform_buffer.set_transform(
            target.header.frame_id, target.child_frame_id,
            Transform2D(target.pose.position, target.pose.heading))

    def _update_vehicle_transforms(self) -> None:
        state = self.vehicle.state
        controller = self.controller
        controller.transform_buffer.set_transform(
            controller.map_frame_id, controller.tracker_frame_id,
            Transform2D(state.position, state.heading))
        # This model places tracker, rear axle, and base_link at the rear axle.
        # Their names still match C++; the front axle is one wheelbase ahead.
        for frame_name, offset in (
                (controller.rear_axis_frame_id, np.zeros(2)),
                (controller.handle_frame_id, np.zeros(2)),
                (controller.front_axis_frame_id, np.array([self.vehicle.wheelbase, 0.0]))):
            controller.transform_buffer.set_transform(
                controller.tracker_frame_id, frame_name, Transform2D(offset))

    @property
    def finished(self) -> bool:
        if self.stop_reason is not None:
            return True
        # Goal status comes from the controller's publisher, not from a second
        # independent goal test inside the simulator. Continue applying the
        # published brake until the physical model actually stops.
        if self.odometry_messages and self.vehicle.state.velocity <= self.stopped_velocity_tolerance:
            if self.controller.last_error is not None:
                self.stop_reason = "controller_error"
            elif self.goal_event_received:
                self.stop_reason = "goal_reached"
            elif len(self.controller.path) < 2:
                self.stop_reason = "no_path" if not self.controller.path else "invalid_path"
        if self.stop_reason is None and self.current_time >= self.maximum_time - 1e-12:
            self.stop_reason = "maximum_time"
        return self.stop_reason is not None

    def step(self) -> bool:
        if self.finished:
            return False
        time_interval = min(self.dt, self.maximum_time - self.current_time)
        self._update_vehicle_transforms()
        odometry = self.vehicle.get_odometry(
            self.current_time, self.controller.map_frame_id, self.controller.tracker_frame_id)
        self.odometry_messages.append(odometry)
        # This synchronous publication triggers PurePursuit.make_step(), which
        # publishes to receive_control_command(). The simulator never calls a
        # steering helper or computes throttle on the controller's behalf.
        self.odometry_publisher.publish(odometry)
        requested_command = self.control_command
        applied_command = self.vehicle.apply_limits(requested_command)
        target_position = (self.target_transform.pose.position.copy()
                           if self.target_transform is not None else self.vehicle.state.position.copy())
        values = {
            "control_time": self.current_time,
            "requested_throttle": requested_command.throttle,
            "requested_steering_angle": np.deg2rad(requested_command.steering),
            "throttle": applied_command.throttle,
            "steering_angle": np.deg2rad(applied_command.steering),
            "steering_degrees": applied_command.steering,
            "brake": applied_command.brake, "hand_brake": applied_command.hand_brake,
            "target_position": target_position, "target_valid": self.target_transform is not None,
            "desired_velocity": self.controller.desired_velocity,
            "integral": self.controller.velocity_pid.integral, "path_index": self.controller.path_index}
        for name, value in values.items():
            self.history[name].append(value)
        updated_state = self.vehicle.step(applied_command, time_interval)
        self.current_time += time_interval
        for name, value in (("time", self.current_time), ("position", updated_state.position.copy()),
                            ("heading", updated_state.heading), ("velocity", updated_state.velocity)):
            self.history[name].append(value)
        return True

    def run(self) -> dict[str, np.ndarray]:
        while self.step():
            pass
        return self.get_data()

    def get_data(self) -> dict[str, np.ndarray]:
        recorded_data = {name: np.asarray(values) for name, values in self.history.items()}
        recorded_data["target_position"] = recorded_data["target_position"].reshape(-1, 2)
        return recorded_data
