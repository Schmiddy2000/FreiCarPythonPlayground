"""Live movement, map geometry, and recorded diagnostics."""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, FancyArrowPatch, Polygon
from .simulator import Simulator


class SimulationVisualization:
    def __init__(self, simulator: Simulator, frames_per_second: float = 30,
                 playback_speed: float = 1.0):
        if (not np.isfinite([frames_per_second, playback_speed]).all()
                or min(frames_per_second, playback_speed) <= 0):
            raise ValueError("Frame rate and playback speed must be positive.")
        self.simulator = simulator
        self.frames_per_second = frames_per_second
        self.playback_speed = playback_speed
        self.animation: FuncAnimation | None = None
        self.figure, self.axes = plt.subplots(figsize=(10, 6))
        self.reference_line, = self.axes.plot([], [], "g.-", markersize=2, label="Published path")
        self.goal_point, = self.axes.plot([], [], "k*", markersize=12, label="Goal")
        self.trajectory_line, = self.axes.plot([], [], color="0.5", label="Rear-axle trajectory")
        self.wheelbase_line, = self.axes.plot([], [], "r-", linewidth=2)
        self.car_body = Polygon(np.zeros((4, 2)), closed=True, color="red", alpha=0.3)
        self.axes.add_patch(self.car_body)
        self.target_arrow = FancyArrowPatch(
            (0, 0), (0, 0), arrowstyle="->", mutation_scale=14,
            color="blue", shrinkA=0, shrinkB=0)
        self.axes.add_patch(self.target_arrow)
        self.target_point, = self.axes.plot([], [], "bo", markersize=5, label="Pursuit target")
        look_ahead_distance = getattr(simulator.controller, "look_ahead_distance", 0.0)
        self.look_ahead_circle = Circle(
            simulator.vehicle.state.position, look_ahead_distance,
            fill=False, color="blue", alpha=0.25, linestyle="--")
        self.axes.add_patch(self.look_ahead_circle)
        self.status_text = self.axes.text(
            0.02, 0.98, "", transform=self.axes.transAxes, va="top",
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"})
        wheelbase = simulator.vehicle.wheelbase
        half_width = 0.3 * wheelbase
        self.body_corners = np.array([
            [0, -half_width], [wheelbase, -half_width],
            [wheelbase, half_width], [0, half_width]])
        self.displayed_map = None
        self.map_lines = []
        self._refresh_map_geometry()
        extent_parts = [simulator.controller.path_positions,
                        simulator.vehicle.state.position.reshape(1, 2)]
        if simulator.simulation_map is not None:
            extent_parts.extend(simulator.simulation_map.boundary_lines)
        extent = np.vstack(extent_parts)
        margin = max(1.0, look_ahead_distance, 2 * wheelbase)
        self.axes.set_xlim(extent[:, 0].min() - margin, extent[:, 0].max() + margin)
        self.axes.set_ylim(extent[:, 1].min() - margin, extent[:, 1].max() + margin)
        self.axes.set_aspect("equal", adjustable="box")
        self.axes.set(xlabel="World x [m]", ylabel="World y [m]", title="Car simulator")
        self.axes.grid(alpha=0.25)
        self.axes.legend(loc="lower right")
        self.figure.tight_layout()
        self.draw_state()

    def _refresh_map_geometry(self) -> None:
        if self.displayed_map is self.simulator.simulation_map:
            return
        for map_line in self.map_lines:
            map_line.remove()
        self.map_lines = []
        self.displayed_map = self.simulator.simulation_map
        if self.displayed_map is not None:
            for boundary_index, boundary in enumerate(self.displayed_map.boundary_lines):
                boundary = np.asarray(boundary)
                map_line, = self.axes.plot(
                    boundary[:, 0], boundary[:, 1], color="0.25", linestyle="--",
                    alpha=0.5, label="Map boundaries" if boundary_index == 0 else None)
                self.map_lines.append(map_line)

    def draw_state(self) -> None:
        self._refresh_map_geometry()
        path = self.simulator.controller.path_positions
        self.reference_line.set_data(path[:, 0], path[:, 1])
        if len(path):
            self.goal_point.set_data([path[-1, 0]], [path[-1, 1]])
        else:
            self.goal_point.set_data([], [])
        vehicle_state = self.simulator.vehicle.state
        heading_cosine, heading_sine = np.cos(vehicle_state.heading), np.sin(vehicle_state.heading)
        robot_to_world_rotation = np.array([
            [heading_cosine, -heading_sine], [heading_sine, heading_cosine]])
        self.car_body.set_xy(self.body_corners @ robot_to_world_rotation.T + vehicle_state.position)
        front_axle = vehicle_state.position + self.simulator.vehicle.wheelbase * np.array([
            heading_cosine, heading_sine])
        self.wheelbase_line.set_data(
            [vehicle_state.position[0], front_axle[0]], [vehicle_state.position[1], front_axle[1]])
        positions = np.asarray(self.simulator.history["position"])
        self.trajectory_line.set_data(positions[:, 0], positions[:, 1])
        self.look_ahead_circle.center = tuple(vehicle_state.position)
        target_visible = (self.simulator.target_transform is not None
                          and not self.simulator.controller.goal_reached
                          and not self.simulator.finished)
        self.look_ahead_circle.set_visible(target_visible)
        self.target_arrow.set_visible(target_visible)
        self.target_point.set_visible(target_visible)
        if self.simulator.target_transform is not None:
            target_position = self.simulator.target_transform.pose.position
            self.target_arrow.set_positions(vehicle_state.position, target_position)
            self.target_point.set_data([target_position[0]], [target_position[1]])
        applied_steering = (self.simulator.history["steering_degrees"][-1]
                            if self.simulator.history["steering_degrees"] else 0.0)
        self.status_text.set_text(
            f"Time: {self.simulator.current_time:.2f} s\n"
            f"Velocity: {vehicle_state.velocity:.3f} m/s\n"
            f"Steering: {applied_steering:.1f} degrees\n"
            f"Status: {self.simulator.stop_reason or 'running'}")
        horizontal_limits, vertical_limits = self.axes.get_xlim(), self.axes.get_ylim()
        if not horizontal_limits[0] <= vehicle_state.position[0] <= horizontal_limits[1]:
            self.axes.set_xlim(min(horizontal_limits[0], vehicle_state.position[0] - 1),
                               max(horizontal_limits[1], vehicle_state.position[0] + 1))
        if not vertical_limits[0] <= vehicle_state.position[1] <= vertical_limits[1]:
            self.axes.set_ylim(min(vertical_limits[0], vehicle_state.position[1] - 1),
                               max(vertical_limits[1], vehicle_state.position[1] + 1))

    def _advance_frame(self, _frame_number: int) -> None:
        self.next_frame_time += self.playback_speed / self.frames_per_second
        while self.simulator.current_time < self.next_frame_time and self.simulator.step():
            pass
        self.draw_state()
        if self.simulator.finished and self.animation is not None:
            self.animation.event_source.stop()

    def show_live(self) -> None:
        self.next_frame_time = self.simulator.current_time
        self.animation = FuncAnimation(
            self.figure, self._advance_frame, init_func=self.draw_state,
            interval=1000 / self.frames_per_second, cache_frame_data=False,
            blit=False, repeat=False)
        plt.show()

    def plot_diagnostics(self):
        data = self.simulator.get_data()
        figure, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
        axes[0].plot(data["time"], data["velocity"], label="Measured velocity")
        axes[0].plot(data["control_time"], data["desired_velocity"], "--",
                     label="Configured desired velocity")
        axes[0].set_ylabel("Velocity [m/s]")
        axes[1].plot(data["control_time"], data["throttle"], label="Applied throttle")
        axes[1].plot(data["control_time"], data["requested_throttle"], "--", alpha=0.6,
                     label="Requested throttle")
        axes[1].plot(data["control_time"], data["brake"], label="Brake")
        axes[1].plot(data["control_time"], data["hand_brake"], ":", label="Hand brake")
        axes[1].set_ylabel("Control")
        axes[2].plot(data["control_time"], data["steering_degrees"], label="Applied steering")
        axes[2].set_ylabel("Steering [degrees]")
        axes[3].plot(data["control_time"], data["integral"], label="PID integral")
        axes[3].set(xlabel="Time [s]", ylabel="Integral [m]")
        for axis in axes:
            axis.grid(alpha=0.25)
            axis.legend()
        figure.tight_layout()
        return figure
