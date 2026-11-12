"""All experiment parameters and startup wiring for the C++-shaped simulation."""
import argparse
import logging
from pathlib import Path
import numpy as np
from car_simulator.controller import ControllerParameters
from car_simulator.maps import DummyMapLoader, MapLoader
from car_simulator.models import VehicleState
from car_simulator.pure_pursuit_controller import PurePursuit, PurePursuitParameters
from car_simulator.simulator import Simulator
from car_simulator.vehicle import BicycleVehicle

# Controller parameters: defaults intentionally match the supplied C++.
MAP_FRAME_ID = "map"
ROBOT_FRAME_ID = "freicar_1"
TARGET_FRAME_ID = ROBOT_FRAME_ID + "/lookahead"
WHEELBASE = 0.36
POSITION_TOLERANCE = 0.1
STEERING_ANGLE_LIMIT = 1.22173  # Radians internally; command messages use degrees.
DESIRED_VELOCITY = 0.5
PID_P = 1.25
PID_I = 0.01
PID_D = 0.01
INTEGRAL_DECAY_FACTOR = 0.0  # Set this to try the Python decay experiment.
THROTTLE_LIMIT = 0.25
APPLY_THROTTLE_LIMIT = False  # The C++ clamp is currently commented out.

# Retain these C++ settings without inventing their unfinished speed policy.
MAXIMUM_VELOCITY = 5.0
CURVATURE_VELOCITY_LIMIT_FACTOR = 0.44
STEERING_VELOCITY_LIMIT_FACTOR = 2.0
DISTANCE_VELOCITY_LIMIT_FACTOR = 0.3
MINIMUM_THROTTLE_LIMIT = 0.0

LOOK_AHEAD_DISTANCE = 0.7
CURVATURE_LOOK_AHEAD_DISTANCE = 1.0  # Stored but unused, as in the WIP C++.
LOG_VELOCITY_DIAGNOSTICS = False

# Map loading has a replaceable interface, with no assumed real-world file format.
MAP_SOURCE: Path | None = None
ROUTE_NAME = "example"
DUMMY_LANE_WIDTH = 0.8

# Simulator-only parameters. Pose refers to the rear axle.
TIME_INTERVAL = 0.02
MAXIMUM_TIME = 60.0
INITIAL_POSITION = np.array([0.0, -0.3])
INITIAL_HEADING = 0.0
INITIAL_VELOCITY = 0.0
ACCELERATION_GAIN = 1.0
LINEAR_DRAG = 0.0
VEHICLE_MAXIMUM_THROTTLE = None
BRAKE_DECELERATION = 3.0
HAND_BRAKE_DECELERATION = 6.0
STOPPED_VELOCITY_TOLERANCE = 0.001

LIVE_VISUALIZATION = True
SHOW_DIAGNOSTICS = True
FRAMES_PER_SECOND = 30.0
PLAYBACK_SPEED = 1.0


def build_simulator(map_loader: MapLoader | None = None) -> Simulator:
    controller = PurePursuit(
        parameters=ControllerParameters(
            map_frame_id=MAP_FRAME_ID, robot_frame_id=ROBOT_FRAME_ID,
            target_frame_id=TARGET_FRAME_ID, wheelbase=WHEELBASE,
            position_tolerance=POSITION_TOLERANCE, steering_angle_limit=STEERING_ANGLE_LIMIT,
            desired_velocity=DESIRED_VELOCITY, maximum_velocity=MAXIMUM_VELOCITY,
            pid_p=PID_P, pid_i=PID_I, pid_d=PID_D,
            integral_decay_factor=INTEGRAL_DECAY_FACTOR,
            throttle_limit=THROTTLE_LIMIT, apply_throttle_limit=APPLY_THROTTLE_LIMIT,
            curvature_velocity_limit_factor=CURVATURE_VELOCITY_LIMIT_FACTOR,
            steering_velocity_limit_factor=STEERING_VELOCITY_LIMIT_FACTOR,
            distance_velocity_limit_factor=DISTANCE_VELOCITY_LIMIT_FACTOR,
            minimum_throttle_limit=MINIMUM_THROTTLE_LIMIT),
        pursuit_parameters=PurePursuitParameters(
            look_ahead_distance=LOOK_AHEAD_DISTANCE,
            curvature_look_ahead_distance=CURVATURE_LOOK_AHEAD_DISTANCE,
            log_velocity_diagnostics=LOG_VELOCITY_DIAGNOSTICS))
    vehicle = BicycleVehicle(
        VehicleState(INITIAL_POSITION.copy(), INITIAL_HEADING, INITIAL_VELOCITY),
        wheelbase=WHEELBASE, acceleration_gain=ACCELERATION_GAIN, linear_drag=LINEAR_DRAG,
        maximum_steering_angle=STEERING_ANGLE_LIMIT, maximum_throttle=VEHICLE_MAXIMUM_THROTTLE,
        brake_deceleration=BRAKE_DECELERATION, hand_brake_deceleration=HAND_BRAKE_DECELERATION)
    simulator = Simulator(
        controller, vehicle, dt=TIME_INTERVAL, maximum_time=MAXIMUM_TIME,
        stopped_velocity_tolerance=STOPPED_VELOCITY_TOLERANCE)
    # Loading the map and publishing a route are deliberately two operations.
    selected_loader = map_loader or DummyMapLoader(MAP_FRAME_ID, DUMMY_LANE_WIDTH)
    simulator.load_map(selected_loader, MAP_SOURCE)
    simulator.publish_route(ROUTE_NAME)
    return simulator


def main() -> None:
    argument_parser = argparse.ArgumentParser(description=__doc__)
    argument_parser.add_argument("--headless", action="store_true", help="Run without windows.")
    argument_parser.add_argument("--save-data", type=Path, help="Save recorded arrays to NPZ.")
    arguments = argument_parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")
    simulator = build_simulator()
    if arguments.headless:
        simulator.run()
    else:
        import matplotlib.pyplot as plt
        from car_simulator.visualization import SimulationVisualization
        visualization = SimulationVisualization(simulator, FRAMES_PER_SECOND, PLAYBACK_SPEED)
        if LIVE_VISUALIZATION:
            visualization.show_live()
        else:
            simulator.run()
            visualization.draw_state()
        if SHOW_DIAGNOSTICS:
            visualization.plot_diagnostics()
        plt.show()

    distance_to_goal = (np.linalg.norm(
        simulator.controller.path[-1].position - simulator.vehicle.state.position)
        if simulator.controller.path else float("nan"))
    print(f"Status: {simulator.stop_reason or 'window_closed'}; "
          f"time: {simulator.current_time:.2f} s; "
          f"distance to goal: {distance_to_goal:.3f} m; "
          f"velocity: {simulator.vehicle.state.velocity:.3f} m/s; "
          f"goal messages: {len(simulator.goal_messages)}.")
    if simulator.controller.last_error:
        print(f"Controller error: {simulator.controller.last_error}")
    if arguments.save_data is not None:
        np.savez(arguments.save_data, **simulator.get_data(),
                 stop_reason=simulator.stop_reason or "window_closed")
        print(f"Saved simulation data to {arguments.save_data}.")


if __name__ == "__main__":
    main()
