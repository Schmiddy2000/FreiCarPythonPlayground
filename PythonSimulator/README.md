# Car simulator — C++ controller structure

A planar Python testbed for the supplied FreiCAR controller. The controller
receives path and odometry messages, then publishes control and goal messages.
The simulator supplies the missing odometry, communication, and vehicle backend.
No ROS or CARLA installation is needed. Requires Python 3.10+.

## Run

Use your existing environment with NumPy and Matplotlib:

```sh
conda activate freicar
python launch_simulation.py
```

Or install dependencies into an environment of your choice with
`python -m pip install -r requirements.txt`.

For a fast run without any windows (NumPy alone is sufficient):

```sh
python launch_simulation.py --headless
python launch_simulation.py --headless --save-data experiment.npz
```

All controller, map, physics, and display settings are in
`launch_simulation.py`. The controller defaults now match the supplied C++:
desired velocity 0.5 m/s; PID (1.25, 0.01, 0.01); wheelbase 0.36 m;
position tolerance 0.1 m; look-ahead distance 0.7 m. Integral decay defaults
to zero for C++ parity. It remains available as your Python experiment.

The live window advances the simulator directly. Closing it ends that run at
the current time and opens diagnostics. Set `LIVE_VISUALIZATION = False` to
compute the run first and display the final trajectory. Diagnostics show
velocity, throttle, brake/hand brake, steering, and integral.

## Runtime flow

1. The launcher constructs `PurePursuit`, `BicycleVehicle`, and `Simulator`.
2. `simulator.load_map(loader, source)` loads geometry and available routes.
3. `simulator.publish_route(name)` publishes a path to `Controller.receive_path()`.
4. Each simulator tick publishes stamped odometry. This invokes the subclass's
   `PurePursuit.make_step(odometry)`.
5. `make_step()` runs PID and steering, then publishes a `ControlCommand`.
   `Simulator.receive_control_command()` receives it.
6. The vehicle adapter applies the command and advances position, heading, and
   velocity. The next tick publishes the resulting odometry.
7. At the goal, the controller publishes its goal message once and commands
   throttle 0, brake 1, hand brake true, and steering 0. The simulator keeps
   applying that command until the model has stopped.

There is no second path-following or goal algorithm inside the simulator.
The goal notification is the controller's output, while stopping is the
vehicle backend's response to its brake command.

## Components

| File | Responsibility |
| --- | --- |
| `launch_simulation.py` | All experiment parameters and component wiring |
| `car_simulator/controller.py` | Base node concepts: shared parameters, path reception, transforms, publishers, PID |
| `car_simulator/pure_pursuit_controller.py` | `PurePursuit(Controller)` and the supplied WIP algorithm |
| `car_simulator/pid_controller.py` | `step(error, stamp)`, full reset, and integral-only reset |
| `car_simulator/messages.py` | Local equivalents of path, odometry, control, goal, and target messages |
| `car_simulator/transport.py` | Synchronous publishers/subscribers; latched path delivery |
| `car_simulator/transforms.py` | Planar pose transforms and frame lookup |
| `car_simulator/maps.py` | `MapLoader`, `DummyMapLoader`, and `SimulationMap` |
| `car_simulator/vehicle.py` | Rear-axle bicycle physics and command/odometry adapter |
| `car_simulator/simulator.py` | Clock, message delivery, recording, and termination |
| `car_simulator/visualization.py` | Map display, live car movement, and diagnostics |
| `car_simulator/paths.py` | Generated example route used by the dummy map |

All module files in the table except the launcher are under `car_simulator/`.

## C++ to Python mapping

Python uses snake_case and longer names, but methods retain their C++ roles.

| C++ | Python |
| --- | --- |
| `Controller::receivePath(new_path)` | `Controller.receive_path(new_path)` |
| `Controller::transformPath(path, target_frame)` | `Controller.transform_path(path, target_frame)` |
| `Controller::sendGoalMsg(reached)` | `Controller.send_goal_msg(reached)` |
| `PurePursuit::makeStep(odom)` | `PurePursuit.make_step(odometry)` |
| `getIntersectionTupleIndices(center, radius)` | `get_intersection_tuple_indices(center, radius)` |
| `getDistance(first, second)` | `get_distance(first, second)` |
| `getEta(x_0, x_1, radius)` | `get_eta(first_endpoint, second_endpoint, radius)` |
| `reachedGoal(car_position)` | `reached_goal(car_position)` |
| `logVelocityDiagnostics(odom)` | `log_velocity_diagnostics(odometry)` |
| `path_` | `path`: list of planar poses; `path_positions` exposes an (N, 2) array |
| `idx_` | `path_index` |
| `goal_reached_` / `completion_advertised_` | `goal_reached` / `completion_advertised` |
| `L_` / `pos_tol_` / `steering_angle_max_` | `wheelbase` / `position_tolerance` / `steering_angle_maximum` |
| `des_v_` / `vel_pid` | `desired_velocity` / `velocity_pid` |
| `ld_dist_` / `ld_curvature` | `look_ahead_distance` / `curvature_look_ahead_distance` |
| `cmd_control_` | `control_command` |
| `pub_acker_` | `control_publisher` |
| `pub_goal_reached_` | `goal_reached_publisher` |
| `tf_buffer_` | `transform_buffer` |
| `target_p_` and target broadcaster | `target_transform` and `target_publisher` |
| `vmax_` | `parameters.maximum_velocity` |
| `throttle_limit_` / `minimum_throttle_limit_` | `parameters.throttle_limit` / `parameters.minimum_throttle_limit` |
| Curvature, steering, distance velocity factors | Corresponding `parameters.*_velocity_limit_factor` fields |

The base class's `make_step()` warns, like the C++ base. To swap controllers,
subclass `Controller` and implement `make_step(odometry)`; call
`publish_control_command()` to send results. Shared path and publisher
infrastructure stays in the base. No steering helper interface is imposed.

## Behavior retained from the supplied version

- The PID uses odometry timestamps, including the proportional-only first
  sample and zero output for duplicate/backward stamps. Decay, when enabled,
  occurs before the trapezoidal integral increment.
- PID is evaluated before the path-following logic.
- Path reception transforms into the map frame and resets path index, goal,
  and completion flags. It does not replace desired velocity or reset PID.
  A failed transform preserves the previous path and flags.
- An empty path clears the path, sets goal_reached true and
  completion_advertised true. It does not advertise successful completion.
- The index search retains strict inside/outside comparisons and mutable
  `idx_` behavior. If no pair is found, it returns the final two indices.
- `get_eta()` uses the supplied plus root and does not constrain the
  intersection to the finite segment. The final-pair fallback may therefore
  target an extension of the final segment. This known WIP behavior is retained.
- Steering is `atan(2 * wheelbase * sin(eta) / look_ahead_distance)`.
- The published steering field is **degrees**, matching the raiscar command.
  Pose headings, transforms, and vehicle physics use radians.
- The desired velocity remains constant: the earlier Python goal-approach
  speed scheduling and nearest-forward projection were removed.
- The commented-out C++ throttle clamp is disabled by default. Set
  `APPLY_THROTTLE_LIMIT = True` to enable the same [0, throttle_limit] clamp.
- The larger curvature look-ahead, maximum velocity, and velocity-limit
  factors are retained as settings but remain inactive in this WIP version.
- Goal state is sticky until a new nonempty route. Goal messages are published
  once per route; steering and throttle are zeroed and brake/hand brake applied.

`reset_integral()` clears only the integral; `reset()` clears all PID history.
The C++ body of `resetIntegral()` was not provided, so its integral-only meaning
is an explicit assumption to check against your current code.

## Explicit differences and simulation boundaries

The structural/numerical port is not a running ROS node or a bit-for-bit
floating-point reproduction of its float casts.

- Empty/one-point paths publish a stop instead of returning while an old
  throttle command stays active.
- Goal checking occurs before the intersection calculation. This avoids an
  invalid geometry calculation preventing an already-reached goal from braking.
- Coincident endpoints or a non-intersecting fallback line cause an explicit
  controller error and braking, instead of division by zero or NaNs.
  Tiny negative discriminants within 1e-12 are treated as roundoff.
- The look-ahead target is broadcast for plotting; this does not alter control.
- Odometry must be in the configured map frame. As in the C++, its pose is
  interpreted as map_from_rear. The simulator reports body-frame longitudinal
  velocity, not world-frame x velocity.
- Planar transforms retain yaw and x/y positions; they do not implement
  roll, pitch, z, quaternions, time-buffered lookup, or a full ROS TF tree.
  The tracker, rear-axis, and base_link origins coincide at the rear axle in
  this model; the front-axis origin is one wheelbase ahead.
- Publishers are synchronous in-process callbacks. The path publisher replays
  its latest value to a late subscriber, modeling the relevant transient-local
  behavior. ROS scheduling, QoS timing, and transport delays are not simulated.
- The vehicle applies its mechanical steering limit, converts degrees to
  radians, and interprets acceleration as
  `acceleration_gain * throttle - linear_drag * velocity - braking_acceleration`.
  Brake/hand brake use configurable deceleration; negative throttle can also
  decelerate in this toy backend. Reverse, actuator lag, tire slip, and CARLA's
  actual throttle/brake adapter are not reconstructed.
- Acceleration and steering are held during each interval. Integrate speed
  and distance, then follow the bicycle's circular arc for that distance.
  Braking to zero partway through an interval does not create reverse motion.
- Goal notification marks entry into the position tolerance. The simulator
  then continues braking and terminates once speed is below the configured
  stopped tolerance. It never snaps the pose to the goal. The time limit can
  end a run before either goal completion or full braking.

## Map loading

`MapLoader.load(source)` is the format boundary. Implement it when your map
format is settled and pass the loader to `build_simulator(map_loader=...)`.
Set `MAP_SOURCE` in the launcher to your future source path. DummyMapLoader
generates a map without reading a file; passing it a source path raises an
explanatory error.

A `SimulationMap` contains a frame name, display boundary polylines, and named
`PathMessage` routes. Geometry must already be in the configured map frame.
Routes can have another frame if the controller's transform buffer has the
appropriate transform. The dummy map supplies a gentle S-shaped route plus two
road-edge polylines. Boundaries are for display, not collision or lane physics.

Loading a map does not send a path. Call `publish_route(name)` separately, or
call `publish_path(PathMessage.from_positions(your_points, frame_id))` for an
externally supplied route. The controller does not read map files.

## Recording and repeatability

`simulator.get_data()` returns arrays:
- `time`, `position`, `heading`, `velocity`: N+1 states for N simulated intervals.
- `control_time`: N observation/control times, at each interval's start.
- `throttle`, `requested_throttle`, `steering_angle` (radians),
  `requested_steering_angle` (radians), `steering_degrees`, `brake`,
  `hand_brake`, `desired_velocity`, `integral`, `path_index`: N control entries.
- `target_position` and `target_valid`: N visualization targets/validity flags.
  Before any target is broadcast, position is a placeholder and validity false.
- `goal_time` and `goal_reached`: one entry per goal notification, on their own
  event time grid.

The configured desired_velocity stays unchanged even during final braking,
matching the C++ state. The zero-throttle/brake command overrides driving.
NPZ exports include these arrays plus stop_reason. Full odometry and goal
message values are also available as simulator.odometry_messages and
simulator.goal_messages.

`simulator.reset()` clears runtime histories and PID state, restores the initial
vehicle pose, and republishes the active route. Receiving a route during a run
only changes the C++ path lifecycle; it does not reset the clock or vehicle.

## Validation

```sh
python -m unittest discover -s tests -v
```

Tests cover the supplied PID recurrence, timestamp edge cases, path lifecycle
and transforms, one-time goal messages, index/fallback and get_eta behavior,
body-frame velocity, degree/radian boundaries, optional throttle limiting,
latched path delivery, map-loader substitution, bicycle motion/braking, time
grids, controller substitution, and repeatable completion of the example.
These tests do not compile or execute the ROS/CARLA project.
