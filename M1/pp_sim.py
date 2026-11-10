"""Pure pursuit simulation based on pure_pursuit_revised.ipynb.

Requires numpy, matplotlib, and the notebook's original data.py.
Position and velocity refer to the rear axle.
"""

import numpy as np

# Configuration: metres, seconds, radians.
velocity = 1
dt = 0.01

wheelbase = 0.36
look_ahead_distance = 2

initial_position = np.array([1, 1])
initial_heading = 0.0

goal_tolerance = 0.02
max_time = 120.0


def find_target(path, position):
    """Find the notebook's first inside-to-outside waypoint pair."""
    distances = np.linalg.norm(path - position, axis=1)

    for i in range(1, len(path)):
        if distances[i - 1] <= look_ahead_distance < distances[i]:
            # Same quadratic as get_eta(), using the '+' root.
            # For an inside-to-outside pair, this root lies on the segment.
            a = path[i - 1]
            d = path[i] - a
            r = a - position

            dd = np.dot(d, d)
            discriminant = np.dot(r, d)**2 - dd * (
                np.dot(r, r) - look_ahead_distance**2
            )

            fraction = (
                -np.dot(r, d) + np.sqrt(max(0.0, discriminant))
            ) / dd

            return a + fraction * d, False

    # Near the end, the endpoint may be inside the look-ahead circle
    # without any later point outside it.
    if distances[-1] <= look_ahead_distance:
        return path[-1].copy(), True

    raise ValueError(
        "No inside-to-outside waypoint pair found. As in the notebook, "
        "at least one path point must be inside the look-ahead circle. "
        "Check the initial pose, path spacing, or look_ahead_distance."
    )


def pursuit_control(position, heading, target):
    # Equivalent to:
    # transform_points(invert_pose(make_pose(x, y, heading)), [target])
    c, s = np.cos(heading), np.sin(heading)
    world_to_robot_rotation = np.array([[c, s], [-s, c]])
    target_robot = world_to_robot_rotation @ (target - position)

    distance_squared = np.dot(target_robot, target_robot)
    eta = np.arctan2(target_robot[1], target_robot[0])

    # On the look-ahead circle:
    # curvature = 2*sin(eta)/look_ahead_distance
    #
    # Using the actual target distance also handles the final endpoint
    # when it is closer than look_ahead_distance.
    curvature = 2 * target_robot[1] / distance_squared
    steering = np.arctan(wheelbase * curvature)

    return eta, curvature, steering


def advance(position, heading, distance, curvature):
    """Exact bicycle motion with constant steering during one step."""
    # curvature = tan(steering)/wheelbase
    # Positive curvature means a left turn.
    turn = curvature * distance
    middle_heading = heading + turn / 2

    # np.sinc(x) = sin(pi*x)/(pi*x).
    # This expression also handles zero curvature without division by zero.
    displacement = distance * np.sinc(turn / (2 * np.pi))

    position = position + displacement * np.array([
        np.cos(middle_heading),
        np.sin(middle_heading),
    ])

    heading = np.arctan2(
        np.sin(heading + turn),
        np.cos(heading + turn),
    )

    return position, heading


def simulate(path):
    path = np.asarray(path, dtype=float)

    if (
        path.ndim != 2
        or path.shape[1] != 2
        or len(path) < 2
        or not np.isfinite(path).all()
    ):
        raise ValueError(
            "path must contain at least two finite [x, y] points."
        )

    parameters = [
        velocity, dt, wheelbase, look_ahead_distance,
        goal_tolerance, max_time,
    ]
    if not np.isfinite(parameters).all() or min(parameters) <= 0:
        raise ValueError(
            "All simulation parameters must be finite and positive."
        )

    position = np.asarray(initial_position, dtype=float).copy()
    heading = float(initial_heading)

    if (
        position.shape != (2,)
        or not np.isfinite(position).all()
        or not np.isfinite(heading)
    ):
        raise ValueError("The initial pose must be finite; position shape (2,).")

    time = 0.0
    history = {
        key: [] for key in (
            "time", "position", "heading", "target",
            "steering", "curvature", "radius",
        )
    }

    while True:
        reached = (
            np.linalg.norm(path[-1] - position) <= goal_tolerance
        )

        if reached:
            target = path[-1].copy()
            final_target = True
            eta = curvature = steering = 0.0
        else:
            target, final_target = find_target(path, position)
            eta, curvature, steering = pursuit_control(
                position, heading, target
            )

        values = (
            time,
            position.copy(),
            heading,
            target.copy(),
            steering,
            curvature,
            np.inf if curvature == 0 else 1 / curvature,
        )

        for key, value in zip(history, values):
            history[key].append(value)

        if reached:
            break

        if time >= max_time:
            raise RuntimeError(
                "Goal not reached within max_time; check path and settings."
            )

        distance = velocity * min(dt, max_time - time)

        if final_target:
            # The endpoint lies on the commanded turning circle.
            # Shorten the last step if necessary to avoid overshooting it.
            if abs(curvature) > 1e-12:
                distance_to_goal = 2 * eta / curvature
            elif abs(eta) < np.pi / 2:
                distance_to_goal = np.linalg.norm(target - position)
            else:
                raise ValueError("Final target is directly behind the car.")

            distance = min(distance, distance_to_goal)

        position, heading = advance(
            position, heading, distance, curvature
        )
        time += distance / velocity

    return {
        key: np.asarray(value)
        for key, value in history.items()
    }


def animate(path, history):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import FancyArrowPatch, Polygon

    fig, ax = plt.subplots(figsize=(11, 6))

    ax.plot(path[:, 0], path[:, 1], "g.-", label="Path")
    ax.plot(*path[-1], "k*", markersize=12, label="Goal")

    trail, = ax.plot(
        [], [], color="0.5", lw=1, label="Rear-axle trajectory"
    )
    wheelbase_line, = ax.plot(
        [], [], "r-", lw=2, label="Wheelbase"
    )

    car = Polygon(
        np.zeros((4, 2)),
        closed=True,
        facecolor="red",
        edgecolor="red",
        alpha=0.25,
    )
    ax.add_patch(car)

    arrow = FancyArrowPatch(
        (0, 0), (0, 0),
        arrowstyle="->",
        mutation_scale=15,
        color="blue",
        lw=2,
        shrinkA=0,
        shrinkB=0,
        label="Pursuit target",
    )
    ax.add_patch(arrow)

    target_dot, = ax.plot([], [], "bo", markersize=4)
    status = ax.text(
        0.02, 0.98, "", transform=ax.transAxes, va="top"
    )

    extent = np.vstack([path, history["position"]])
    margin = max(look_ahead_distance, 2 * wheelbase)

    ax.set_xlim(
        extent[:, 0].min() - margin,
        extent[:, 0].max() + margin,
    )
    ax.set_ylim(
        extent[:, 1].min() - margin,
        extent[:, 1].max() + margin,
    )
    ax.set_aspect("equal", adjustable="box")
    ax.set(
        xlabel="World x [m]",
        ylabel="World y [m]",
        title="Pure pursuit",
    )
    ax.grid(alpha=0.25)
    ax.legend(loc="lower right")

    # Rectangle from rear axle (x=0) to front axle (x=wheelbase).
    half_width = 0.3 * wheelbase
    body = np.array([
        [0, -half_width],
        [wheelbase, -half_width],
        [wheelbase, half_width],
        [0, half_width],
    ])

    def update(i):
        position = history["position"][i]
        heading = history["heading"][i]
        target = history["target"][i]

        c, s = np.cos(heading), np.sin(heading)
        rotation = np.array([[c, -s], [s, c]])

        car.set_xy(body @ rotation.T + position)

        front = position + wheelbase * np.array([c, s])
        wheelbase_line.set_data(
            [position[0], front[0]],
            [position[1], front[1]],
        )

        trail.set_data(
            history["position"][:i + 1, 0],
            history["position"][:i + 1, 1],
        )

        arrow.set_positions(position, target)
        arrow.set_visible(i < len(history["time"]) - 1)
        target_dot.set_data([target[0]], [target[1]])

        status.set_text(
            f't = {history["time"][i]:.2f} s\n'
            f'steering = '
            f'{np.degrees(history["steering"][i]):.1f} degrees'
        )

        return car, wheelbase_line, trail, arrow, target_dot, status

    animation = FuncAnimation(
        fig,
        update,
        frames=len(history["time"]),
        interval=1000 * dt,
        blit=False,
        repeat=False,
    )
    fig.tight_layout()
    return fig, animation


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from data.paths import path_1

    path = np.asarray(path_1, dtype=float)
    history = simulate(path)

    print(
        f'Stored {len(history["time"])} states; '
        f'duration {history["time"][-1]:.2f} s.'
    )

    # Optional: save all recorded arrays.
    # np.savez("pure_pursuit_history.npz", **history)

    # Keep this reference alive while the animation window is open.
    fig, animation = animate(path, history)
    plt.show()