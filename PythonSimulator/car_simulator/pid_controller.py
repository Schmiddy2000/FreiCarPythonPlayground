"""Timestamp-driven PID matching C++ step(error, stamp), with optional Python decay."""
import math


class PID:
    def __init__(self, pid_p: float, pid_i: float, pid_d: float,
                 decay_factor: float = 0.0):
        if not all(math.isfinite(value) for value in (pid_p, pid_i, pid_d, decay_factor)):
            raise ValueError("PID parameters must be finite.")
        if not 0 <= decay_factor <= 1:
            raise ValueError("decay_factor must lie in [0, 1].")
        self.pid_p = pid_p
        self.pid_i = pid_i
        self.pid_d = pid_d
        self.decay_factor = decay_factor
        self.reset()

    def reset(self) -> None:
        self.have_previous_sample = False
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = 0.0

    def reset_integral(self) -> None:
        # Counterpart of resetIntegral(): clear the accumulator, keep history.
        self.integral = 0.0

    def step(self, error: float, stamp: float) -> float:
        if not math.isfinite(error) or not math.isfinite(stamp):
            raise ValueError("The error and timestamp must be finite.")
        if not self.have_previous_sample:
            self.previous_time = stamp
            self.previous_error = error
            self.have_previous_sample = True
            return self.pid_p * error

        time_interval = stamp - self.previous_time
        if time_interval <= 0:
            # Preserve the supplied C++ behavior for duplicate/backward stamps.
            self.previous_time = stamp
            self.previous_error = error
            return 0.0

        self.decay_integral()
        self.integral += time_interval * (error + self.previous_error) / 2
        control = (self.pid_p * error + self.pid_i * self.integral
                   + self.pid_d * (error - self.previous_error) / time_interval)
        self.previous_time = stamp
        self.previous_error = error
        return control

    def decay_integral(self) -> None:
        # The original per-call experiment. Zero disables decay for C++ parity.
        self.integral = self.integral * (1 - self.decay_factor)
