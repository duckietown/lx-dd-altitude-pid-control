"""
1-D altitude simulation for the Duckiedrone DD24.

The DD24 uses PX4 in OFFBOARD mode.  Unlike the DD21 (which accepted raw PWM
throttle in [1100, 1900]), the PID here outputs a **normalized thrust** in
[0.0, 1.0] that is sent to the ``setpoint_attitude/thrust`` MAVROS2 topic.

Physics
-------
At hover thrust K the net vertical force is zero:

    F_max = m * g / K          (maximum thrust force, in Newtons)
    F_net = thrust * F_max - m * g - drag_coeff * v
    a     = F_net / m
    v    += a * dt
    z    += v * dt  +  Gaussian(0, sensor_noise)

The drone starts at z = 0 m with v = 0 m/s.
"""

from __future__ import annotations
import collections
import random
import numpy as np
import matplotlib
import matplotlib.pyplot as plt


class VerticalDrone:
    """Simulated 1-D (vertical) drone for PID altitude tuning exercises.

    Parameters
    ----------
    pid :
        An object with ``step(err, dt) -> float`` and ``reset()`` methods.
        ``step`` must return a normalized thrust in [0.0, 1.0].
    step_size : int
        Number of simulation sub-steps per control step.  Larger values make
        the simulation finer-grained.
    drag_coeff : float
        Linear drag coefficient (N·s/m). Typical realistic value: 0.02.
    latency : int
        Number of control steps of measurement delay (simulates sensor/comm
        lag). 0 = no latency.
    sensor_noise : float
        Standard deviation of Gaussian noise added to altitude readings (m).
        0 = perfect sensor.
    mass : float
        Drone mass in kilograms (default: 0.7 kg for DD24).
    """

    GRAVITY = 9.81       # m/s²
    DT = 0.05            # simulation timestep (seconds) → 20 Hz
    HOVER_THRUST = 0.44  # normalized thrust at which a DD24-B hovers

    def __init__(
        self,
        pid,
        step_size: int = 10,
        drag_coeff: float = 0.0,
        latency: int = 0,
        sensor_noise: float = 0.0,
        mass: float = 0.7,
    ):
        self.pid = pid
        self.step_size = step_size
        self.drag_coeff = drag_coeff
        self.latency = latency
        self.sensor_noise = sensor_noise
        self.mass = mass

        # Maximum thrust the motors can produce, in newtons.  This is a
        # property of the airframe, so it must NOT depend on the gains the
        # student picks: deriving it from K would make the drone hover at
        # every K and Exercise 1 would have no answer.
        self.f_max = (mass * self.GRAVITY) / self.HOVER_THRUST

        # State
        self.z = 0.0          # altitude (m)
        self.v = 0.0          # vertical velocity (m/s)
        self.setpoint = 0.0   # desired altitude (m)

        # History (recorded after simulation)
        self.altitude_history: list[float] = []
        self.thrust_commands: list[float] = []   # normalized [0,1]
        self.time_history: list[float] = []

        # Latency buffer: stores past altitude readings
        self._latency_buf: collections.deque[float] = collections.deque(
            [0.0] * (latency + 1), maxlen=latency + 1
        )

    # Public API

    def update_setpoint(self, height: float) -> None:
        """Set the desired altitude in metres."""
        self.setpoint = height

    def simulate(self, end_time: float = 15.0) -> None:
        """Run the simulation for *end_time* seconds.

        Records altitude and thrust at each control step.
        """
        self.z = 0.0
        self.v = 0.0
        self.altitude_history = []
        self.thrust_commands = []
        self.time_history = []
        self._latency_buf = collections.deque(
            [0.0] * (self.latency + 1), maxlen=self.latency + 1
        )
        self.pid.reset()

        t = 0.0
        prev_t = 0.0

        while t <= end_time:
            # Measured altitude (with optional noise and latency)
            noisy_z = self.z + random.gauss(0, self.sensor_noise)
            self._latency_buf.append(noisy_z)
            measured_z = self._latency_buf[0]  # oldest reading

            dt = t - prev_t
            err = self.setpoint - measured_z
            thrust = self.pid.step(err, dt)

            # Physics integration
            for _ in range(self.step_size):
                sub_dt = self.DT / self.step_size
                f_net = thrust * self.f_max - self.mass * self.GRAVITY - self.drag_coeff * self.v
                a = f_net / self.mass
                self.v += a * sub_dt
                self.z += self.v * sub_dt
                self.z = max(self.z, 0.0)  # floor at ground

            self.altitude_history.append(self.z)
            self.thrust_commands.append(thrust)
            self.time_history.append(t)

            prev_t = t
            t += self.DT

    def plot_step_response(self) -> None:
        """Plot altitude and thrust command over time."""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

        ax1.plot(self.time_history, self.altitude_history, label='Altitude (m)')
        ax1.axhline(self.setpoint, color='orange', linestyle='--', label='Setpoint')
        ax1.set_ylabel('Altitude (m)')
        ax1.set_title('DD24 Altitude PID — Step Response')
        ax1.legend()
        ax1.grid(True)

        ax2.plot(self.time_history, self.thrust_commands, color='green', label='Thrust (normalized)')
        ax2.axhline(self.HOVER_THRUST, color='gray', linestyle=':', label='Hover thrust')
        ax2.set_ylim(-0.05, 1.05)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Thrust (normalized)')
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        plt.show()

    # Alias for backward compatibility with test code
    @property
    def pwm_commands(self):
        """Return thrust commands (alias used by test infrastructure)."""
        return self.thrust_commands
