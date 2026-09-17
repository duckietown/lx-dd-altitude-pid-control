"""
Vertical flight model of a Duckiedrone, used by Notebook 3.

The model moves only vertically. The controller commands a normalized
thrust u in [0.0, 1.0], and at u = HOVER_THRUST the thrust balances the weight:

    F_max = m * g / HOVER_THRUST
    m * a = u * F_max - m * g   ->   a = g * (u / HOVER_THRUST - 1)

The mass cancels, so the simulation does not need it. The floor is at z = 0 m.

The controller runs every DT seconds. Two options make the measured altitude
differ from the true one: `latency` delays it by a number of control steps,
and `sensor_noise` adds Gaussian noise with that standard deviation, in meters.
"""

from __future__ import annotations
import collections

import numpy as np
import matplotlib.pyplot as plt


class VerticalDrone:
    """Vertical flight model of a Duckiedrone.

    Parameters
    ----------
    pid :
        An object with ``step(err, dt) -> float`` and ``reset()`` methods.
    step_size : int
        Physics sub-steps per control step.
    latency : int
        Measurement delay, in control steps of DT seconds. 0 = no delay.
    sensor_noise : float
        Standard deviation of the altitude measurement noise, in meters.
    seed : int
        Seed of the noise generator, so the same inputs give the same run.
    """

    GRAVITY = 9.81       # m/s^2
    DT = 0.05            # control period (s), 20 Hz as in the pid_controller node
    HOVER_THRUST = 0.44  # normalized thrust at which the Duckiedrone hovers

    def __init__(
        self,
        pid,
        step_size: int = 10,
        latency: int = 0,
        sensor_noise: float = 0.0,
        seed: int = 0,
    ):
        self.pid = pid
        self.step_size = step_size
        self.latency = latency
        self.sensor_noise = sensor_noise
        self.seed = seed
        self.setpoint = 0.0

        self.time_history: list[float] = []
        self.altitude_history: list[float] = []
        self.measured_history: list[float] = []
        self.thrust_commands: list[float] = []

    def update_setpoint(self, height: float) -> None:
        """Set the desired altitude in meters."""
        self.setpoint = height

    def simulate(self, end_time: float = 15.0) -> None:
        """Start on the floor at rest and run for `end_time` seconds."""
        rng = np.random.default_rng(self.seed)
        z, v = 0.0, 0.0
        measurements = collections.deque([0.0] * (self.latency + 1), maxlen=self.latency + 1)
        self.time_history, self.altitude_history = [], []
        self.measured_history, self.thrust_commands = [], []
        self.pid.reset()

        n_steps = int(round(end_time / self.DT)) + 1
        sub_dt = self.DT / self.step_size
        for k in range(n_steps):
            t = k * self.DT
            measurements.append(z + rng.normal(0.0, self.sensor_noise))
            measured_z = measurements[0]

            dt = 0.0 if k == 0 else self.DT
            thrust = self.pid.step(self.setpoint - measured_z, dt)

            self.time_history.append(t)
            self.altitude_history.append(z)
            self.measured_history.append(measured_z)
            self.thrust_commands.append(thrust)

            a = self.GRAVITY * (thrust / self.HOVER_THRUST - 1.0)
            for _ in range(self.step_size):
                v += a * sub_dt
                z += v * sub_dt
                if z <= 0.0:
                    z, v = 0.0, max(v, 0.0)

    def plot_step_response(self) -> None:
        """Plot altitude and thrust command over time."""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

        ax1.axhline(self.setpoint, color='tab:orange', linestyle='--', label='setpoint')
        ax1.plot(self.time_history, self.altitude_history, color='tab:blue', label='true altitude')
        if self.latency or self.sensor_noise:
            ax1.plot(self.time_history, self.measured_history, '.', color='gray', markersize=3,
                     label='measured altitude')
        ax1.set_ylim(0.0, 2.5)
        ax1.set_ylabel('altitude [m]')
        ax1.legend(loc='upper right')
        ax1.grid(True)

        ax2.axhline(self.HOVER_THRUST, color='gray', linestyle=':', label='hover thrust')
        ax2.plot(self.time_history, self.thrust_commands, color='tab:green', label='thrust command')
        ax2.set_ylim(-0.05, 1.05)
        ax2.set_xlabel('time [s]')
        ax2.set_ylabel('normalized thrust')
        ax2.legend(loc='upper right')
        ax2.grid(True)

        plt.tight_layout()
        plt.show()
