import sys
import matplotlib.pyplot as plt

EXERCISE_DIRECTORY = "/code/src/lx-dd-altitude-pid-control/packages/solution/solution"
GAINS_PATH = EXERCISE_DIRECTORY + "/z_pid.yaml"

sys.path.append(EXERCISE_DIRECTORY)

from utils.writer import load_gains
from drone_sim import VerticalDrone


class TestPIDOutput:
    def __init__(self, gains_path: str, PID, setpoint: float = 1.0) -> None:
        self.z_0 = 0
        self.setpoint = setpoint
        self.gains_path = gains_path
        self.thrust_upper_limit = 1.0
        self.thrust_lower_limit = 0.0
        self.PID = PID

    def simulate(self, latency=0, noise=0):
        pid_gains = load_gains(self.gains_path)

        my_pid_instance = self.PID(
            kp=pid_gains['Kp'],
            kd=pid_gains['Kd'],
            ki=pid_gains['Ki'],
            k=pid_gains['K'],
        )

        sim = VerticalDrone(
            pid=my_pid_instance,
            step_size=10,
            latency=latency,
            sensor_noise=noise,
        )

        sim.update_setpoint(height=self.setpoint)
        sim.simulate()
        return sim.thrust_commands

    def simulate_with_limits(self):
        thrust_commands = self.simulate()
        fig, ax = plt.subplots()
        ax.plot(thrust_commands)

        border = 0.1
        ax.set_ylim([self.thrust_lower_limit - border, self.thrust_upper_limit + border])
        ax.set_xlabel('Simulation step')
        ax.set_ylabel('Normalized Thrust Output [0, 1]')
        ax.set_title(f'Thrust Control Output, setpoint {self.setpoint} m')

        plt.axhline(y=self.thrust_lower_limit, xmin=-100, xmax=100, color='r', linestyle='-')
        plt.axhline(y=self.thrust_upper_limit, xmin=-100, xmax=100, color='r', linestyle='-')
        fig.show()

        violations = [t for t in thrust_commands if t < self.thrust_lower_limit or t > self.thrust_upper_limit]
        if violations:
            print(f"FAIL: {len(violations)} thrust value(s) outside [0.0, 1.0]: {violations[:5]}")
        else:
            print(f"PASS: all {len(thrust_commands)} thrust values are within [0.0, 1.0]")


if __name__ == "__main__":
    from pid_class import PID
    test = TestPIDOutput(GAINS_PATH, PID)
    test.simulate_with_limits()
