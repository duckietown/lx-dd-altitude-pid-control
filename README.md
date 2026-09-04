# Duckiedrone PID Altitude Tuning — Learning Experience

`Software: ente`; `Hardware: DD24-B`

This Learning Experience teaches you how to design, implement, and tune a **PID altitude controller** for the **Duckiedrone DD24**, which runs **PX4** as its flight controller and communicates with ROS2 via **MAVROS2**.

## What You Will Learn

- Discrete-time PID control theory (proportional, integral, derivative terms)
- How PX4's OFFBOARD mode works and why the heartbeat rate matters
- How the `setpoint_attitude` MAVROS2 plugin lets a companion computer command normalized thrust and attitude
- How to tune PID gains systematically using the Ziegler–Nichols method
- How to transfer a simulation-tuned controller to real hardware

## Notebooks

Work through the notebooks **in order**:

| # | Notebook | Description |
|---|----------|-------------|
| 1 | `1-general_overview.ipynb` | PID control theory and tuning concepts |
| 2 | `2-drone_pids_overview.ipynb` | DD24 / PX4 / MAVROS2 architecture |
| 3 | `3-altitude_pid_activity.ipynb` | Implement and tune PID in simulation |
| 4 | `4-hands-on-altitude-tuning.ipynb` | Deploy and tune on real DD24 hardware |

## What You Implement

In `packages/solution/solution/pid_class.py` you implement a `PID` class with:

- `step(err, dt) → float` — returns a **normalized thrust command** in [0.0, 1.0]
- `reset()` — resets integral and derivative state

The `K` parameter is the hover thrust offset (typically ~0.44 for the physical Duckiedrone).

## Hardware Requirements (Notebook 4)

- Duckiedrone DD24 with PX4 firmware
- MAVROS2 running on the companion computer
- Bottom rangefinder (defaults to `/<robot>/bottom_tof_driver_node/range`; override via the `range_topic` ROS2 parameter)
- RC transmitter for safety override