# Duckiedrone PID Altitude Tuning — Learning Experience

`Software: ente`; `Hardware: DD24-B`

This Learning Experience contains activities on how to design, implement, and tune a **PID controller** for altitude control of a **Duckiedrone** (model DD24-B), which runs **PX4** as its flight controller and communicates with ROS2 on a companion Raspberry Pi 4 via **MAVROS2**.

## General instructions

The general instructions on how to run this and other Duckietown learning experiences (LXs) is available on the Duckietown Manual: [](https://docs.duckietown.com/ente/opmanual-dd24/50-learning-experiences/supported-lxs/pid-altitude-control.html). 

## Intended Learning Outcomes

Through this learning experience, you will learn:

- Discrete-time PID (proportional, integral, derivative) control theory
- A time-honored PID coefficients tuning strategy: the Ziegler–Nichols method
- Implementation details such as how PX4's OFFBOARD mode works and why the heartbeat rate matters
- How the setpoint_attitude MAVROS2 plugin lets a companion computer command normalized thrust and attitude
- How to transfer a simulation-tuned controller to physical hardware

## Requirements

This LX runs both on virtual and on physical Duckiedrones. 

### Software Requirements

- A computer with a working [Duckietown Shell installation](https://docs.duckietown.com/ente/opmanual-dd24/10-duckiedrone-preliminaries/initial-setup.html#required-software-and-accounts).

### Hardware Requirements (Notebook 4)

- Duckietown [Duckiedrone](https://get.duckietown.com/products/autonomous-raspberrypi-quadcopter-duckiedrone-dd24?variant=43227749023919) with PX4 firmware (model DD24-B)

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
