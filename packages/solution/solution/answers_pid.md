# DD24 PID Altitude Controller: Answers

## Notebook 2: Flight Control Architecture

### Altitude Control

**1. Controlled output and control variable:**

**2. How the process variable is measured:**

**3. What is K? What happens if K is too high (0.9) or too low (0.1)?**

**4. Why is the output clipped to [0.0, 1.0]? What physical constraint is that?**

**5. A hovering Duckiedrone is nudged sideways. What happens?**

### OFFBOARD Mode

**6. Why must setpoints be published at > 2 Hz? What happens if the stream stops, and why?**

**7. Role of the `/enable_altitude_control` safety gate, and why the PID is not enabled at startup:**

---

## Notebook 3: Simulation

### Activity 1: the PID controller

- Result of `check_range()`:

### Activity 2: the effect of each term

- Why K alone does not hold the model at an altitude:
- Why the P term alone oscillates, and what the D term changes:
- Steady-state error left by a wrong K, and how the I term removes it:

### Activity 3: the reset

- What the second flight does when `reset()` is missing, and why:

### Activity 4: delay and noise

- Effect of a measurement delay on the gains:
- Why noise reaches the thrust command mostly through the D term:
- Gains settled on with delay and noise (Kp, Ki, Kd, K):

### Thinking activities

- Is this tuning expected to work on the virtual Duckiedrone? Why or why not?
- Is this tuning expected to work on the physical Duckiedrone? Why or why not?

---

## Notebook 4: Tuning on the Duckiedrone

*(Fill in after flying.)*

- Duckiedrone used (virtual or physical):
- Measured hover thrust offset K:

### Ziegler-Nichols

- Ultimate gain $K_u$:
- Ultimate period $T_u$ (s):
- Computed Kp = , Ki = , Kd =

### Final Gains

| Gain | Value |
|------|-------|
| K    |       |
| Kp   |       |
| Ki   |       |
| Kd   |       |

### Observations

*(Describe how each gain change affected flight behaviour.)*

- Behaviour at the 0.5 m setpoint:
