# DD24 PID Altitude Controller — Answers

## Notebook 2: PID on the DD24

### Altitude Control

**1. Process variable, error, and control variable:**

**2. What is K? What happens if K is too high or too low?**

**3. Why is the output clipped to [0.0, 1.0]?**

---

### OFFBOARD Mode

**4. Why must setpoints be published at > 2 Hz? What happens if the stream stops?**

**5. Role of the `/enable_altitude_control` safety gate:**

---

## Notebook 3: Simulation Tuning

### Problem 1 — Idealized PID

*(Fill in your observations and final gains here.)*

- K takeoff value:
- Kp = 0.5 observation:
- Kp = 2.0 observation:
- Kp = 5.0 observation:
- Kd = 0.5 observation:
- Kd = 2.0 observation:
- Kd = 5.0 observation:
- Final PD gains (Kp, Kd):
- Role of I term:
- reset() importance:
- Final PID gains: Kp = , Ki = , Kd = , K =

### Problem 2 — PID with Latency (6 steps)

*(Fill in your observations and final gains here.)*

- Final PID gains: Kp = , Ki = , Kd = , K =
- Comparison to Problem 1:
- Effect of latency on each term:

### Problem 3 — PID with Latency + Noise + Drag

*(Fill in your observations and final gains here.)*

- Final PID gains: Kp = , Ki = , Kd = , K =
- Comparison to Problems 1 and 2:

---

## Notebook 4: Hardware Tuning

*(Fill in after flying.)*

### Ziegler–Nichols

- Ultimate gain $K_u$:
- Ultimate period $T_u$ (s):
- Computed Kp = , Ki = , Kd =

### Final Hardware Gains

| Gain | Value |
|------|-------|
| K    |       |
| Kp   |       |
| Ki   |       |
| Kd   |       |

### Observations

*(Describe how each gain change affected flight behaviour.)*
