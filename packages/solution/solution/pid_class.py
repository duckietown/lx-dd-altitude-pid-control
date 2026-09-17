import numpy as np


class PID:
    """
    Altitude PID controller for the Duckiedrone.

    `step` returns a normalized thrust in [0.0, 1.0], which the pid_controller
    node sends to PX4 through MAVROS2. Notebook 3 flies this class on a model,
    and Notebook 4 on a virtual or physical Duckiedrone.

    The node reads the gains and the state back by name, so keep them:

        gains: _p, _i, _d, _k
        state: err, e_int, e_der
    """

    def __init__(self, kp, ki, kd, k):
        """
        :param kp: Proportional gain K_p
        :param ki: Integral gain K_i
        :param kd: Derivative gain K_d
        :param k:  Hover thrust offset K, normalized in [0.0, 1.0]
        """
        self._p = kp
        self._i = ki
        self._d = kd
        self._k = k

        self.err = 0.0    # previous error e_{k-1}, in m
        self.e_int = 0.0  # discrete integral e_int,k, equation (6), in m*s
        self.e_der = 0.0  # discrete derivative e_der,k, equation (7), in m/s

    def step(self, err, dt):
        """
        Compute one controller step.

            (6)  e_int,k = e_int,k-1 + e_k * dt
            (7)  e_der,k = (e_k - e_{k-1}) / dt
            (8)  u_k     = K_p * e_k + K_i * e_int,k + K_d * e_der,k + K

        :param err: Tracking error e_k = setpoint - measured altitude, in m.
        :param dt:  Time since the previous call, in s. 0.0 on the first call.
        :returns:   Normalized thrust u_k, clipped to [0.0, 1.0].
        """
        # ------------------------------------------------------------------
        # TO IMPLEMENT: the discrete PID controller, equations (6) to (8) in
        # Notebook 3, Activity 1, "What step computes". Under each numbered
        # comment below, write the line shown, with every TO IMPLEMENT
        # replaced by your own expression, in order:
        #
        #   1. Integral: update self.e_int.
        #   2. Derivative: compute self.e_der, guarding the first call.
        #   3. Save err into self.err, after step 2 has used the old value.
        #   4. Combine the terms into u.
        #   5. Clip u to [0.0, 1.0].
        # ------------------------------------------------------------------

        # 1. Integral ("The sum" in Activity 1).
        #    self.e_int holds e_int,k-1, the total from the previous call.
        #    Update it with equation (6):
        #
        #        self.e_int += TO IMPLEMENT

        # 2. Derivative, equation (7) ("The difference" in Activity 1).
        #    self.err still holds the error from the previous call, e_{k-1}.
        #    On the first call dt is 0.0, and dividing by it would give inf or
        #    NaN, so the derivative must be 0.0 there. Use err, self.err and
        #    dt:
        #
        #        if TO IMPLEMENT:
        #            self.e_der = TO IMPLEMENT
        #        else:
        #            self.e_der = TO IMPLEMENT

        # 3. Remember the current error for the next call.
        #    This must come after step 2, which needs the old value:
        #
        #        self.err = TO IMPLEMENT

        # 4. Combine the four terms of equation (8).
        #    Multiply err, self.e_int and self.e_der each by its gain
        #    (self._p, self._i, self._d) and add the hover thrust offset
        #    self._k. REPLACE the line `u = 0.0` just below with:
        #
        #        u = TO IMPLEMENT
        #
        #    Until it is replaced, u stays 0.0 and the model never leaves the
        #    floor.
        u = 0.0

        # 5. Clip u to the range PX4 accepts ("The range" in Activity 1).
        #    A normalized thrust outside [0.0, 1.0] does not exist, so
        #    saturate it with np.clip:
        #
        #        u = np.clip(TO IMPLEMENT)
        #
        #    check_range() in Notebook 3 fails if this line is missing.

        # 6. Nothing to write: the method returns u as a plain float.
        return float(u)

    def reset(self):
        """
        Clear the state carried between calls, so a new flight does not start
        with the integral and previous error of the last one.
        """
        # ------------------------------------------------------------------
        # TO IMPLEMENT: set the three state variables back to 0.0. Notebook 3,
        # Activity 1, "What reset does" explains why, and Activity 3 checks
        # it. Under each numbered comment below, write the line shown, with
        # TO IMPLEMENT replaced by the value the variable starts from.
        # ------------------------------------------------------------------

        # 1. Forget the previous error, so the first derivative of the next
        #    flight is not computed against the last error of this one:
        #
        #        self.err = TO IMPLEMENT

        # 2. Empty the integral, so the next flight does not start with the
        #    thrust this flight accumulated:
        #
        #        self.e_int = TO IMPLEMENT

        # 3. Clear the derivative, so the node in Notebook 4 does not log a
        #    stale D term before the first step:
        #
        #        self.e_der = TO IMPLEMENT

        # When all three lines are written, delete the `pass` below.
        pass
