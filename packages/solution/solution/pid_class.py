from __future__ import division
import numpy as np


class PID:
    """
    Altitude PID controller for the Duckiedrone DD24.

    The DD24 uses PX4 in OFFBOARD mode with MAVROS2. 
    The PID outputs a **normalized thrust** value in [0.0, 1.0] 
    that is sent directly to the ``setpoint_attitude/thrust`` topic.

    The offset constant ``K`` represents the hover thrust — the base
    normalized thrust needed to hold the drone at a constant altitude
    against gravity (typically ~0.44 for the DD24B).  At hover, the net
    force on the drone is zero:

        F_net = K * F_max - m * g = 0   →   K = m * g / F_max

    Be sure to follow the naming conventions for your control-term variables:

        P term: _p
        I term: _i
        D term: _d

    This ensures your class works correctly with the rest of the code stack.
    """

    def __init__(self, kp, ki, kd, k):
        """
        Initialise the PID controller.

        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        :param k:  Hover thrust offset — normalized value in [0.0, 1.0]
                   added to the PID output so the drone can hover with zero
                   error.  A good starting point for the DD24 is 0.44.
        """
        self._p = kp
        self._i = ki
        self._d = kd
        self._k = k

        # State variables
        self.err = 0.0    # previous error (for derivative)
        self.e_der = 0.0  # derivative term value
        self.e_int = 0.0  # accumulated integral

    def step(self, err, dt):
        """
        Compute one PID step and return the normalized thrust command.

        Implements the discrete PID control function:

            u(t_k) = K_p * e(t_k)
                   + K_i * Σ e(t_i) * Δt
                   + K_d * (e(t_k) − e(t_{k-1})) / Δt
                   + K

        :param err: Current altitude error in metres.
                    Positive means the drone is *below* the setpoint.
        :param dt:  Time elapsed since the last call, in seconds.
        :returns:   Normalized thrust command, clipped to [0.0, 1.0].
        """
        # ------------------------------------------------------------------
        # TODO: implement the steps below, one at a time.
        #
        # Problem 1 in the notebook walks through them in order: get the
        # hover offset working first, then add one term per exercise and
        # observe what changes before moving on.
        # ------------------------------------------------------------------

        # STEP 1 - Integral term state.
        #   Accumulate the error over time into `self.e_int`:
        #   add the current error, weighted by how long it has been acting
        #   (`dt`), to whatever has accumulated so far.
        #   Leave this at 0.0 until Exercise 5.
        self.e_int = 0.0

        # STEP 2 - Derivative term state.
        #   Store the rate of change of the error in `self.e_der`:
        #   the difference between the current error and the previous one
        #   (`self.err`), divided by the elapsed time.
        #   Guard against dt == 0.0: the very first call has no previous
        #   sample, and dividing by zero would produce inf/NaN thrust.
        self.e_der = 0.0

        # NOTE: `self.e_int` and `self.e_der` must be stored on `self`, not
        # kept as local variables. The controller node reads them back to
        # publish each control term separately for plotting and debugging.

        # STEP 3 - Combine the terms.
        #   Multiply each state by its gain (`self._p`, `self._i`, `self._d`)
        #   and add the hover offset `self._k`, following the equation above.
        #   Start with only `self._k` (Exercise 1), then add the P term
        #   (Exercise 2), the D term (Exercise 3) and finally the I term
        #   (Exercise 5).
        u = 0.0

        # STEP 4 - Remember the current error.
        #   Save `err` into `self.err` so the next call can compute the
        #   derivative. Do this *after* using the old value in STEP 2.

        # STEP 5 - Clip and return.
        #   PX4 accepts a normalized thrust in [0.0, 1.0]. Anything outside
        #   that range is physically meaningless and MAVROS2 will reject it,
        #   so saturate the output before returning it as a float.
        return float(u)

    def reset(self):
        """
        Reset all internal state.

        Call this when the drone transitions from ARMED to FLYING, or when
        the simulation is reset, so that stale integral and derivative values
        do not affect the new control epoch.
        """
        # TODO: clear every state variable the controller carries between
        # calls (`self.err`, `self.e_der`, `self.e_int`) back to 0.0.
        # Think about what a leftover integral would do to the first thrust
        # command after a reset (Exercise 6).
        pass
