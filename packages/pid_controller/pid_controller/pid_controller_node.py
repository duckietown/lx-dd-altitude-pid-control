#!/usr/bin/env python3
"""
DD24 Altitude PID Controller Node
==================================
Controls the vertical altitude of the Duckiedrone DD24 using PX4 in OFFBOARD
mode via MAVROS2.

Control interface
-----------------
The PID outputs a **normalized thrust** in [0.0, 1.0] which is sent to PX4
via the MAVROS2 ``setpoint_attitude`` plugin:

  /mavros/setpoint_attitude/attitude  (geometry_msgs/PoseStamped)
      → level quaternion (w=1) unless a joystick override is active

  /mavros/setpoint_attitude/thrust    (mavros_msgs/Thrust)
      → idle (0.1) until /enable_altitude_control is called,
        then PID-computed value

Debug output
------------
While the PID is ACTIVE the node logs its internals twice a second, so the
controller can be tuned by watching the terminal it was launched from::

  alt=+0.231 sp=+0.250 err=+0.019 | P=+0.002 I=+0.011 D=-0.004 K=+0.700 | raw=0.709 cmd=0.709

``cmd`` is the normalized thrust PX4 actually receives, ``raw`` the PID output
before the thrust cap.  ``raw`` equals ``P + I + D + K`` except while the
output is saturated at the [0, 1] limit; the two diverging is itself the
signal that the controller is asking for more thrust than it can have.

OFFBOARD heartbeat
------------------
PX4 will exit OFFBOARD mode if it does not receive setpoints for more than
~0.5 s.  A 20 Hz timer always publishes (level attitude + thrust) regardless
of whether the altitude PID is active.

State machine
-------------
1. PREFLIGHT  – streams 100 idle setpoints before requesting OFFBOARD
2. OFFBOARD_IDLE – OFFBOARD active, armed; idle thrust (drone stays on ground)
3. ACTIVE     – after /enable_altitude_control Trigger; PID controls thrust

Safety
------
- Call ``ros2 service call /enable_altitude_control std_srvs/srv/Trigger``
  to activate the PID.  Without this the drone will not fly even if armed.
- An RC transmitter with a mode switch is recommended for emergency override.
- The node disables PID and logs a warning if range data stops for > 1 s.
"""

import os
import math
import time
import signal
import yaml
import threading
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Range
from mavros_msgs.msg import State, Thrust, ManualControl
from mavros_msgs.srv import SetMode, CommandBool, CommandLong
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult


class ControlState(Enum):
    PREFLIGHT = auto()
    OFFBOARD_IDLE = auto()
    ACTIVE = auto()


class AltitudePIDNode(Node):
    """ROS2 node for altitude PID control on the DD24 via MAVROS2."""

    # Normalized thrust sent when control is inactive (keeps OFFBOARD alive
    # without spinning up motors enough to fly)
    IDLE_THRUST = 0.1

    # Frequency of the setpoint publishing timer (Hz) — must be > 2 Hz
    CONTROL_FREQ = 20.0

    # Number of setpoints streamed before requesting OFFBOARD mode
    PREFLIGHT_COUNT = 100

    # Timeout before declaring range sensor lost (seconds)
    RANGE_TIMEOUT = 1.0

    # Joystick override attitude timeout (seconds)
    JOY_TIMEOUT = 0.5

    # Interval between PID term log lines while ACTIVE (seconds)
    DEBUG_LOG_PERIOD = 0.5

    def __init__(self):
        super().__init__('altitude_pid_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('range_topic', 'RANGE_TOPIC')
        self.declare_parameter('setpoint_z', 0.25)
        self.declare_parameter('control_frequency', self.CONTROL_FREQ)
        self.declare_parameter('thrust_cap', 0.55)

        range_topic = self.get_parameter('range_topic').value
        self.setpoint_z = self.get_parameter('setpoint_z').value
        freq = self.get_parameter('control_frequency').value
        self.thrust_cap = self.get_parameter('thrust_cap').value

        self.get_logger().info(f'Range topic: {range_topic}')
        self.get_logger().info(f'Altitude setpoint: {self.setpoint_z} m')

        # ── Load PID gains from YAML (used as defaults) ─────────────────────
        import solution as _sol_pkg
        gains_path = os.path.join(os.path.dirname(_sol_pkg.__file__), 'z_pid.yaml')
        self.get_logger().info(f'Loading gains from: {gains_path}')
        with open(gains_path, 'r') as f:
            gains = yaml.full_load(f)

        # ── Declare PID gains as ROS2 parameters (tunable at runtime) ────────
        self.declare_parameter('kp', gains['Kp'])
        self.declare_parameter('ki', gains['Ki'])
        self.declare_parameter('kd', gains['Kd'])
        self.declare_parameter('k', gains['K'])

        # Import student PID
        from solution.pid_class import PID
        self.pid = PID(
            kp=self.get_parameter('kp').value,
            ki=self.get_parameter('ki').value,
            kd=self.get_parameter('kd').value,
            k=self.get_parameter('k').value,
        )
        self.get_logger().info(
            f"PID gains — Kp={self.pid._p}, Ki={self.pid._i}, "
            f"Kd={self.pid._d}, K={self.pid._k}"
        )

        # ── Parameter change callback (live tuning) ──────────────────────────
        self.add_on_set_parameters_callback(self._cb_params)

        # ── State ────────────────────────────────────────────────────────────
        self.control_state = ControlState.PREFLIGHT
        self.current_mavros_state = State()
        self.current_altitude = 0.0
        self.last_range_time: float | None = None
        # None = dt not yet known; forces dt=0.0 on the first control cycle
        # after the PID starts flying, so the derivative term doesn't spike
        # off a stale last_time after a reset.
        self.last_time: float | None = None

        # Joystick attitude override
        self.override_attitude: PoseStamped | None = None
        self.last_joy_time: float | None = None

        self._preflight_count = 0
        self._last_request_time = 0.0  # throttle OFFBOARD/arm requests
        self._was_armed = False
        self._thrust_cap_active = False  # tracks rising edge for the cap-hit warning
        self._last_debug_log = 0.0
        self._lock = threading.Lock()

        # ── QoS profile for MAVROS ───────────────────────────────────────────
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_attitude = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_attitude/attitude',
            10,
        )
        self.pub_thrust = self.create_publisher(
            Thrust,
            '/mavros/setpoint_attitude/thrust',
            10,
        )
        self.pub_manual_control = self.create_publisher(
            ManualControl,
            '/mavros/manual_control/send',
            10,
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            Range,
            range_topic,
            self._cb_range,
            qos_profile=mavros_qos,
        )
        self.create_subscription(
            State,
            '/mavros/state',
            self._cb_mavros_state,
            10,
        )
        self.create_subscription(
            PoseStamped,
            '/dd24/attitude_override',
            self._cb_attitude_override,
            10,
        )

        # ── Service servers ──────────────────────────────────────────────────
        self.create_service(
            Trigger,
            '/enable_altitude_control',
            self._srv_enable_control,
        )

        # ── Service clients ──────────────────────────────────────────────────
        self.cli_set_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_arming = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_command = self.create_client(CommandLong, '/mavros/cmd/command')

        self.get_logger().info('Waiting for MAVROS services...')
        self.cli_set_mode.wait_for_service()
        self.cli_arming.wait_for_service()
        self.cli_command.wait_for_service()
        self.get_logger().info('MAVROS services ready!')

        # ── 20 Hz control timer ──────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / freq, self._timer_cb)

        self.get_logger().info('AltitudePIDNode initialised — streaming preflight setpoints…')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _cb_range(self, msg: Range) -> None:
        if not (msg.min_range <= msg.range <= msg.max_range):
            return  # outside the sensor's own reported range = glitch/sentinel; keep last-valid
        with self._lock:
            self.current_altitude = msg.range
            self.last_range_time = self.get_clock().now().nanoseconds * 1e-9

    def _cb_mavros_state(self, msg: State) -> None:
        with self._lock:
            self.current_mavros_state = msg

    def _cb_attitude_override(self, msg: PoseStamped) -> None:
        with self._lock:
            self.override_attitude = msg
            self.last_joy_time = self.get_clock().now().nanoseconds * 1e-9

    def _cb_params(self, params) -> SetParametersResult:
        with self._lock:
            for p in params:
                if p.name == 'kp':
                    self.pid._p = p.value
                elif p.name == 'ki':
                    self.pid._i = p.value
                elif p.name == 'kd':
                    self.pid._d = p.value
                elif p.name == 'k':
                    self.pid._k = p.value
                elif p.name == 'setpoint_z':
                    self.setpoint_z = p.value
                elif p.name == 'thrust_cap':
                    self.thrust_cap = p.value
        self.get_logger().info(
            f'Params updated — Kp={self.pid._p}, Ki={self.pid._i}, '
            f'Kd={self.pid._d}, K={self.pid._k}, setpoint={self.setpoint_z}, '
            f'thrust_cap={self.thrust_cap}'
        )
        return SetParametersResult(successful=True)

    # ── Service handler ──────────────────────────────────────────────────────

    def _srv_enable_control(self, request, response):
        with self._lock:
            if self.control_state == ControlState.OFFBOARD_IDLE:
                self.pid.reset()
                self.last_time = None  # forces dt=0.0 on the first control cycle while flying
                self.control_state = ControlState.ACTIVE
                self.get_logger().info('✅ Altitude PID control ENABLED')
                response.success = True
                response.message = 'Altitude PID control enabled.'
            elif self.control_state == ControlState.ACTIVE:
                self.control_state = ControlState.OFFBOARD_IDLE
                self.get_logger().info('⏸  Altitude PID control DISABLED')
                response.success = True
                response.message = 'Altitude PID control disabled.'
            else:
                response.success = False
                response.message = (
                    'Not ready: waiting for OFFBOARD mode to be established. '
                    'Current state: ' + self.control_state.name
                )
        return response

    # ── Main timer callback ──────────────────────────────────────────────────

    def _timer_cb(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9

        with self._lock:
            state = self.control_state
            altitude = self.current_altitude
            setpoint = self.setpoint_z
            last_range = self.last_range_time

        # Check for range sensor timeout in ACTIVE state
        if state == ControlState.ACTIVE:
            if last_range is None or (now - last_range) > self.RANGE_TIMEOUT:
                self.get_logger().warn(
                    'Range sensor data lost — disabling altitude PID control!'
                )
                with self._lock:
                    self.control_state = ControlState.OFFBOARD_IDLE
                state = ControlState.OFFBOARD_IDLE

        # Compute thrust
        thrust_value = 0.0
        if state == ControlState.PREFLIGHT:
            thrust_value = 0.0
        elif state == ControlState.OFFBOARD_IDLE:
            thrust_value = self.IDLE_THRUST
        elif state == ControlState.ACTIVE:
            dt = 0.0 if self.last_time is None else (now - self.last_time)
            self.last_time = now
            err = setpoint - altitude
            raw_thrust = self.pid.step(err, dt)

            thrust_value = min(self.thrust_cap, raw_thrust)
            if raw_thrust > self.thrust_cap:
                if not self._thrust_cap_active:
                    self.get_logger().warn(
                        f'⚠ THRUST CAP HIT: PID requested {raw_thrust:.3f}, '
                        f'capped to {self.thrust_cap:.3f} (err={err:.3f} m)'
                    )
                self._thrust_cap_active = True
            else:
                self._thrust_cap_active = False

            self._log_pid_terms(
                now, altitude, setpoint, err, raw_thrust, thrust_value
            )

        # Compute attitude (level or joystick override)
        attitude_msg = self._make_level_attitude()
        if state in (ControlState.OFFBOARD_IDLE, ControlState.ACTIVE):
            with self._lock:
                joy = self.override_attitude
                joy_t = self.last_joy_time
            if joy is not None and joy_t is not None and (now - joy_t) < self.JOY_TIMEOUT:
                attitude_msg = joy
                attitude_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish
        self.pub_attitude.publish(attitude_msg)
        thrust_msg = Thrust()
        thrust_msg.header.stamp = self.get_clock().now().to_msg()
        thrust_msg.thrust = float(thrust_value)
        self.pub_thrust.publish(thrust_msg)

        # Neutral MANUAL_CONTROL stream, satisfies COM_RC_IN_MODE=3 arming health
        # (no RC receiver on this airframe; MAVROS-sent MANUAL_CONTROL stands in for it)
        mc_msg = ManualControl()
        mc_msg.header.stamp = self.get_clock().now().to_msg()
        mc_msg.x = 0.0
        mc_msg.y = 0.0
        mc_msg.z = 0.0
        mc_msg.r = 0.0
        self.pub_manual_control.publish(mc_msg)

        # State machine transitions
        if state == ControlState.PREFLIGHT:
            with self._lock:
                self._preflight_count += 1
                count = self._preflight_count

            # Wait for MAVROS connection and preflight count
            with self._lock:
                connected = self.current_mavros_state.connected
            if count >= self.PREFLIGHT_COUNT and connected:
                with self._lock:
                    self.control_state = ControlState.OFFBOARD_IDLE
                self.get_logger().info(
                    'Preflight complete, MAVROS connected — transitioning to OFFBOARD_IDLE.'
                )

        # In OFFBOARD_IDLE (or ACTIVE), continuously ensure OFFBOARD mode + armed
        if state in (ControlState.OFFBOARD_IDLE, ControlState.ACTIVE):
            with self._lock:
                mavros_state = self.current_mavros_state
            if (now - self._last_request_time) > 5.0:
                if mavros_state.mode != 'OFFBOARD':
                    self._request_set_mode('OFFBOARD')
                    self._last_request_time = now
                elif not mavros_state.armed:
                    self._request_arm(True)
                    self._last_request_time = now

            # Detect fresh arming → reset PID timing
            if mavros_state.armed and not self._was_armed:
                self.pid.reset()
                self.last_time = None  # same dt=0.0-on-first-cycle guard as enable_control
                self.get_logger().info('Vehicle just armed — PID state reset.')
            self._was_armed = mavros_state.armed

    # Helpers

    def _log_pid_terms(self, now, altitude, setpoint, err,
                       raw_thrust, thrust_cmd) -> None:
        """Log the controller's internals, throttled to DEBUG_LOG_PERIOD.

        Printing every control cycle would put 20 lines a second through the
        container's log driver and scroll too fast to read; twice a second
        still shows the shape of the response.  The throttle is checked before
        anything is computed, so the skipped cycles cost one subtraction.
        """
        if (now - self._last_debug_log) < self.DEBUG_LOG_PERIOD:
            return
        self._last_debug_log = now

        # Recover the individual contributions from the PID's own state. The
        # PID class contract fixes these attribute names, so this needs no
        # cooperation from the student's implementation.
        p_term = self.pid._p * err
        i_term = self.pid._i * self.pid.e_int
        d_term = self.pid._d * self.pid.e_der

        self.get_logger().info(
            f'alt={altitude:+.3f} sp={setpoint:+.3f} err={err:+.3f} | '
            f'P={p_term:+.3f} I={i_term:+.3f} D={d_term:+.3f} K={self.pid._k:+.3f} | '
            f'raw={raw_thrust:.3f} cmd={thrust_cmd:.3f}'
        )

    def _make_level_attitude(self) -> PoseStamped:
        """Return a level-hover PoseStamped holding the drone's boot heading.

        Setpoints are ENU but PX4 is NED, so yaw differs by 90°. ENU yaw=+90°
        (z=w=√½) maps to PX4 native yaw=0 ("hold heading"); an identity
        quaternion would be ENU yaw=0, which PX4 slews to as a ~90° spin.
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.70710678, w=0.70710678)
        return msg

    def _request_set_mode(self, mode: str) -> None:
        """Send an async SetMode request (non-blocking, result logged via callback)."""
        if not self.cli_set_mode.service_is_ready():
            self.get_logger().warn('set_mode service not yet available')
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.cli_set_mode.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'{mode} mode requested successfully.')
            if f.result() and f.result().mode_sent
            else self.get_logger().warn(f'Failed to set {mode} mode.')
        )

    def _request_arm(self, arm: bool) -> None:
        """Send an async arming request (non-blocking, result logged via callback)."""
        if not self.cli_arming.service_is_ready():
            self.get_logger().warn('arming service not yet available')
            return
        req = CommandBool.Request()
        req.value = arm
        future = self.cli_arming.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info('Vehicle armed successfully.')
            if f.result() and f.result().success
            else self.get_logger().warn('Arming failed. Check pre-arm checks.')
        )


_shutdown_requested = False


def _request_shutdown(signum, frame):
    """SIGINT/SIGTERM handler: just sets a flag. The actual disarm runs on
    the main thread afterward, never spin the executor from inside a
    signal handler."""
    global _shutdown_requested
    _shutdown_requested = True


def _blocking_force_disarm(node, timeout_sec=2.0) -> None:
    """Synchronous force-disarm (MAV_CMD 400, param2=21196, the PX4 kill code),
    called on every shutdown path so the vehicle never stays armed once
    the setpoint stream stops."""
    req = CommandLong.Request(
        broadcast=False, command=400, confirmation=0,
        param1=0.0, param2=21196.0, param3=0.0, param4=0.0,
        param5=0.0, param6=0.0, param7=0.0,
    )
    future = node.cli_command.call_async(req)
    t0 = time.time()
    while rclpy.ok() and not future.done() and (time.time() - t0) < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.05)
    if future.done():
        node.get_logger().warn('🛑 Shutdown force-disarm: request acknowledged.')
    else:
        node.get_logger().error(
            '🛑 Shutdown force-disarm: NO acknowledgement received in time, '
            'FCU may still be armed. Verify manually.'
        )


def main(args=None):
    # Use our own SIGINT handler so Ctrl-C doesn't skip the disarm below.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = AltitudePIDNode()
    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    # Wait for FCU connection before entering main loop
    node.get_logger().info('Waiting for FCU connection...')
    while rclpy.ok() and not node.current_mavros_state.connected and not _shutdown_requested:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.current_mavros_state.connected:
        node.get_logger().info('FCU connected!')

    try:
        while rclpy.ok() and not _shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.get_logger().warn('Shutting down, force-disarming...')
        with node._lock:
            node.control_state = ControlState.OFFBOARD_IDLE
        _blocking_force_disarm(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
