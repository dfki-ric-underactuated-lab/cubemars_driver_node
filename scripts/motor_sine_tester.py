#!/usr/bin/env python3
"""
motor_sine_tester.py

ROS 2 (rclpy) node to test N motors via robot_control_msgs:
- Publishes robot_control_msgs/JointCommand on topic `joint_commands`
- Subscribes robot_control_msgs/JointState on topic `joint_states`
- Optionally activates a Lifecycle driver node (e.g. cubemars_hardware_node)
- Always publishes at `rate_hz` to avoid “no messages -> damping/safety” behavior

Modes:
  1) mode=driver_pd
     Sends desired position/velocity + kp/kd (effort=0). Driver closes the loop.

  2) mode=effort_pd
     Node closes loop and sends effort only (kp/kd=0). Uses joint_state feedback.

Trajectory (RUN phase):
  q_des  = center_q + a(t) * sin(omega*t_rel + phase)
  qd_des = a_dot(t) * sin(...) + a(t) * omega * cos(...)
where:
  - omega = vmax / A  (vmax is max |qd_des| once fully ramped)
  - a(t) ramps smoothly from 0 -> A over ramp_time_s (smoothstep),
    so the trajectory starts at the current joint position with ~0 initial velocity.

Plots:
  Saves one PNG per motor with: position, velocity, effort (measured + targets where applicable)

Notes:
  - The driver expects all arrays length n_motors (always true here).
  - Topics are namespaced under "/<driver_node_name>/..." (as in your working version).
  - Lifecycle polling is throttled to lifecycle_tick_hz (default 1 Hz) to reduce chatter.
"""

import os
import math
import time
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from robot_control_msgs.msg import JointCommand, JointState

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State


@dataclass
class Params:
    n_motors: int
    rate_hz: float
    duration_s: float
    mode: str  # "driver_pd" or "effort_pd"

    # Trajectory
    amplitude: float
    vmax: float
    phase_offsets: np.ndarray
    position_offsets: np.ndarray

    # Gains
    kp: np.ndarray
    kd: np.ndarray

    # Effort limits
    effort_limit: float  # <=0 disables clipping

    # Topics (relative, will be namespaced by driver_node_name)
    topic_cmd: str
    topic_state: str

    # Lifecycle
    activate_on_start: bool
    driver_node_name: str
    lifecycle_tick_hz: float

    # Timing
    use_timer_time: bool  # if True, advance t by dt each tick (deterministic phase)

    # Start behavior
    start_from_current: bool
    ramp_time_s: float

    # Output
    out_dir: str
    show_plots: bool


def _as_array(val, n: int, name: str) -> np.ndarray:
    """Accept scalar or list-like; return np.ndarray length n."""
    if isinstance(val, (float, int)):
        return np.full((n,), float(val), dtype=np.float64)
    arr = np.array(val, dtype=np.float64).reshape(-1)
    if arr.size != n:
        raise ValueError(f"Parameter '{name}' must have length {n}, got {arr.size}")
    return arr


class MotorSineTester(Node):
    def __init__(self):
        super().__init__("motor_sine_tester")

        # ---------------- Parameters ----------------
        self.declare_parameter("n_motors", 1)
        self.declare_parameter("rate_hz", 200.0)
        self.declare_parameter("duration_s", 10.0)
        self.declare_parameter("mode", "driver_pd")  # or "effort_pd"

        self.declare_parameter("amplitude", float(2.0 * math.pi))
        self.declare_parameter("vmax", 4.0)

        self.declare_parameter("phase_offsets", [])      # if empty -> zeros
        self.declare_parameter("position_offsets", [])   # if empty -> zeros

        self.declare_parameter("kp", 20.0)  # scalar or list
        self.declare_parameter("kd", 0.5)   # scalar or list

        self.declare_parameter("effort_limit", 0.0)  # <=0 disables clipping

        self.declare_parameter("topic_cmd", "joint_commands")
        self.declare_parameter("topic_state", "joint_states")

        self.declare_parameter("activate_on_start", True)
        self.declare_parameter("driver_node_name", "cubemars_hardware_node")
        self.declare_parameter("lifecycle_tick_hz", 10.0)

        self.declare_parameter("use_timer_time", False)

        self.declare_parameter("start_from_current", True)
        self.declare_parameter("ramp_time_s", 1.0)

        self.declare_parameter("out_dir", "motor_test_plots")
        self.declare_parameter("show_plots", False)

        # ---------------- Read parameters ----------------
        n = int(self.get_parameter("n_motors").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        duration_s = float(self.get_parameter("duration_s").value)
        mode = str(self.get_parameter("mode").value).strip()
        if mode not in ("driver_pd", "effort_pd"):
            raise ValueError("mode must be 'driver_pd' or 'effort_pd'")

        amp = float(self.get_parameter("amplitude").value)
        vmax = float(self.get_parameter("vmax").value)
        if abs(amp) < 1e-12:
            raise ValueError("amplitude must be non-zero.")

        phase_offsets = self.get_parameter("phase_offsets").value
        position_offsets = self.get_parameter("position_offsets").value
        if isinstance(phase_offsets, list) and len(phase_offsets) == 0:
            phase_offsets = [0.0] * n
        if isinstance(position_offsets, list) and len(position_offsets) == 0:
            position_offsets = [0.0] * n

        kp = self.get_parameter("kp").value
        kd = self.get_parameter("kd").value

        effort_limit = float(self.get_parameter("effort_limit").value)

        topic_cmd = str(self.get_parameter("topic_cmd").value)
        topic_state = str(self.get_parameter("topic_state").value)

        activate_on_start = bool(self.get_parameter("activate_on_start").value)
        driver_node_name = str(self.get_parameter("driver_node_name").value).strip()
        lifecycle_tick_hz = float(self.get_parameter("lifecycle_tick_hz").value)
        if lifecycle_tick_hz <= 0.0:
            lifecycle_tick_hz = 1.0

        use_timer_time = bool(self.get_parameter("use_timer_time").value)

        start_from_current = bool(self.get_parameter("start_from_current").value)
        ramp_time_s = float(self.get_parameter("ramp_time_s").value)
        if ramp_time_s < 0.0:
            ramp_time_s = 0.0

        out_dir = str(self.get_parameter("out_dir").value)
        show_plots = bool(self.get_parameter("show_plots").value)

        self.params = Params(
            n_motors=n,
            rate_hz=rate_hz,
            duration_s=duration_s,
            mode=mode,
            amplitude=amp,
            vmax=vmax,
            phase_offsets=_as_array(phase_offsets, n, "phase_offsets"),
            position_offsets=_as_array(position_offsets, n, "position_offsets"),
            kp=_as_array(kp, n, "kp"),
            kd=_as_array(kd, n, "kd"),
            effort_limit=effort_limit,
            topic_cmd=topic_cmd,
            topic_state=topic_state,
            activate_on_start=activate_on_start,
            driver_node_name=driver_node_name,
            lifecycle_tick_hz=lifecycle_tick_hz,
            use_timer_time=use_timer_time,
            start_from_current=start_from_current,
            ramp_time_s=ramp_time_s,
            out_dir=out_dir,
            show_plots=show_plots,
        )

        # omega from vmax = A*omega
        self.omega = self.params.vmax / self.params.amplitude

        # Lifecycle tick throttling
        self.lifecycle_tick_period = 1.0 / self.params.lifecycle_tick_hz
        self.last_lifecycle_tick_t = -1e9

        # ---------------- QoS ----------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Namespaced topics (your working setup)
        cmd_topic = f"/{self.params.driver_node_name}/{self.params.topic_cmd}"
        state_topic = f"/{self.params.driver_node_name}/{self.params.topic_state}"
        self.pub_cmd = self.create_publisher(JointCommand, cmd_topic, qos)
        self.sub_state = self.create_subscription(JointState, state_topic, self._on_state, qos)

        # ---------------- Lifecycle clients (async) ----------------
        self.driver_active = (not self.params.activate_on_start)
        self.fut_get_state = None
        self.fut_change_state = None

        self.srv_get_state = f"/{self.params.driver_node_name}/get_state"
        self.srv_change_state = f"/{self.params.driver_node_name}/change_state"
        self.cli_get_state = self.create_client(GetState, self.srv_get_state)
        self.cli_change_state = self.create_client(ChangeState, self.srv_change_state)

        # ---------------- State & logging ----------------
        self.last_state: Optional[JointState] = None

        self.started = False
        self.t0_wall = time.time()

        self.dt = 1.0 / self.params.rate_hz
        self.t_sim = 0.0

        self.phase = "KEEPALIVE"  # KEEPALIVE -> RUN

        # RUN phase anchors
        self.run_t0: Optional[float] = None      # time when RUN begins
        self.center_q: Optional[np.ndarray] = None  # per-motor center position for sine

        self.t_log: List[float] = []
        self.q_des_log: List[np.ndarray] = []
        self.qd_des_log: List[np.ndarray] = []
        self.tau_cmd_log: List[np.ndarray] = []
        self.t_meas_log: List[float] = []
        self.q_meas_log: List[np.ndarray] = []
        self.qd_meas_log: List[np.ndarray] = []
        self.tau_meas_log: List[np.ndarray] = []

        # ---------------- Timer ----------------
        self.timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(
            f"motor_sine_tester: mode={self.params.mode}, n={self.params.n_motors}, "
            f"rate={self.params.rate_hz}Hz, duration={self.params.duration_s}s, "
            f"amp={self.params.amplitude}rad, vmax={self.params.vmax}rad/s, omega={self.omega:.3f}rad/s"
        )
        self.get_logger().info(f"Topics: cmd='{cmd_topic}', state='{state_topic}'")
        self.get_logger().info(
            f"Start: start_from_current={self.params.start_from_current}, ramp_time_s={self.params.ramp_time_s}"
        )
        if self.params.activate_on_start:
            self.get_logger().info(
                f"Lifecycle activation enabled: driver='{self.params.driver_node_name}', "
                f"poll={self.params.lifecycle_tick_hz}Hz "
                f"(services: '{self.srv_get_state}', '{self.srv_change_state}')"
            )

    # ---------------- Callbacks & helpers ----------------
    def _on_state(self, msg: JointState):
        n = self.params.n_motors
        if len(msg.position) < n or len(msg.velocity) < n or len(msg.effort) < n:
            self.get_logger().warn(
                f"joint_states lengths pos={len(msg.position)} vel={len(msg.velocity)} eff={len(msg.effort)} "
                f"but expected >= {n}. Ignoring."
            )
            return
        self.t_meas_log.append(self._now_t())
        self.q_meas_log.append(msg.position[:n])
        self.qd_meas_log.append(msg.velocity[:n])
        self.tau_meas_log.append(msg.effort[:n])   
        self.last_state = msg

    def _now_t(self) -> float:
        if self.params.use_timer_time:
            t = self.t_sim
            self.t_sim += self.dt
            return t
        return time.time() - self.t0_wall

    def _clip_effort(self, tau: np.ndarray) -> np.ndarray:
        lim = self.params.effort_limit
        if lim is None or lim <= 0.0:
            return tau
        return np.clip(tau, -lim, lim)

    def _make_empty_cmd(self) -> JointCommand:
        n = self.params.n_motors
        cmd = JointCommand()
        cmd.position = [0.0] * n
        cmd.velocity = [0.0] * n
        cmd.acceleration = [0.0] * n
        cmd.effort = [0.0] * n
        cmd.kp = [0.0] * n
        cmd.kd = [0.0] * n
        return cmd

    def _publish_keepalive(self):
        """
        Keepalive at rate_hz to prevent damping/safety due to missing messages.

        Strategy:
        - If joint_state available: "hold current" (position=q_meas, velocity=0, kp=kd=0, effort=0)
        - Else: all zeros (kp=kd=0, effort=0)
        """
        n = self.params.n_motors
        cmd = self._make_empty_cmd()

        if self.last_state is not None and len(self.last_state.position) >= n:
            cmd.position = list(self.last_state.position[:n])
            cmd.velocity = [0.0] * n

        self.pub_cmd.publish(cmd)

    def _lifecycle_tick(self):
        """
        Non-blocking lifecycle activation:
        - Periodically call get_state (throttled by lifecycle_tick_hz in _on_timer)
        - If INACTIVE, call ACTIVATE via change_state
        """
        if not self.params.activate_on_start:
            self.driver_active = True
            return
        if self.driver_active:
            return

        # Services must be ready (non-blocking check)
        if (not self.cli_get_state.service_is_ready()) or (not self.cli_change_state.service_is_ready()):
            return

        # If we don't have a request in flight, start get_state
        if self.fut_get_state is None and self.fut_change_state is None:
            self.fut_get_state = self.cli_get_state.call_async(GetState.Request())
            return

        # If get_state completed, interpret it
        if self.fut_get_state is not None and self.fut_get_state.done():
            res = self.fut_get_state.result()
            self.fut_get_state = None
            if res is None:
                return

            sid = res.current_state.id
            if sid == State.PRIMARY_STATE_ACTIVE:
                self.driver_active = True
                self.get_logger().info("Driver is ACTIVE.")
                return

            if sid == State.PRIMARY_STATE_INACTIVE:
                req = ChangeState.Request()
                req.transition.id = Transition.TRANSITION_ACTIVATE
                self.fut_change_state = self.cli_change_state.call_async(req)
                self.get_logger().info("Driver INACTIVE -> sending ACTIVATE transition...")
                return

            # Other states: keep polling at lifecycle_tick_hz
            return

        # If change_state completed, clear it (next poll will re-check state)
        if self.fut_change_state is not None and self.fut_change_state.done():
            _ = self.fut_change_state.result()
            self.fut_change_state = None

    def _desired_traj(self, t: float) -> (np.ndarray, np.ndarray):
        """
        Desired trajectory with smooth amplitude ramp-in to avoid initial jumps.

        Uses t_rel = t - run_t0 for phase progression.
        Center is self.center_q, captured at RUN start.

        a(t) ramps from 0 -> A over ramp_time_s using smoothstep:
          s(x)=3x^2-2x^3, x=t_rel/Tr clamped to [0,1]
        """
        assert self.run_t0 is not None
        assert self.center_q is not None

        t_rel = t - self.run_t0
        if t_rel < 0.0:
            t_rel = 0.0

        Tr = self.params.ramp_time_s
        if Tr <= 1e-9:
            s = 1.0
            s_dot = 0.0
        else:
            x = max(0.0, min(1.0, t_rel / Tr))
            s = x * x * (3.0 - 2.0 * x)
            s_dot = (6.0 * x * (1.0 - x)) / Tr

        A = self.params.amplitude
        a = A * s
        a_dot = A * s_dot

        th = self.omega * t_rel + self.params.phase_offsets
        q_des = self.center_q + a * np.sin(th)
        qd_des = a_dot * np.sin(th) + a * self.omega * np.cos(th)
        return q_des, qd_des

    def _publish_sine_command_and_log(self, t: float):
        n = self.params.n_motors

        q_des, qd_des = self._desired_traj(t)

        q_meas = np.array(self.last_state.position[:n], dtype=np.float64)
        qd_meas = np.array(self.last_state.velocity[:n], dtype=np.float64)
        tau_meas = np.array(self.last_state.effort[:n], dtype=np.float64)

        cmd = self._make_empty_cmd()

        if self.params.mode == "driver_pd":
            cmd.position = q_des.tolist()
            cmd.velocity = qd_des.tolist()
            cmd.effort = [0.0] * n
            cmd.kp = self.params.kp.tolist()
            cmd.kd = self.params.kd.tolist()
            tau_cmd = np.zeros((n,), dtype=np.float64)  # driver internal

        else:  # effort_pd
            e = q_des - q_meas
            ed = qd_des - qd_meas
            tau_cmd = self.params.kp * e + self.params.kd * ed
            tau_cmd = self._clip_effort(tau_cmd)

            cmd.effort = tau_cmd.tolist()
            cmd.kp = [0.0] * n
            cmd.kd = [0.0] * n
            # position/velocity arrays remain filled (zeros) but unused

        self.pub_cmd.publish(cmd)

        # Log
        self.t_log.append(t)
        self.q_des_log.append(q_des.copy())
        self.qd_des_log.append(qd_des.copy())
        self.tau_cmd_log.append(tau_cmd.copy())

    def _finalize_and_plot(self):
        if len(self.t_log) < 2:
            self.get_logger().warn("Not enough data to plot.")
            return

        t = np.array(self.t_log, dtype=np.float64)
        q_des = np.vstack(self.q_des_log)
        qd_des = np.vstack(self.qd_des_log)
        tau_cmd = np.vstack(self.tau_cmd_log)

        tl = np.vstack(self.t_meas_log)
        q = np.vstack(self.q_meas_log)
        qd = np.vstack(self.qd_meas_log)
        tau = np.vstack(self.tau_meas_log)

        os.makedirs(self.params.out_dir, exist_ok=True)

        import matplotlib
        if not self.params.show_plots:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        for i in range(self.params.n_motors):
            fig, axs = plt.subplots(
                3, 1,
                sharex=True,
                figsize=(10, 4.5),  # smaller height
                constrained_layout=False
            )

            ax1, ax2, ax3 = axs

            # --- Position ---
            ax1.plot(tl, q[:, i], label="q_meas")
            ax1.plot(t, q_des[:, i], "--", label="q_des")
            ax1.set_ylabel("Pos [rad]", fontsize=9)
            ax1.grid(True)
            ax1.legend(loc="upper right", fontsize=8, frameon=False)

            # --- Velocity ---
            ax2.plot(tl, qd[:, i], label="qd_meas")
            ax2.plot(t, qd_des[:, i], "--", label="qd_des")
            ax2.set_ylabel("Vel [rad/s]", fontsize=9)
            ax2.grid(True)
            ax2.legend(loc="upper right", fontsize=8, frameon=False)

            # --- Effort ---
            ax3.plot(tl, tau[:, i], label="effort_meas")
            if self.params.mode == "effort_pd":
                ax3.plot(t, tau_cmd[:, i], "--", label="effort_cmd")
            ax3.set_ylabel("Eff", fontsize=9)
            ax3.set_xlabel("Time [s]", fontsize=9)
            ax3.grid(True)
            ax3.legend(loc="upper right", fontsize=8, frameon=False)

            # Make ticks smaller and remove x tick labels on upper plots
            for ax in axs:
                ax.tick_params(labelsize=8)
            ax1.tick_params(labelbottom=False)
            ax2.tick_params(labelbottom=False)

            # Tighten spacing (this is the "squeeze" knob)
            fig.subplots_adjust(top=0.90, hspace=0.08)

            fig.suptitle(
                f"mode={self.params.mode} | kp={self.params.kp[i]:g}, kd={self.params.kd[i]:g}",
                fontsize=6
            )

            fname = os.path.join(self.params.out_dir, f"motor_{i:02d}_{self.params.mode}.png")
            fig.savefig(fname, dpi=200, bbox_inches="tight", pad_inches=0.05)
            plt.close(fig)


    # ---------------- Timer loop ----------------
    def _on_timer(self):
        t = self._now_t()

        # Stop condition
        if t > self.params.duration_s:
            self.get_logger().info("Duration reached. Plotting and shutting down.")
            self._finalize_and_plot()
            rclpy.shutdown()
            return

        # Throttled lifecycle polling
        if (t - self.last_lifecycle_tick_t) >= self.lifecycle_tick_period:
            self.last_lifecycle_tick_t = t
            self._lifecycle_tick()

        # Always publish something at rate_hz:
        # - KEEPALIVE until driver ACTIVE and first joint_state received
        # - then RUN sine
        if (not self.driver_active) or (self.last_state is None):
            if not self.started:
                self.started = True
                self.get_logger().info("Publishing keepalive at rate_hz until driver ACTIVE and joint_states received...")
            self._publish_keepalive()
            return

        # Switch to RUN once
        if self.phase != "RUN":
            self.phase = "RUN"
            self.run_t0 = t

            n = self.params.n_motors
            q_meas = np.array(self.last_state.position[:n], dtype=np.float64)

            if self.params.start_from_current:
                self.center_q = q_meas + self.params.position_offsets
            else:
                self.center_q = self.params.position_offsets.copy()

            self.get_logger().info(
                f"Driver ACTIVE and joint_states available -> starting sine trajectory. "
                f"start_from_current={self.params.start_from_current}, ramp_time_s={self.params.ramp_time_s}"
            )

        self._publish_sine_command_and_log(t)


def main():
    rclpy.init()
    node = MotorSineTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt: plotting and exiting.")
        node._finalize_and_plot()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
