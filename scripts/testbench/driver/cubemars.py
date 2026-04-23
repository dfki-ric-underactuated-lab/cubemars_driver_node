"""CubeMars motor driver implementation over ROS 2."""

from __future__ import annotations

import logging
import threading
from typing import Callable

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from robot_control_msgs.msg import JointCommand, JointState
from std_msgs.msg import Float32MultiArray

from .base import MotorCommand, MotorDriverBase, MotorState
from ..lifecycle_node import LifecycleNode

_BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

_TARGET_NODE = "/cubemars_hardware_node"


class CubemarsDriver(Node, MotorDriverBase):
    """
    ROS 2 driver for CubeMars actuators.

    Publishes JointCommand messages at *command_freq* Hz and subscribes
    to joint state, temperature, and CAN cycle feedback topics.
    """

    def __init__(
        self,
        joint_id: int = 0,
        command_freq: float = 1000.0,
        default_kd: float = 5.0,
    ) -> None:
        rclpy.init()
        Node.__init__(self, "cubemars_motor_test_node")

        self._joint_id = joint_id
        self._default_kd = default_kd
        self._step_callback: Callable[[], None] | None = None
        self._state = MotorState()
        self._spin_thread: threading.Thread | None = None

        # Pre-allocated command message — mutated in-place to avoid heap
        # allocation inside the 1 kHz control loop.
        self._cmd_msg = JointCommand()
        self._cmd_msg.position = [0.0]
        self._cmd_msg.velocity = [0.0]
        self._cmd_msg.acceleration = [0.0]
        self._cmd_msg.kp = [0.0]
        self._cmd_msg.kd = [default_kd]
        self._cmd_msg.effort = [0.0]

        self._publisher = self.create_publisher(
            JointCommand,
            f"{_TARGET_NODE}/joint_commands",
            1,
        )
        self._timer = self.create_timer(1.0 / command_freq, self._timer_cb)

        self.create_subscription(
            JointState,
            f"{_TARGET_NODE}/joint_states",
            self._state_cb,
            _BEST_EFFORT_QOS,
        )
        self.create_subscription(
            Float32MultiArray,
            "/joint_temperatures",
            self._temp_cb,
            _BEST_EFFORT_QOS,
        )
        self.create_subscription(
            Float32MultiArray,
            "/can_cycle_frequencies",
            self._can_cycle_cb,
            _BEST_EFFORT_QOS,
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)

        self._lifecycle = LifecycleNode(target_node=_TARGET_NODE)
        logging.info("CubemarsDriver initialised")

    # ------------------------------------------------------------------
    # MotorDriverBase
    # ------------------------------------------------------------------

    def send_command(self, command: MotorCommand) -> None:
        jid = self._joint_id
        self._cmd_msg.effort[jid] = command.effort
        self._cmd_msg.velocity[jid] = command.velocity
        self._cmd_msg.position[jid] = command.position
        self._cmd_msg.kp[jid] = command.kp
        self._cmd_msg.kd[jid] = command.kd
        self._publisher.publish(self._cmd_msg)

    def get_latest_state(self) -> MotorState:
        return self._state

    def bring_to_active(self) -> None:
        self._lifecycle.bring_to_state("active")

    def bring_to_inactive(self) -> None:
        self._lifecycle.bring_to_state("inactive")

    def start(self, step_callback: Callable[[], None]) -> None:
        """Launch the background spin thread and begin calling *step_callback* on each tick."""
        self._step_callback = step_callback
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True
        )
        self._spin_thread.start()

    def shutdown(self) -> None:
        self._timer.cancel()
        self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
        self.destroy_node()
        rclpy.shutdown()
        logging.info("CubemarsDriver shut down")

    # ------------------------------------------------------------------
    # ROS 2 callbacks
    # ------------------------------------------------------------------

    def _timer_cb(self) -> None:
        if self._step_callback is not None:
            self._step_callback()

    def _state_cb(self, msg: JointState) -> None:
        jid = self._joint_id
        self._state.effort = msg.effort[jid]
        self._state.velocity = msg.velocity[jid]
        self._state.position = msg.position[jid]

    def _temp_cb(self, msg: Float32MultiArray) -> None:
        self._state.temperature = msg.data[self._joint_id]

    def _can_cycle_cb(self, msg: Float32MultiArray) -> None:
        self._state.can_cycle_freq = msg.data[self._joint_id]
