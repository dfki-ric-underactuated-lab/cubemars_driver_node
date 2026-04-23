"""Manages lifecycle state transitions for a ROS 2 managed node."""

from __future__ import annotations

import logging
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState


class LifecycleNode:
    """
    Controls the lifecycle of a ROS 2 managed node.

    Wraps the ``change_state`` and ``get_state`` services to provide a
    simple polling interface for driving a node through its lifecycle
    (unconfigured → inactive → active).
    """

    def __init__(self, target_node: str = "/cubemars_hardware_node") -> None:
        self._node = Node("cubemars_motor_lifecycle_node")
        self._target_node = target_node

        callback_group = MutuallyExclusiveCallbackGroup()

        self._change_state_client = self._node.create_client(
            ChangeState,
            f"{target_node}/change_state",
            callback_group=callback_group,
        )
        self._get_state_client = self._node.create_client(
            GetState,
            f"{target_node}/get_state",
            callback_group=callback_group,
        )

        self._wait_for_services()
        self.bring_to_state("inactive")
        logging.info(f"Configured {target_node}")

    def _wait_for_services(self) -> None:
        logging.info("Waiting for lifecycle services…")
        self._change_state_client.wait_for_service()
        self._get_state_client.wait_for_service()

    def _get_state(self) -> str:
        req = GetState.Request()
        future = self._get_state_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().current_state.label

    def _change_state(self, transition: int) -> bool:
        req = ChangeState.Request()
        req.transition.id = transition
        future = self._change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().success

    def bring_to_state(self, target: str) -> bool:
        """Poll-drive the hardware node to *target* (e.g. ``"inactive"``, ``"active"``)."""
        while rclpy.ok():
            state = self._get_state()
            if state == target:
                logging.info(f"Lifecycle state: {state}")
                return True
            elif state == "unconfigured":
                self._change_state(Transition.TRANSITION_CONFIGURE)
            elif state == "inactive":
                self._change_state(Transition.TRANSITION_ACTIVATE)
            elif state == "active":
                self._change_state(Transition.TRANSITION_DEACTIVATE)
            else:
                logging.warning(f"Unknown lifecycle state: {state}")
                time.sleep(0.2)
        return False
