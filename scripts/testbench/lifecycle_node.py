#!/usr/bin/env python3

import os
import sys
import time
import math
import signal
import logging
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from robot_control_msgs.msg import JointCommand, JointState
from std_msgs.msg import Float32MultiArray
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
#from bode_plot import plot_bode, load_and_align, estimate_transfer

from csv_logger import CsvLogger

class LifecycleNode:
    def __init__(self, target_node="/cubemars_hardware_node"):
        # ---- Lifecycle clients ----
        self.lifecycle_node = Node("CubemarsMotorLifecycleNode")
        self.target_node = target_node

        service_group = MutuallyExclusiveCallbackGroup()

        self.change_state_client = self.lifecycle_node.create_client(
            ChangeState,
            f"{self.target_node}/change_state",
            callback_group=service_group)

        self.get_state_client = self.lifecycle_node.create_client(
            GetState,
            f"{self.target_node}/get_state",
            callback_group=service_group)


        self._wait_for_services()

        self.bring_to_state("inactive")

        logging.info(f"Configured {self.target_node} node")

    
    def _wait_for_services(self):
        logging.info("Waiting for lifecycle services...")
        self.change_state_client.wait_for_service()
        self.get_state_client.wait_for_service()

    def _get_state(self):
        req = GetState.Request()
        future = self.get_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.lifecycle_node, future)
        return future.result().current_state.label

    def _change_state(self, transition):
        req = ChangeState.Request()
        req.transition.id = transition
        future = self.change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.lifecycle_node, future)
        return future.result().success

    def bring_to_state(self, target):
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
                logging.warn(f"Invalid lifecycle state: {state}")
                time.sleep(0.2)