#!/usr/bin/env python3

import os
import sys
import time
import math
import signal
import logging
import csv
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


# ================= CONFIG =================

JOINT_ID = 0
KD = 5.

PID_FILE = "/tmp/hilscher.pid"
LOGDIR = "/home/testbench/mtb-data"

TORQUE_RAMP = True
MAX_TORQUE = 3.0
NUM_REF_TORQUE_STEPS = 1
SECS_PER_TORQUE_STEP = 300.
TORQUE_RAMP_REPEAT = 1
SINGLE_STEP_RAMP_RATE_NM_PER_S = 1000.0
TORQUE_NAME = f"_TORQUE_RAMP_{MAX_TORQUE}_{NUM_REF_TORQUE_STEPS}_{SECS_PER_TORQUE_STEP}"


VEL_RAMP = False
MAX_RPM = 10.0
NUM_REF_VEL_STEPS = 1
SECS_PER_VEL_STEP = 300.0
VEL_RAMP_REPEAT = 1
SINGLE_STEP_RAMP_RATE_RPM_PER_S = 1000.0
VEL_NAME = f"_VEL_RAMP_{MAX_RPM}_{NUM_REF_VEL_STEPS}_{SECS_PER_VEL_STEP}"

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s - %(levelname)s - %(message)s")

global_interrupted = False
global_run = False


# ================= UTIL =================

def rpm_to_rad_s(rpm):
    return rpm * (2 * math.pi / 60.0)


def rad_s_to_rpm(rad_s):
    return rad_s * (60.0 / (2 * math.pi))


def zero_command(cmd):
    cmd.position[JOINT_ID] = 0.0
    cmd.velocity[JOINT_ID] = 0.0
    cmd.acceleration[JOINT_ID] = 0.0
    cmd.kp[JOINT_ID] = 0.0
    cmd.kd[JOINT_ID] = KD # If KD = 0. motor accelerates
    cmd.effort[JOINT_ID] = 0.0


# ================= SIGNALS =================

def signal_handler(sig, frame):
    global global_interrupted, global_run

    if sig == signal.SIGUSR1:
        global_run = True
        logging.info("SIGUSR1 → RUN")

    elif sig == signal.SIGUSR2:
        global_run = False
        logging.info("SIGUSR2 → PAUSE")

    elif sig in (signal.SIGINT, signal.SIGTERM):
        global_interrupted = True
        logging.warning("Shutdown requested")

    elif sig == signal.SIGRTMIN:
        controller.start_logging() # TODO: Why extra signal for logging and not start with RUN?

    elif sig == signal.SIGRTMIN + 1:
        controller.stop_logging()


for s in (signal.SIGINT, signal.SIGTERM,
          signal.SIGUSR1, signal.SIGUSR2,
          signal.SIGRTMIN, signal.SIGRTMIN + 1):
    signal.signal(s, signal_handler)


# ================= CONTROLLER =================

class CubemarsController:

    def __init__(self):
        self.node = Node("CubemarsMotorTestNode")

        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)

        # ---- Lifecycle clients ----
        self.target_node = "/cubemars_hardware_node"

        service_group = MutuallyExclusiveCallbackGroup()

        self.change_state_client = self.node.create_client(
            ChangeState,
            f"{self.target_node}/change_state",
            callback_group=service_group)

        self.get_state_client = self.node.create_client(
            GetState,
            f"{self.target_node}/get_state",
            callback_group=service_group)

        self._wait_for_services()

        # ---- Publisher (must run continuously) ----
        self.cmd_msg = JointCommand()
        self.cmd_msg.position = [0.0]
        self.cmd_msg.velocity = [0.0]
        self.cmd_msg.acceleration = [0.0]
        self.cmd_msg.kp = [0.0]
        self.cmd_msg.kd = [KD]
        self.cmd_msg.effort = [0.0]

        self.publisher = self.node.create_publisher(
            JointCommand,
            f"{self.target_node}/joint_commands",
            1)

        # 2ms timer → ensures publishing during lifecycle transitions
        self.node.create_timer(0.002, self._publish_cmd)

        # ---- Subscribers ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.state_msg = None
        self.temp = None
        self.can_cycle = None

        cb_group = ReentrantCallbackGroup()

        self.node.create_subscription(
            JointState,
            f"{self.target_node}/joint_states",
            self._state_cb,
            qos,
            callback_group=cb_group)

        self.node.create_subscription(
            Float32MultiArray,
            f"/joint_temperatures",
            self._temp_cb,
            qos,
            callback_group=cb_group)

        self.node.create_subscription(
            Float32MultiArray,
            f"/can_cycle_frequencies",
            self._can_cycle_cb,
            qos,
            callback_group=cb_group)

        # ---- Trajectory state ----
        self.ref_vels_rpm = []
        self.ref_torque = []
        self.vel_idx = 0
        self.torque_idx = 0
        self.vel_dir = 1
        self.torque_dir = 1
        self.t_step_start = time.monotonic()
        self.cycle_count = 0
        self.was_running = False

        # ---- Logging ----
        self.data_file = None
        self.csv_writer = None
        self.logging_enabled = False

        # ---- Spin thread (kept from original) ----
        self.spin_thread = threading.Thread(target=self.executor.spin)
        self.spin_thread.start()

        logging.info("ROS2 Node created")

    # ================= ROS =================

    def _wait_for_services(self):
        logging.info("Waiting for lifecycle services...")
        self.change_state_client.wait_for_service()
        self.get_state_client.wait_for_service()

    def _get_state(self):
        req = GetState.Request()
        future = self.get_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().current_state.label

    def _change_state(self, transition):
        req = ChangeState.Request()
        req.transition.id = transition
        future = self.change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().success

    def bringup_to_active(self):
        while rclpy.ok():
            state = self._get_state()
            if state == "active":
                logging.info(f"Lifecycle state: {state}")
                return True
            if state == "unconfigured":
                self._change_state(Transition.TRANSITION_CONFIGURE)
            elif state == "inactive":
                self._change_state(Transition.TRANSITION_ACTIVATE)
            else:
                time.sleep(0.2)

    # ================= CALLBACKS =================

    def _state_cb(self, msg):
        self.state_msg = msg

    def _temp_cb(self, msg):
        self.temp = msg.data[JOINT_ID]

    def _can_cycle_cb(self, msg):
        self.can_cycle = msg.data[JOINT_ID]

    def _publish_cmd(self):
        self.publisher.publish(self.cmd_msg)

    # ================= PROFILE =================

    def _setup_profile(self):
        self.ref_torque.clear()
        if TORQUE_RAMP:
            if NUM_REF_TORQUE_STEPS <= 1:
                self.ref_torque = [0.0, MAX_TORQUE]
            else:
                for i in range(NUM_REF_TORQUE_STEPS + 1):
                    self.ref_torque.append(i * MAX_TORQUE / NUM_REF_TORQUE_STEPS)

            logging.info(f"Torque profile generated: {self.ref_torque}")

        self.ref_vels_rpm.clear()
        if VEL_RAMP:
            if NUM_REF_VEL_STEPS <= 1:
                self.ref_vels_rpm = [0.0, MAX_RPM]
            else:
                for i in range(NUM_REF_VEL_STEPS + 1):
                    self.ref_vels_rpm.append(i * MAX_RPM / NUM_REF_VEL_STEPS)

            logging.info(f"Velocity profile generated: {self.ref_vels_rpm}")

    # ================= LOGGING =================

    def start_logging(self):
        if self.data_file:
            return
        t = time.localtime()
        name = time.strftime("%Y%m%d_%H%M%S", t)
        path = f"{LOGDIR}/{name}_Cubemars{TORQUE_NAME if TORQUE_RAMP else ""}{VEL_NAME if VEL_RAMP else ""}.csv"
        self.data_file = open(path, "w", newline="")
        self.csv_writer = csv.writer(self.data_file)
        self.csv_writer.writerow(["Time", 
                                  "Torque",
                                  "Speed", 
                                  "Position", 
                                  "Temp", 
                                  "TorqueCmd", 
                                  "VelCmd", 
                                  "PosCmd", 
                                  "KP", 
                                  "KD",
                                  "CanCycleFreq"])
        self.data_file.flush()
        logging.info(f"Logging started → {path}")

    def stop_logging(self):
        if self.data_file:
            self.data_file.close()
            self.data_file = None
            self.csv_writer = None
            logging.info("Logging stopped")
    
    def _log(self):
        if not self.csv_writer:
            return
        if not self.state_msg:
            return

        try:
            self.csv_writer.writerow([
                f"{time.time():.6f}",
                f"{self.state_msg.effort[JOINT_ID]:.6f if self.state_msg else math.nan}",
                f"{rad_s_to_rpm(self.state_msg.velocity[JOINT_ID]):.6f if self.state_msg else math.nan}",
                f"{self.state_msg.position[JOINT_ID]:.6f if self.state_msg else math.nan}",
                f"{self.temp if self.temp is not None else -1.0:.2f}",
                f"{self.cmd_msg.effort[JOINT_ID]:.6f}",
                f"{rad_s_to_rpm(self.cmd_msg.velocity[JOINT_ID]):.6f}",
                f"{self.cmd_msg.position[JOINT_ID]:.6f}",
                f"{self.cmd_msg.kp[JOINT_ID]:.6f}",
                f"{self.cmd_msg.kd[JOINT_ID]:.6f}",
                f"{self.can_cycle if self.can_cycle is not None else -1.0:.2f}",
            ])
            self.data_file.flush()
        except Exception as e:
            logging.warning(f"CSV write failed: {e}")

    # ================= TRAJECTORY =================

    def _traj_step(self):
        global global_run

        now = time.monotonic()

        if not global_run:
            zero_command(self.cmd_msg)
            self.was_running = False
            return

        if TORQUE_RAMP and NUM_REF_TORQUE_STEPS == 1:
            if not self.was_running:
                self.was_running = True
                self.t_step_start = now
                logging.info("Single-step started")

            elapsed = now - self.t_step_start
            if elapsed >= SECS_PER_TORQUE_STEP:
                global_run = False
                zero_command(self.cmd_msg)
                logging.info("Single-step finished")
                return

            if SINGLE_STEP_RAMP_RATE_NM_PER_S > 0:
                ramp = SINGLE_STEP_RAMP_RATE_NM_PER_S
                ramp_time = MAX_TORQUE / ramp
                if elapsed < ramp_time:
                    torque = ramp * elapsed
                elif elapsed > SECS_PER_TORQUE_STEP - ramp_time:
                    torque = ramp * (SECS_PER_TORQUE_STEP - elapsed)
                else:
                    torque = MAX_TORQUE
            else:
                torque = MAX_TORQUE

            self.cmd_msg.effort[0] = torque
            return

        if VEL_RAMP and NUM_REF_VEL_STEPS == 1:
            if not self.was_running:
                self.was_running = True
                self.t_step_start = now
                logging.info("Single-step started")

            elapsed = now - self.t_step_start
            if elapsed >= SECS_PER_VEL_STEP:
                global_run = False
                zero_command(self.cmd_msg)
                logging.info("Single-step finished")
                return

            if SINGLE_STEP_RAMP_RATE_RPM_PER_S > 0:
                ramp = SINGLE_STEP_RAMP_RATE_RPM_PER_S
                ramp_time = MAX_RPM / ramp
                if elapsed < ramp_time:
                    rpm = ramp * elapsed
                elif elapsed > SECS_PER_VEL_STEP - ramp_time:
                    rpm = ramp * (SECS_PER_VEL_STEP - elapsed)
                else:
                    rpm = MAX_RPM
            else:
                rpm = MAX_RPM

            self.cmd_msg.velocity[0] = rpm_to_rad_s(rpm)
            self.cmd_msg.kd[0] = KD
            return

        # ----- Multi-step -----
        if not self.was_running:
            self.was_running = True
            self.vel_idx = 0
            self.torque_idx = 0
            self.vel_dir = 1
            self.torque_dir = 1
            self.t_step_start = now
            self.cycle_count = 0
            logging.info("Ramp sequence started")

        
        if TORQUE_RAMP:
            if now - self.t_step_start >= SECS_PER_TORQUE_STEP:
                self.torque_idx += self.torque_dir

                if self.torque_idx >= len(self.ref_torque):
                    self.torque_idx = len(self.ref_torque) - 2
                    self.torque_dir = -1
                elif self.torque_idx < 0:
                    self.torque_idx = 0
                    self.torque_dir = 1
                    self.cycle_count += 1
                    logging.info(f"Cycle {self.cycle_count} complete")

                    if TORQUE_RAMP_REPEAT and self.cycle_count >= TORQUE_RAMP_REPEAT:
                        global_run = False
                        zero_command(self.cmd_msg)
                        logging.info("Ramp finished → paused")
                        return

                self.t_step_start = now

            torque = self.ref_torque[self.torque_idx]
            self.cmd_msg.effort[0] = torque

        if VEL_RAMP:
            if now - self.t_step_start >= SECS_PER_VEL_STEP:
                self.vel_idx += self.vel_dir

                if self.vel_idx >= len(self.ref_vels_rpm):
                    self.vel_idx = len(self.ref_vels_rpm) - 2
                    self.vel_dir = -1
                elif self.vel_idx < 0:
                    self.vel_idx = 0
                    self.vel_dir = 1
                    self.cycle_count += 1
                    logging.info(f"Cycle {self.cycle_count} complete")

                    if VEL_RAMP_REPEAT and self.cycle_count >= VEL_RAMP_REPEAT:
                        global_run = False
                        zero_command(self.cmd_msg)
                        logging.info("Ramp finished → paused")
                        return

                self.t_step_start = now

            rpm = self.ref_vels_rpm[self.vel_idx]
            self.cmd_msg.velocity[0] = rpm_to_rad_s(rpm)
            self.cmd_msg.kd[0] = KD
        else:
            self.cmd_msg.kd[0] = 0.0

    # ================= MAIN LOOP =================

    def run(self):
        self._setup_profile()
        self.bringup_to_active()

        logging.info("Starting control loop")

        while not global_interrupted and rclpy.ok():
            rclpy.spin_once(self.node)
            self._traj_step()
            self._log()
            time.sleep(0.001)

        zero_command(self.cmd_msg)
        logging.info("Controller stopping")



def print_user_configuration():
    if TORQUE_RAMP:
        logging.info("========== USER TORQUE CONFIGURATION ==========")
        logging.info(f"Max output torque (MAX_TORQUE): {MAX_TORQUE} Nm")
        logging.info(f"Number of velocity steps (NUM_REF_TORQUE_STEPS): {NUM_REF_TORQUE_STEPS}")
        logging.info(f"Seconds per velocity step (SECS_PER_TORQUE_STEP): {SECS_PER_TORQUE_STEP} s")
        logging.info(f"Ramp repeat count (VEL_RAMP_REPEAT): {TORQUE_RAMP_REPEAT}")
        logging.info("Single-step mode parameters (if applicable):")
        logging.info(
            f"  Ramp rate (SINGLE_STEP_RAMP_RATE_RPM_PER_S): "
            f"{SINGLE_STEP_RAMP_RATE_RPM_PER_S} rpm/s"
        )
        logging.info("========================================")

    if VEL_RAMP:
        logging.info("========== USER VEL CONFIGURATION ==========")
        logging.info(f"Max output speed (MAX_RPM): {MAX_RPM} rpm")
        logging.info(f"Number of velocity steps (NUM_REF_VEL_STEPS): {NUM_REF_VEL_STEPS}")
        logging.info(f"Seconds per velocity step (SECS_PER_VEL_STEP): {SECS_PER_VEL_STEP} s")
        logging.info(f"Ramp repeat count (VEL_RAMP_REPEAT): {VEL_RAMP_REPEAT}")
        logging.info("Single-step mode parameters (if applicable):")
        logging.info(
            f"  Ramp rate (SINGLE_STEP_RAMP_RATE_RPM_PER_S): "
            f"{SINGLE_STEP_RAMP_RATE_RPM_PER_S} rpm/s"
        )
        logging.info("========================================")

# ================= MAIN =================

if __name__ == "__main__":

    with open(PID_FILE, "w") as f:
        f.write(str(os.getpid()))
    print_user_configuration()

    rclpy.init()
    controller = CubemarsController()
    try:
        controller.run()
    finally:
        if os.path.exists(PID_FILE):
            os.remove(PID_FILE)
        rclpy.shutdown()
        logging.info("Exited cleanly")
