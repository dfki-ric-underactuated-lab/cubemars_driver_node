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
KD = 5.0

PID_FILE = "/tmp/hilscher.pid"
LOGDIR = "/home/testbench/mtb-data"

# -------- Torque Steps --------
MAX_TORQUE = 10.0
NUM_REF_TORQUE_STEPS = 1          # number of torque levels
TORQUE_RAMP_REPEAT = 1            # full sweep repeats

# -------- Chirp --------
LOGARITHMIC_CHIRP = True
CHIRP_START_FREQ = 0.1            # Hz
CHIRP_END_FREQ = 10.0            # Hz
CHIRP_DURATION = 40.0             # seconds
CHIRP_REPEAT_PER_TORQUE = 1       # chirps per torque level

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
        self.ref_freq = []
        self.ref_torque = []
        self.freq_idx = 0
        self.torque_idx = 0
        self.freq_dir = 1
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
        if NUM_REF_TORQUE_STEPS <= 1:
            self.ref_torque = [MAX_TORQUE]
        else:
            self.ref_torque = [
                i * MAX_TORQUE / (NUM_REF_TORQUE_STEPS - 1)
                for i in range(NUM_REF_TORQUE_STEPS)
            ]

        logging.info(f"Torque steps: {self.ref_torque}")

    # ================= LOGGING =================

    def start_logging(self):
        if self.data_file:
            return
        name = time.strftime("%Y%m%d_%H%M%S")
        path = f"{LOGDIR}/{name}_Cubemars.csv"
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
                f"{self.state_msg.effort[JOINT_ID]:.6f}",
                f"{self.state_msg.velocity[JOINT_ID]:.6f}",
                f"{self.state_msg.position[JOINT_ID]:.6f}",
                f"{self.temp if self.temp is not None else -1.0:.2f}",
                f"{self.cmd_msg.effort[JOINT_ID]:.6f}",
                f"{self.cmd_msg.velocity[JOINT_ID]:.6f}",
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
            self.state = "IDLE"
            return

        # ---------- INIT ----------
        if not hasattr(self, "state") or self.state == "IDLE":
            self.state = "INIT"

        if self.state == "INIT":
            self.torque_idx = 0
            self.torque_cycle = 0
            self.chirp_cycle = 0
            self.t_start = now
            self.state = "SET_TORQUE"
            logging.info("Starting torque-chirp sequence")
            return

        # ---------- SET TORQUE ----------
        if self.state == "SET_TORQUE":

            self.current_torque = self.ref_torque[self.torque_idx]

            self.cmd_msg.kp[JOINT_ID] = 0.0
            self.cmd_msg.kd[JOINT_ID] = 0.0
            self.cmd_msg.effort[JOINT_ID] = self.current_torque

            self.chirp_cycle = 0
            self.t_start = now
            self.state = "CHIRP"

            logging.info(f"Torque step {self.torque_idx+1}/{len(self.ref_torque)} "
                        f"→ {self.current_torque:.2f} Nm")
            return

        # ---------- CHIRP ----------
        if self.state == "CHIRP":
            t = now - self.t_start

            # Chirp finished?
            if t >= CHIRP_DURATION:
                self.chirp_cycle += 1

                if self.chirp_cycle >= CHIRP_REPEAT_PER_TORQUE:
                    self.state = "NEXT_TORQUE"
                    return

                self.t_start = now
                t = 0.0

            # ----- Linear frequency sweep -----
            f0 = CHIRP_START_FREQ
            f1 = CHIRP_END_FREQ
            T = CHIRP_DURATION

            k = (f1 - f0) / T  # frequency slope

            if LOGARITHMIC_CHIRP:
                beta = math.log(f1 / f0) / T
                phase = 2.0 * math.pi * f0 * (math.exp(beta * t) - 1.0) / beta
            else:
                # Proper integrated phase for linear chirp:
                # φ(t) = 2π ( f0 t + 0.5 k t² )
                phase = 2.0 * math.pi * (f0 * t + 0.5 * k * t * t)

            # Symmetric torque chirp: +A to -A
            A = self.current_torque

            torque_cmd = A * math.sin(phase)

            self.cmd_msg.kp[JOINT_ID] = 0.0
            self.cmd_msg.kd[JOINT_ID] = 0.0
            self.cmd_msg.effort[JOINT_ID] = torque_cmd

            return

        # ---------- NEXT TORQUE ----------
        if self.state == "NEXT_TORQUE":

            self.torque_idx += 1

            if self.torque_idx >= len(self.ref_torque):
                self.torque_cycle += 1

                if TORQUE_RAMP_REPEAT and self.torque_cycle >= TORQUE_RAMP_REPEAT:
                    logging.info("Full torque-chirp test complete")
                    global_run = False
                    zero_command(self.cmd_msg)
                    return

                self.torque_idx = 0

            self.state = "SET_TORQUE"
            return

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
    logging.info("========== USER CONFIGURATION ==========")
    
    # Torque ramp info
    logging.info("Torque Ramp:")
    logging.info(f"  Max torque (MAX_TORQUE): {MAX_TORQUE} Nm")
    logging.info(f"  Number of torque steps (NUM_REF_TORQUE_STEPS): {NUM_REF_TORQUE_STEPS}")
    logging.info(f"  Ramp repeat count (TORQUE_RAMP_REPEAT): {TORQUE_RAMP_REPEAT}")
    
    # Chirp info
    logging.info("Chirp Test:")
    logging.info(f"  Chirp start frequency (CHIRP_START_FREQ): {CHIRP_START_FREQ} Hz")
    logging.info(f"  Chirp end frequency (CHIRP_END_FREQ): {CHIRP_END_FREQ} Hz")
    logging.info(f"  Chirp duration (CHIRP_DURATION): {CHIRP_DURATION} s")
    logging.info(f"  Repeats per torque step (CHIRP_REPEAT_PER_TORQUE): {CHIRP_REPEAT_PER_TORQUE}")
    
    logging.info("KD (derivative gain): {:.2f}".format(KD))
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
