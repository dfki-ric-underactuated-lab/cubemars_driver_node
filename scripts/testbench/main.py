#!/usr/bin/env python3

import os
import sys
import time
import math
import signal
import logging
import numpy as np
import threading
from itertools import product

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from robot_control_msgs.msg import JointCommand, JointState
from std_msgs.msg import Float32MultiArray

from csv_logger import CsvLogger
from lifecycle_node import LifecycleNode


from plotting import plot_chirp, plot_ramp, plot_trajectory

from trajectories import generate_chirp, generate_ramp

# ================= CONFIG =================
JOINT_ID = 0
KD = 1.0

PID_FILE = "/tmp/hilscher.pid"
LOGDIR = "/home/testbench/mtb-data"

COMMAND_FREQ = 1000.
SECS_PER_STEP = (1 / COMMAND_FREQ)
MAX_TORQUE = 10.0

# -------- Chirp --------
CHIRP_TEST=True
LOGARITHMIC_CHIRP = True
CHIRP_START_FREQ = 0.1            # Hz
CHIRP_END_FREQ = 100.0            # Hz
CHIRP_DURATION = 10.0             # seconds
CHIRP_REPEAT = 1       # chirps per torque level
CHIRP_NAME = f"CHIRP_{MAX_TORQUE}Nm_{CHIRP_START_FREQ}Hz_{CHIRP_END_FREQ}Hz_{CHIRP_DURATION}s_{CHIRP_REPEAT}"

# ------------ Ramp ---------

MIN_TORQUE = 0.0
NUM_REF_TORQUE_STEPS = 10 # Two is minimum to go from 0 to max and back
SECS_PER_TORQUE_STEP = 300.
TORQUE_RAMP_REPEAT = 1
SINGLE_STEP_RAMP_RATE_NM_PER_S = 1000.0

MIN_RPM = 0.0
MAX_RPM = 10.0
NUM_REF_VEL_STEPS = 1
SECS_PER_VEL_STEP = 30.0
VEL_RAMP_REPEAT = 1
SINGLE_STEP_RAMP_RATE_RPM_PER_S = 1000.0
RAMP_NAME = f"_TORQUE_{MAX_TORQUE}_{NUM_REF_TORQUE_STEPS}_VEL_{MAX_RPM}_{NUM_REF_VEL_STEPS}_{SECS_PER_STEP}"

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s - %(levelname)s - %(message)s")

global_interrupted = False
global_run = False


# ================= UTIL =================

def rpm_to_rad_s(rpm):
    return rpm * (2 * math.pi / 60.0)
MIN_VELOCITY = (rpm_to_rad_s(MIN_RPM))
MAX_VELOCITY = (rpm_to_rad_s(MAX_RPM))


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
        controller.start_logging()

    elif sig == signal.SIGRTMIN + 1:
        controller.stop_logging()


for s in (signal.SIGINT, signal.SIGTERM,
          signal.SIGUSR1, signal.SIGUSR2,
          signal.SIGRTMIN, signal.SIGRTMIN + 1):
    signal.signal(s, signal_handler)


# ================= CONTROLLER =================

class CubemarsController:

    def __init__(self):
        self.cmd_logger = None
        self.state_logger = None

        self.node = Node("CubemarsMotorTestNode")
        self.target_node="/cubemars_hardware_node"

        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)

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
        self.timer = self.node.create_timer(1. / COMMAND_FREQ, self._timer_cb)

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
        logging.info("ROS2 Node created")

        self.spin_thread = threading.Thread(target=self.executor.spin)
        self.spin_thread.start()

        self.lifecycle_node = LifecycleNode()

        self.traj_step = 0
        if CHIRP_TEST:
            self.torque_traj = generate_chirp(
                MAX_TORQUE,
                CHIRP_START_FREQ,
                CHIRP_END_FREQ,
                CHIRP_DURATION,
                COMMAND_FREQ,
                LOGARITHMIC_CHIRP,
            )
            self.vel_traj = np.zeros(len(self.torque_traj))
        else:
            self.vel_traj, self.torque_traj = generate_ramp(
                MIN_VELOCITY, MAX_VELOCITY, NUM_REF_VEL_STEPS,
                MIN_TORQUE, MAX_TORQUE, NUM_REF_TORQUE_STEPS
            )
        
        plot_trajectory(self.torque_traj, self.vel_traj, SECS_PER_STEP)

    def _state_cb(self, msg):
        self.state_msg = msg
        if self.state_logger:
            self.state_logger.log([
                    f"{time.time():.6f}",
                    f"{self.state_msg.effort[JOINT_ID] if self.state_msg else math.nan}",
                    f"{rad_s_to_rpm(self.state_msg.velocity[JOINT_ID]) if self.state_msg else math.nan}",
                    f"{self.state_msg.position[JOINT_ID] if self.state_msg else math.nan}",
                    f"{self.temp if self.temp is not None else -1.0:.2f}",
                    f"{self.can_cycle if self.can_cycle is not None else -1.0:.2f}",
            ])

    def _temp_cb(self, msg):
        self.temp = msg.data[JOINT_ID]

    def _can_cycle_cb(self, msg):
        self.can_cycle = msg.data[JOINT_ID]

    def _timer_cb(self):
        if global_interrupted:
            self.get_logger().info("Stopping control loop")
            self.timer.cancel()
            return
        self._traj_step()
        self.publisher.publish(self.cmd_msg)

        if self.cmd_logger:
            self.cmd_logger.log([
                    f"{time.time():.6f}",
                    f"{self.cmd_msg.effort[JOINT_ID]:.6f}",
                    f"{rad_s_to_rpm(self.cmd_msg.velocity[JOINT_ID]):.6f}",
                    f"{self.cmd_msg.position[JOINT_ID]:.6f}",
                    f"{self.cmd_msg.kp[JOINT_ID]:.6f}",
                    f"{self.cmd_msg.kd[JOINT_ID]:.6f}",
            ])

    # ================= LOGGING =================

    def start_logging(self):
        t = time.localtime()
        self.name = time.strftime("%Y%m%d_%H%M%S", t)
        if CHIRP_TEST:
            cmd_path = f"{LOGDIR}/{self.name}_Cubemars_{CHIRP_NAME}_COMMAND.csv"
            state_path = f"{LOGDIR}/{self.name}_Cubemars_{CHIRP_NAME}_STATE.csv"
        else:
            cmd_path = f"{LOGDIR}/{self.name}_Cubemars_{RAMP_NAME}_COMMAND.csv"
            state_path = f"{LOGDIR}/{self.name}_Cubemars_{RAMP_NAME}_STATE.csv"

        self.cmd_logger = CsvLogger(filename=cmd_path, header=["Time", 
                                                              "Torque (Nm)", 
                                                              "Velocity (RPM)", 
                                                              "Position (rad)", 
                                                              "KP", 
                                                              "KD"])
        
        self.state_logger = CsvLogger(filename=state_path, header=["Time", 
                                                              "Torque (Nm)",
                                                              "Velocity (RPM)", 
                                                              "Position (rad)", 
                                                              "Temp (°C)",
                                                              "Can Cycle (Hz)"])
        
        logging.info(f"Logging started → {LOGDIR}/{self.name}")

    def stop_logging(self):
        if self.cmd_logger:
            self.cmd_logger.close()
        if self.state_logger:
            self.state_logger.close()
        self.cmd_logger = None
        self.state_logger = None
        logging.info("Logging stopped")

    # ================= TRAJECTORY =================

    def _traj_step(self):
        global global_run
        if not global_run:
            zero_command(self.cmd_msg)
            self.state = "IDLE"
            return

        if not hasattr(self, "state") or self.state == "IDLE":
            self.state = "INIT"

        if self.state == "INIT":
            self.chirp_cycle = 0
            self.traj_step = 0
            self.state = "ACTIVE"
            logging.info("Starting sequence")

            self.lifecycle_node.bring_to_state("active")
            return
        
        if self.state == "DONE":
            meas_file = f"{LOGDIR}/{self.name}_HilsherData.csv"
            if CHIRP_TEST:
                cmd_file = f"{LOGDIR}/{self.name}_Cubemars_{CHIRP_NAME}.csv"
                plot_chirp(f, cmd_file, meas_file, CHIRP_END_FREQ, CHIRP_NAME)
            else:
                cmd_file = f"{LOGDIR}/{self.name}_Cubemars_{RAMP_NAME}.csv"
                plot_ramp(cmd_file, meas_file, RAMP_NAME)

            self.state = "IDLE"
            global_run = False
            
            return

        if self.state == "ACTIVE":
            if self.traj_step >= len(self.torque_traj):
                self.chirp_cycle += 1
                if self.chirp_cycle >= CHIRP_REPEAT:
                    logging.info("Full torque-chirp test complete")
                    self.state = "DONE"
                    zero_command(self.cmd_msg)
                    self.lifecycle_node.bring_to_state("inactive")
                    return


            self.cmd_msg.kp[JOINT_ID] = 0.0
            self.cmd_msg.kd[JOINT_ID] = 0.0
            self.cmd_msg.effort[JOINT_ID] = self.torque_traj[self.traj_step]
            self.cmd_msg.velocity[JOINT_ID] = self.vel_traj[self.traj_step]
            self.traj_step += 1
            return



def print_chirp_configuration():
    logging.info("========== USER CONFIGURATION ==========")
    # Chirp info
    logging.info("Chirp Test:")
    logging.info(f"  Max torque (MAX_TORQUE): {MAX_TORQUE} Nm")
    logging.info(f"  Chirp start frequency (CHIRP_START_FREQ): {CHIRP_START_FREQ} Hz")
    logging.info(f"  Chirp end frequency (CHIRP_END_FREQ): {CHIRP_END_FREQ} Hz")
    logging.info(f"  Chirp duration (CHIRP_DURATION): {CHIRP_DURATION} s")
    logging.info(f"  Repeats per torque step (CHIRP_REPEAT): {CHIRP_REPEAT}")

    logging.info("KD (derivative gain): {:.2f}".format(KD))
    logging.info("========================================")

# ================= MAIN =================

if __name__ == "__main__":
    with open(PID_FILE, "w") as f:
        f.write(str(os.getpid()))
    print_chirp_configuration()

    rclpy.init()
    controller = CubemarsController()
    try:
        while not global_interrupted:
            time.sleep(0.1)
    finally:
        if os.path.exists(PID_FILE):
            os.remove(PID_FILE)
        rclpy.shutdown()
        logging.info("Exited cleanly")