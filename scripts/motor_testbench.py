#!/bin/env python3

import os
import threading
import time
import math
import signal
import sys
import logging
import csv


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from robot_control_msgs.msg import JointCommand, JointState

from std_msgs.msg import Float32MultiArray

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import DurabilityPolicy

#TODO: Change turns per second with rad/s

JOINT_ID = 0
TORQUE_CONSTANT=1.1314 # AK10-9 v3

# TODO: Gains: 100p 2d

# Used to read the PID by other programs.
PID_FILE = "/tmp/hilscher.pid"

# --- LOGGING SETUP ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- GLOBAL SIGNAL CONTROL (Replicating C++ Logic) ---
global_interrupted = 0    # only set to 1 for Ctrl+C / fatal errors -> program exits
global_run = False        # True => motor trajectory running, False => paused

def signal_handler(sig, frame):
    """Handles Unix signals (SIGINT, SIGTERM, SIGUSR1, SIGUSR2)."""
    global global_interrupted, global_run

    if sig == signal.SIGUSR2:
        # SIGUSR2 -> Pause/Stop the trajectory
        global_run = False
        logging.info("SIGUSR2 received: Trajectory Stopped.")
        return
    if sig == signal.SIGUSR1:
        # SIGUSR1 -> Start/Resume the trajectory
        global_run = True
        logging.info("SIGUSR1 received: Trajectory Running.")
        return
    if sig == signal.SIGRTMIN:
        # To start recording data into CSV file
        logging.info("SIGRTMIN received : START CSV recording.")
        controller.start_logging()
        return
    if sig == signal.SIGRTMIN + 1:
        # To stop recording data into csv file
        logging.info("SIGRTMIN+1 received : STOP CSV recording")
        controller.stop_logging()
        return

    # SIGINT (Ctrl+C) or SIGTERM -> Controlled shutdown and program exit
    global_interrupted = 1
    logging.warning(f"Signal {sig} received: Initiating controlled shutdown (program will exit).")

# Register signal handlers
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGUSR1, signal_handler)
signal.signal(signal.SIGUSR2, signal_handler)
signal.signal(signal.SIGRTMIN, signal_handler)
signal.signal(signal.SIGRTMIN + 1, signal_handler)

# --- CONSTANTS AND LIMITS ---

# def rad_s_to_turns_s(rad_s):
#     return rad_s / (2 * math.pi)

# def turns_s_to_rad_s(turns_s):
#     return turns_s * (2 * math.pi)

def rad_s_to_rpm(rad_s):
    return rad_s / ((2 * math.pi) / 60)

def rpm_to_rad_s(rpm):
    return rpm * ((2 * math.pi) / 60)
# -------------------------
# USER-CONFIGURABLE PARAMETERS
# -------------------------

# Max velocity in *output shaft rpm* 
MAX_RPM = 30.0

# Number of steps for the ramp.
# If NUM_REF_VEL_STEPS == 1: "single-value" mode (one speed, one hold time)
# If NUM_REF_VEL_STEPS > 1: stepped ramp 0 -> max -> 0
NUM_REF_VEL_STEPS = 1

# How long each step is held (seconds).
# For single-value mode, this is the hold time at the single speed.
SECS_PER_VEL_STEP = 10.0

# How many full up+down cycles to run (multi-step mode only).
# 0 means "run forever until SIGUSR2".
RAMP_REPEAT = 1

# SINGLE-VALUE mode ramp parameters (used only when NUM_REF_VEL_STEPS == 1)
# Define threshold in OUTPUT rpm (user-facing)
# Single-step ramp rate (OUTPUT rpm per second)
# Example:
#   1000.0 -> 1 rpm per 1 ms (Ramp Time = Max Rpm / Ramp rate) (0.03s = 30rpm / 1000 rpm/s)
#   0.0    -> no ramp (step input)
SINGLE_STEP_RAMP_RATE_RPM_PER_S = 1000.0


# -------------------- END OF USER-CONFIGURABLE PARAMETERS --------------------------------------------
REF_VEL_MAX_RAD_S = rpm_to_rad_s(MAX_RPM)

# Controller Limits (in motor turns/s)
VEL_MAX_RAD_S = REF_VEL_MAX_RAD_S
CUR_MAX_AMPS = 9.0

# Function to print user defined variables in the terminal
def print_user_configuration():
    logging.info("========== USER CONFIGURATION ==========")
    logging.info(f"Max output speed (MAX_RPM): {MAX_RPM} rpm")
    logging.info(f"Number of velocity steps (NUM_REF_VEL_STEPS): {NUM_REF_VEL_STEPS}")
    logging.info(f"Seconds per velocity step (SECS_PER_VEL_STEP): {SECS_PER_VEL_STEP} s")
    logging.info(f"Ramp repeat count (RAMP_REPEAT): {RAMP_REPEAT}")

    logging.info("Single-step mode parameters (if applicable):")
    logging.info(f"  Ramp rate (SINGLE_STEP_RAMP_RATE_RPM_PER_S): {SINGLE_STEP_RAMP_RATE_RPM_PER_S} rpm/s")
    logging.info("========================================")

def validate_single_step_ramp():
    if NUM_REF_VEL_STEPS != 1:
        return  # Only applies to single-step mode

    if SINGLE_STEP_RAMP_RATE_RPM_PER_S <= 0.0:
        return  # Ramp disabled ? always valid

    ramp_time = MAX_RPM / SINGLE_STEP_RAMP_RATE_RPM_PER_S
    total_ramp_time = 2.0 * ramp_time

    if total_ramp_time > SECS_PER_VEL_STEP:
        logging.error("INVALID SINGLE-STEP RAMP CONFIGURATION")
        logging.error(
            f"Ramp-up + ramp-down time ({total_ramp_time:.3f}s) "
            f"exceeds execution time SECS_PER_VEL_STEP ({SECS_PER_VEL_STEP:.3f}s)."
        )
        sys.exit(1)

# Safety / logging
MIN_RAMP_RATE = 0.01   # minimum vel_ramp_rate (turns/s^2) when we compute ramps

# --- STATE ENUMS (Matching C++ mystate) ---
class MyState:
    WAIT_CONF = 0
    PREPARE = 1
    WAIT_EN = 2
    TRAJ = 3
    END = 4

class CubemarsController():
    """Manages configuration, live control, and trajectory execution."""
    def __init__(self, drive_id=None):
        self.node = Node('CubemarsMotorTestNode')
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)

        self.target_node = "/cubemars_hardware_node"
        service_callback_group = MutuallyExclusiveCallbackGroup()
        self.change_state_client = self.node.create_client(
            ChangeState,
            f'{self.target_node}/change_state',
            callback_group=service_callback_group
        )

        self.get_state_client = self.node.create_client(
            GetState,
            f'{self.target_node}/get_state',
            callback_group=service_callback_group
        )
        
        self.timer = self.node.create_timer(0.002, self.timer_callback)

        self._wait_for_services()

        self.lock_state = threading.Lock()
        self.lock_temp = threading.Lock()
        self.latest_state_msg = None
        self.state_msg = None
        self.lastest_temp = None
        self.temp = None

        self.cmd_msg = JointCommand()

        self.cmd_msg.position = [0.]
        self.cmd_msg.kp = [0.]
        self.cmd_msg.velocity = [0.]
        self.cmd_msg.acceleration = [0.]
        self.cmd_msg.kd = [0.]
        self.cmd_msg.effort = [0.]
        
        def state_callback(msg):
            with self.lock_state:
                self.latest_state_msg = msg
        def temp_callback(msg):
            with self.lock_temp:
                self.lastest_temp = msg[JOINT_ID]

        self.publisher_ = self.node.create_publisher(
            JointCommand,
            f'{self.target_node}/joint_commands',
            1)
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,   # <-- RELIABILITY_QOS_POLICY
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.cb_group = ReentrantCallbackGroup()
        self.subscription = self.node.create_subscription(
            JointState,
            "/cubemars_hardware_node/joint_states",
            state_callback,
            qos_profile,
            callback_group=self.cb_group)

        self.subscription = self.node.create_subscription(
            Float32MultiArray,
            f'{self.target_node}/joint_temperatures',
            temp_callback,
            1,
            callback_group=self.cb_group)
        
        self.subscription  # prevent unused variable warning

        def executor_loop():
            self.executor.spin()
        
        self.thread = threading.Thread(target = executor_loop)

        self.state = MyState.WAIT_CONF
        self.my_drive = None
        self.drive_id = drive_id

        # Trajectory variables (make it rpm)
        self.ref_vels_rpm = []
        self.idx_ref_vel = 0
        self.idx_ref_vel_dir = 1
        self.t_step_start = time.monotonic()

        # Logging
        self.data_file = None
        self.t_start = time.monotonic()
        self.logging_enabled = False    # logging starts only after first SIGUSR1

        # Runtime helpers for repeats & single-step behavior
        self._cycle_count = 0           # multi-step cycles completed
        self._was_running = False       # edge detection for global_run (SIGUSR1)

        # Single-step (NUM_REF_VEL_STEPS == 1) helper state
        self._single_phase = "idle"     # "idle", "ramp_up", "hold"
        self._single_start_time = 0.0   # phase start time

        self.node.get_logger().info('ROS2 Node Creation completed')

    def timer_callback(self):
        # print("test")
        self.publisher_.publish(self.cmd_msg)


    # ---------------- Lifecycle stuff -----------------
    def _wait_for_services(self):
        self.node.get_logger().info('Waiting for lifecycle services...')
        self.change_state_client.wait_for_service()
        self.get_state_client.wait_for_service()

    def _get_state(self):
        request = GetState.Request()
        future = self.get_state_client.call_async(request)
        # self.executor.spin_until_future_complete(future)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().current_state.label

    def _change_state(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id

        future = self.change_state_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        #self.executor.spin_until_future_complete(future)

        return future.result().success
    
    def bringup_to_state(self, until: str="inactive"):
        while rclpy.ok():
            state = self._get_state()
            self.node.get_logger().info(f'Current state: {state}')
            if state == "active":
                return True
            if until == state:
                return True
            if state == 'unconfigured':
                self.node.get_logger().info('Configuring...')
                if not self._change_state(Transition.TRANSITION_CONFIGURE):
                    self.node.get_logger().error('Failed to configure')
                    return False

            elif state == 'inactive':
                self.node.get_logger().info('Activating...')
                if not self._change_state(Transition.TRANSITION_ACTIVATE):
                    self.node.get_logger().error('Failed to activate')
                    return False

            elif state == 'active':
                self.node.get_logger().info('Cubemars Controller is active')
                if until == 'unconfigured':
                    if not self._change_state(Transition.TRANSITION_DEACTIVATE):
                        self.node.get_logger().error('Failed to deactivate')
                        return False
                else:
                    print("test")
                    return True

            elif state == 'finalized':
                self.node.get_logger().error('Cubemars Controller  is finalized')
                return False

            else:
                # configuring / activating / deactivating / error_processing
                self.node.get_logger().info('Waiting for transition...')
                rclpy.spin_once(self, timeout_sec=0.2)

    # ---------------- Trajectory setup ----------------

    def _setup_velocity_profile(self): 
        """Pre-calculates the vector of velocity steps (in RPM)."""
        self.ref_vels_rpm.clear()
        if NUM_REF_VEL_STEPS <= 0:
            self.ref_vels_rpm.append(0.0)

        elif NUM_REF_VEL_STEPS == 1:
            self.ref_vels_rpm.append(0.0)
            self.ref_vels_rpm.append(MAX_RPM)
        else:
            for i in range(NUM_REF_VEL_STEPS + 1):
                vel_rpm = i * MAX_RPM / NUM_REF_VEL_STEPS
                self.ref_vels_rpm.append(vel_rpm)

        logging.info(
            f"Generated {len(self.ref_vels_rpm)-1} velocity steps "
            f"(Max output: {MAX_RPM:.2f} rpm)."
        )

    def _check_errors_and_warnings(self):
        # Currently disabled; you can add real error checks later.
        return False

    # ---------------- CSV logging ----------------
    def start_logging(self):
        if self.data_file is not None:
            return  # already logging

        start_label = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        filename = f"/home/testbench/mtb-data/{start_label}_Cubemars_AK-10-9-V3.csv"

        self.data_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.data_file)

        self.csv_writer.writerow([
            "Time",
            "Torque",
            "Speed",
            "Current",
            "Temperature",
            "Ref_Torque",
            "Ref_Velocity"
        ])
        self.data_file.flush()

        logging.info(f"CSV recording STARTED: {filename}")
        self.logging_enabled = True


    def stop_logging(self):
        if self.data_file:
            self.data_file.flush()
            self.data_file.close()
            self.data_file = None
            self.csv_writer = None

            logging.info("CSV recording STOPPED (file closed)")

        self.logging_enabled = False

    def _log_data(self, cmd_vel_rpm_out, measured_pos, measured_vel_rpm_out,
                  tau_meas_Nm, Iq_measured,  temp_fet, temp_motor):
        if self.csv_writer is None:
            return
        
        # print(cmd_vel_rpm_out)
        # print(measured_vel_rpm_out)
        # print(tau_meas_Nm)
        # print(Iq_measured)
        # print(temp_motor)
        temp_motor = 0.

        t_unix = time.time()

        self.csv_writer.writerow([
            f"{t_unix:.6f}",
            f"{tau_meas_Nm:.3f}",
            f"{measured_vel_rpm_out:.3f}",
            f"{Iq_measured:.3f}",
            f"{temp_motor:.1f}",
            f"{0.0:.3f}",
            f"{cmd_vel_rpm_out:.3f}"
        ])
        self.data_file.flush()

    # ---------------- Main control loop ----------------

    def run_controller(self):
        print("test0")
        # self.thread.start()
        global global_interrupted, global_run
        print("test0")
        self._setup_velocity_profile()
        print("test0")

        # if not self._bringup_to_state():
        #     logging.critical("Initial configuration failed. Exiting.")
        #     sys.exit(1)

        self.t_start = time.monotonic()
        logging.info("--- PHASE 3: Starting Control Loop ---")
        # self.thread.join()
        while global_interrupted == 0:
            rclpy.spin_once(controller.node)
            try:
                # Threadsafe update of state and temp
                with self.lock_state:
                    self.state_msg = self.latest_state_msg
                with self.lock_temp:
                    self.temp = self.lastest_temp
                    
                if self._check_errors_and_warnings():
                    self.state = MyState.END

                if self.state == MyState.WAIT_CONF:
                    if self.bringup_to_state("active"):
                        self.state = MyState.PREPARE
                        logging.info("State change: WAIT_CONF -> PREPARE")

                elif self.state == MyState.PREPARE:
                    self.t_step_start = time.monotonic()
                    self.state = MyState.WAIT_EN
                    logging.info("State change: PREPARE -> WAIT_EN (Commanding CLOSED_LOOP_CONTROL)")

                    # To have a dry test run without having the motor run
                    '''logging.warning("TEST MODE: Skipping CLOSED_LOOP_CONTROL (motor stays IDLE)")
                    self.t_step_start = time.monotonic()
                    self.state = MyState.TRAJ'''

                elif self.state == MyState.WAIT_EN:

                    print_user_configuration()
                    validate_single_step_ramp()
                    if self.bringup_to_state("active"):
                        self.cmd_msg.position[JOINT_ID] = 0.
                        self.cmd_msg.kp[JOINT_ID] = 0.

                        self.cmd_msg.velocity[JOINT_ID] = 0.
                        self.cmd_msg.acceleration[JOINT_ID] = 0.
                        self.cmd_msg.kd[JOINT_ID] = 0.
                        self.cmd_msg.effort[JOINT_ID] = 0.

                        self.t_step_start = time.monotonic()
                        self.state = MyState.TRAJ
                        logging.info("State change: WAIT_EN -> TRAJ. Motor enabled.")
                    elif time.monotonic() - self.t_step_start > 5.0:
                        logging.error("Failed to enable motor within 5 seconds.")
                        self.state = MyState.END
                    else:
                        time.sleep(0.1)

                elif self.state == MyState.TRAJ:
                    self._traj_step()
                    self._log_and_monitor()

                elif self.state == MyState.END:
                    # Used only for fatal conditions; here we shut down and exit.
                    self._safe_shutdown()
                    break

                time.sleep(0.001)

            except Exception as e:
                logging.critical(f"FATAL: Unhandled error in control loop: {e}")
                self.state = MyState.END
                break

        # Loop ended: either END state or global_interrupted != 0
        
        self._safe_shutdown()

        if self.data_file:
            self.data_file.close()

    # ---------------- Trajectory logic ----------------

    def _traj_step(self):
        """
        Trajectory logic with gear ratio.

        Internal velocities are in MOTOR turns/s.
        Any velocity printed or logged is converted to OUTPUT rpm:

            rpm_output = motor_turns_s * 60 / GEAR_RATIO
        """
        global global_run

        now = time.monotonic()

        # ---------- Global PAUSE handling (SIGUSR2 OR post-run pause) ----------
        if not global_run:
            # Ensure motor commanded to zero while paused
            try:
                self.cmd_msg.position[JOINT_ID] = 0.
                self.cmd_msg.kp[JOINT_ID] = 0.

                self.cmd_msg.velocity[JOINT_ID] = 0.
                self.cmd_msg.acceleration[JOINT_ID] = 0.
                self.cmd_msg.kd[JOINT_ID] = 0.

                self.cmd_msg.effort[JOINT_ID] = 0.
            except Exception:
                pass

            # Reset timing / indices so next SIGUSR1 starts from a clean state
            self.t_step_start = now
            self.idx_ref_vel = 0
            self._was_running = False
            self._single_phase = "idle"
            return

        # At this point, global_run == True (trajectory should be running)

                # ================= SINGLE-STEP MODE =================
        if NUM_REF_VEL_STEPS == 1:
            target_rpm = MAX_RPM

            if not self._was_running:
                self._was_running = True
                self._single_start_time = now
                logging.info(
                    f"Single-step execution started: "
                    f"target={target_rpm:.2f} rpm, "
                    f"time={SECS_PER_VEL_STEP:.3f}s"
                )

            elapsed = now - self._single_start_time

            if elapsed >= SECS_PER_VEL_STEP:
                self.cmd_msg.velocity[JOINT_ID] = 0.0
                global_run = False
                self._was_running = False
                
                self.cmd_msg.position[JOINT_ID] = 0.
                self.cmd_msg.kp[JOINT_ID] = 0.

                self.cmd_msg.velocity[JOINT_ID] = 0.
                self.cmd_msg.acceleration[JOINT_ID] = 0.
                self.cmd_msg.kd[JOINT_ID] = 0.

                self.cmd_msg.effort[JOINT_ID] = 0.
                
                logging.info("Single-step finished. Paused.")
                return

            # --- RPM RAMP LOGIC ---
            if SINGLE_STEP_RAMP_RATE_RPM_PER_S <= 0.0:
                cmd_rpm = target_rpm
            else:
                ramp_rate = SINGLE_STEP_RAMP_RATE_RPM_PER_S

                ramp_time = target_rpm / ramp_rate
                ramp_time = min(ramp_time, SECS_PER_VEL_STEP / 2.0)

                if elapsed < ramp_time:
                    cmd_rpm = ramp_rate * elapsed
                elif elapsed > (SECS_PER_VEL_STEP - ramp_time):
                    cmd_rpm = ramp_rate * (SECS_PER_VEL_STEP - elapsed)
                else:
                    cmd_rpm = target_rpm

            # Convert to rad/s only HERE
            self.cmd_msg.velocity[JOINT_ID] = rpm_to_rad_s(cmd_rpm)
            self.cmd_msg.kd[JOINT_ID] = 2.0
            return


        # ================= MULTI-STEP MODE =================
        # All ref_vels_rpm are MOTOR turns/s, we print them as OUTPUT rpm.

        # Rising edge: start a fresh multi-step sequence
        if not self._was_running:
            self._was_running = True
            self._cycle_count = 0
            self.idx_ref_vel = 0
            self.idx_ref_vel_dir = 1
            self.t_step_start = now
            logging.info("Ramp sequence started (multi-step).")

            # Enable logging at first start
            if not self.logging_enabled:
                self.logging_enabled = True
                logging.info("Logging ENABLED (first SIGUSR1 received).")


        time_since_step = now - self.t_step_start

        # Step boundary
        if time_since_step >= SECS_PER_VEL_STEP:
            next_idx = self.idx_ref_vel + self.idx_ref_vel_dir

            if next_idx >= len(self.ref_vels_rpm):
                # reached top -> go down
                self.idx_ref_vel_dir = -1
                self.idx_ref_vel = len(self.ref_vels_rpm) - 2
                max_rpm_out = rad_s_to_rpm(REF_VEL_MAX_RAD_S)
                logging.info(
                    f"Reached top step ({max_rpm_out:.2f} rpm output). Starting ramp DOWN."
                )
            elif next_idx < 0:
                # back to bottom -> one full cycle done
                self.idx_ref_vel_dir = +1
                self.idx_ref_vel = 0
                self._cycle_count += 1
                logging.info(f"Completed cycle #{self._cycle_count}.")

                if RAMP_REPEAT > 0 and self._cycle_count >= RAMP_REPEAT:
                    # After last cycle: stop and PAUSE, not exit.
                    self.cmd_msg.position[JOINT_ID] = 0.
                    self.cmd_msg.kp[JOINT_ID] = 0.

                    self.cmd_msg.velocity[JOINT_ID] = 0.
                    self.cmd_msg.acceleration[JOINT_ID] = 0.
                    self.cmd_msg.kd[JOINT_ID] = 0.
                    self.cmd_msg.effort[JOINT_ID] = 0.

                    global_run = False
                    self._was_running = False
                    logging.info(
                        "Requested repetition count reached. Controller is now PAUSED. "
                        "Send SIGUSR1 to start again."
                    )
                    return
            else:
                # normal step
                self.idx_ref_vel = next_idx

            self.t_step_start = now
            step_rpm = self.ref_vels_rpm[self.idx_ref_vel]
            logging.info(
                f"Step index {self.idx_ref_vel}: command {step_rpm:.2f} rpm (output)."
            )

        # Command current step velocity (within step hold window)
        cmd_rpm = self.ref_vels_rpm[self.idx_ref_vel]
        try:
            self.cmd_msg.position[JOINT_ID] = 0.0
            self.cmd_msg.kp[JOINT_ID] = 0.0
            self.cmd_msg.velocity[JOINT_ID] = rpm_to_rad_s(cmd_rpm)
            self.cmd_msg.acceleration[JOINT_ID] = 0.0
            self.cmd_msg.kd[JOINT_ID] = 1.0
            self.cmd_msg.effort[JOINT_ID] = 0.0

        except Exception:
            pass

    # ---------------- Logging of current state ----------------

    def _log_and_monitor(self):
        if not self.logging_enabled or self.csv_writer is None:
            # Do not log until first SIGUSR1
            return

        try:
            measured_pos = self.state_msg.position[JOINT_ID]
            measured_vel_rad_s_motor = self.state_msg.velocity[JOINT_ID]
            cmd_vel_rad_s_motor = self.cmd_msg.velocity[JOINT_ID]

            Iq_measured = 0 # TODO: Find out what this is
            tau_meas_Nm = self.cmd_msg.effort[JOINT_ID]

            # Convert motor turns/s -> output rpm for logging
            cmd_vel_rpm_out = rad_s_to_rpm(cmd_vel_rad_s_motor)
            measured_vel_rpm_out = rad_s_to_rpm(measured_vel_rad_s_motor)

            # TODO: Check if we can get temp (motor interface dont provide it)
            temp_fet = float("nan")
            temp_motor = float("nan")
            try:
                temp_motor = self.temp
            except Exception:
                pass

            self._log_data(
                cmd_vel_rpm_out,
                measured_pos,
                measured_vel_rpm_out,
                tau_meas_Nm,
                Iq_measured,
                temp_fet,
                temp_motor
            )

        except Exception as e:
            logging.warning(f"Non-fatal error in _log_and_monitor: {e}")

    # ---------------- Shutdown ----------------

    def _safe_shutdown(self):
        logging.warning("Initiating Cubemars shutdown procedure (AxisState.IDLE).")
        self.bringup_to_state("finelized")
        
        logging.info("Controller process finished.")

# ---------------- main ----------------

if __name__ == "__main__":
    # Write PID to file so the C program can find and delete the file once the process is done.
    try:
        with open(PID_FILE, "w") as f:
            f.write(str(os.getpid()))
    except Exception as e:
        logging.error(f"Failed to write PID file {PID_FILE}: {e}")

    controller = None
    rclpy.init()
    controller = CubemarsController()
    controller.bringup_to_state("active")
    try:
        controller.run_controller()
        # controller.thread.start()
        # print("Thread started")
        # while(rclpy.ok()):
        #     time.sleep(1)
        rclpy.spin(controller.node)

    except KeyboardInterrupt:
        # Just set global_interrupted; run_controller loop will see it and shut down cleanly
        logging.warning("KeyboardInterrupt received — requesting controlled shutdown.")
        global_interrupted = 1

    except Exception as e:
        logging.critical(f"Unhandled exception in main process: {e}")
        try:
            if controller and controller.my_drive:
                # dump_errors(controller.my_drive)
                ...
        except Exception:
            pass

    finally:
        try:
            if controller:
                controller._safe_shutdown()
        except Exception as e:
            logging.warning(f"Exception while safe-shutdown from main finally: {e}")

        try:
            if controller and controller.data_file:
                controller.data_file.close()
        except Exception as e:
            logging.warning(f"Failed to close data file: {e}")

        try:
            if os.path.exists(PID_FILE):
                os.remove(PID_FILE)
        except Exception as e:
            logging.warning(f"Failed to remove PID file {PID_FILE}: {e}")

        logging.info("Program exited cleanly.")
