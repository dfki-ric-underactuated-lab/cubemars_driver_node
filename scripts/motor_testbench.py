import os
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

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State

# Used to read the PID by other programs.
PID_FILE = "/home/testbench/odrive/python_cubemars/ake90-8/cubemars_control_ake90.pid"

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
        logging.info("SIGUSR2 received: Trajectory Paused.")
        return
    if sig == signal.SIGUSR1:
        # SIGUSR1 -> Start/Resume the trajectory
        global_run = True
        logging.info("SIGUSR1 received: Trajectory Running.")
        return
    if sig == signal.SIGRTMIN:
        # To start recording data into CSV file
        logging.info("SIGRTMIN received : START CSV recording.")
        controller.logging_enabled = True
        return
    if sig == signal.SIGRTMIN + 1:
        # To stop recording data into csv file
        logging.info("SIGRTMIN+1 received : STOP CSV recording")
        controller.logging_enabled = False
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

def rad_s_to_turns_s(rad_s):
    return rad_s / (2 * math.pi)

# -------------------------
# USER-CONFIGURABLE PARAMETERS
# -------------------------

# Gear ratio: motor_speed = output_speed * GEAR_RATIO
GEAR_RATIO = 8

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

# Convert to motor turns/sec for the ODrive:
# output_rpm -> output_turns/s = rpm / 60
# motor_turns/s = output_turns/s * GEAR_RATIO
REF_VEL_MAX_TURNS_S = (MAX_RPM / 60.0) * GEAR_RATIO

# Convert ramp rate to MOTOR turns/s2
SINGLE_STEP_RAMP_RATE_TS_PER_S2 = (SINGLE_STEP_RAMP_RATE_RPM_PER_S / 60.0) * GEAR_RATIO


# Controller Limits (in motor turns/s)
VEL_MAX_TURNS_S = REF_VEL_MAX_TURNS_S * 1.5
CUR_MAX_AMPS = 9.0

# Function to print user defined variables in the terminal
def print_user_configuration():
    logging.info("========== USER CONFIGURATION ==========")
    logging.info(f"Gear ratio (GEAR_RATIO): {GEAR_RATIO}")
    logging.info(f"Max output speed (MAX_RPM): {MAX_RPM} rpm")
    logging.info(f"Number of velocity steps (NUM_REF_VEL_STEPS): {NUM_REF_VEL_STEPS}")
    logging.info(f"Seconds per velocity step (SECS_PER_VEL_STEP): {SECS_PER_VEL_STEP} s")
    logging.info(f"Ramp repeat count (RAMP_REPEAT): {RAMP_REPEAT}")

    logging.info("Single-step mode parameters (if applicable):")
    logging.info(f"  Ramp rate (SINGLE_STEP_RAMP_RATE_RPM_PER_S): {SINGLE_STEP_RAMP_RATE_RPM_PER_S} rpm")
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

class CubemarsController(Node):
    """Manages configuration, live control, and trajectory execution."""
    def __init__(self, drive_id=None):
        super().__init__('CubemarsMotorTestNode')

        self.state_msg = JointState()
        
        def sub_callback(msg):
            self.state_msg = msg

        self.publisher_ = self.create_publisher(
            JointCommand,
            '/cubemars_hardware_node/joint_commands',
            10)

        self.subscription = self.create_subscription(
            JointState,
            '/cubemars_hardware_node/ros2_joint_state',
            sub_callback,
            10)
        
        self.subscription  # prevent unused variable warning

        self.state = MyState.WAIT_CONF
        self.my_drive = None
        self.axis = None
        self.drive_id = drive_id

        # Trajectory variables (in MOTOR turns/s)
        self.ref_vels_ts = []
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

    # ---------------- Trajectory setup ----------------

    def _setup_velocity_profile(self):
        """Pre-calculates the vector of velocity steps (in MOTOR turns/s)."""
        self.ref_vels_ts.clear()
        if NUM_REF_VEL_STEPS <= 0:
            self.ref_vels_ts.append(0.0)
        elif NUM_REF_VEL_STEPS == 1:
            # single-value mode: 0 and max
            self.ref_vels_ts.append(0.0)
            self.ref_vels_ts.append(REF_VEL_MAX_TURNS_S)
        else:
            # multi-step: 0 .. max in N steps
            for i in range(NUM_REF_VEL_STEPS + 1):
                vel_ts_motor = i * REF_VEL_MAX_TURNS_S / NUM_REF_VEL_STEPS
                self.ref_vels_ts.append(vel_ts_motor)

        max_rpm_output = REF_VEL_MAX_TURNS_S * 60.0 / GEAR_RATIO
        logging.info(
            f"Generated {len(self.ref_vels_ts)} velocity steps "
            f"(Max output: {max_rpm_output:.2f} rpm)."
        )

    def _check_errors_and_warnings(self):
        # Currently disabled; you can add real error checks later.
        return False

    # ---------------- CSV logging ----------------

    def _log_data(self, cmd_vel_rpm_out, measured_pos, measured_vel_rpm_out,
                  tau_meas_Nm, Iq_measured,  temp_fet, temp_motor):
        t_unix = time.time()

        if not self.data_file:
            start_label = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            filename = f"/home/testbench/mtb-data/{start_label}_OdriveData_ake90.csv"

            try:
                self.data_file = open(filename, 'w', newline='')
            except Exception as e:
                logging.error(f"Failed to open log file {filename}: {e}")
                raise

            self.csv_writer = csv.writer(self.data_file)
            self.csv_writer.writerow([
                "time_unix_s",
                "Measured_Torque(Nm)",
                "Measured_Velocity_output(rpm)",
                "Current_Measured (A)",
                "Command_Velocity_output(rpm)",
                "Command_Torque(A)",
                "Measured_Position(turns)",
                "Motor_Temperature(°C)"
            ])
            self.data_file.flush()
            logging.info(f"Logging to {filename}")

        self.csv_writer.writerow([
            f"{t_unix:.6f}",
            f"{tau_meas_Nm:.3f}",
            f"{measured_vel_rpm_out:.3f}",
            f"{Iq_measured:.3f}",
            f"{cmd_vel_rpm_out:.3f}",
            f"{0.0:.3f}",
            f"{measured_pos:.3f}",
            f"{temp_motor:.1f}",
        ])
        self.data_file.flush()

    # ---------------- Main control loop ----------------

    def run_controller(self):
        global global_interrupted, global_run

        self._setup_velocity_profile()

        if not self._apply_and_save_config():
            logging.critical("Initial configuration failed. Exiting.")
            sys.exit(1)

        self.t_start = time.monotonic()
        logging.info("--- PHASE 3: Starting Control Loop ---")

        self.log_count = 0
        self.log_last_time = time.time()

        while global_interrupted == 0:
            try:
                if self._check_errors_and_warnings():
                    self.state = MyState.END

                if self.state == MyState.WAIT_CONF:
                    if self.axis.current_state == AxisState.IDLE:
                        self.state = MyState.PREPARE
                        logging.info("State change: WAIT_CONF -> PREPARE")

                elif self.state == MyState.PREPARE:
                    self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
                    self.t_step_start = time.monotonic()
                    self.state = MyState.WAIT_EN
                    logging.info("State change: PREPARE -> WAIT_EN (Commanding CLOSED_LOOP_CONTROL)")

                    # To have a dry test run without having the motor run
                    '''logging.warning("TEST MODE: Skipping CLOSED_LOOP_CONTROL (motor stays IDLE)")
                    self.t_step_start = time.monotonic()
                    self.state = MyState.TRAJ'''

                elif self.state == MyState.WAIT_EN:
                    if self.axis.current_state == AxisState.CLOSED_LOOP_CONTROL:
                        self.axis.controller.input_vel = 0.0
                        self.axis.controller.input_torque = 0.0
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
        if self.axis:
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
                self.axis.controller.input_vel = 0.0
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
            target_ts = REF_VEL_MAX_TURNS_S  # motor turns/s

            # Rising edge: start single-step execution
            if not self._was_running:
                self._was_running = True
                self._single_start_time = now

                # Enable logging at first start
                if not self.logging_enabled:
                    self.logging_enabled = True
                    logging.info("Logging ENABLED (first SIGUSR1 received).")

                logging.info(
                    f"Single-step execution started: "
                    f"target={target_rpm:.2f} rpm, "
                    f"total time={SECS_PER_VEL_STEP:.3f}s"
                )

            elapsed = now - self._single_start_time

            # Finished execution ? pause
            if elapsed >= SECS_PER_VEL_STEP:
                try:
                    self.axis.controller.input_vel = 0.0
                except Exception:
                    pass

                global_run = False
                self._was_running = False
                logging.info(
                    "Single-step execution complete; controller is now PAUSED. "
                    "Send SIGUSR1 to start again."
                )
                return

            # -------- Compute commanded velocity --------

            if SINGLE_STEP_RAMP_RATE_RPM_PER_S <= 0.0:
                # No ramp ? immediate step
                cmd_ts = target_ts

            else:
                # Convert ramp rate to motor units
                ramp_rate_ts = SINGLE_STEP_RAMP_RATE_TS_PER_S2

                # Time needed to ramp from 0 ? target
                ramp_time = target_ts / ramp_rate_ts

                # Limit ramp time so up+down fits in total time
                max_ramp_time = SECS_PER_VEL_STEP / 2.0
                ramp_time = min(ramp_time, max_ramp_time)

                # Ramp-up
                if elapsed < ramp_time:
                    cmd_ts = ramp_rate_ts * elapsed

                # Ramp-down
                elif elapsed > (SECS_PER_VEL_STEP - ramp_time):
                    t_down = SECS_PER_VEL_STEP - elapsed
                    cmd_ts = ramp_rate_ts * t_down

                # Hold
                else:
                    cmd_ts = target_ts

            # Apply command
            try:
                self.axis.controller.config.input_mode = InputMode.PASSTHROUGH
                self.axis.controller.input_vel = cmd_ts
            except Exception:
                pass

            return


        # ================= MULTI-STEP MODE =================
        # All ref_vels_ts are MOTOR turns/s, we print them as OUTPUT rpm.

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

            if next_idx >= len(self.ref_vels_ts):
                # reached top -> go down
                self.idx_ref_vel_dir = -1
                self.idx_ref_vel = len(self.ref_vels_ts) - 2
                max_rpm_out = REF_VEL_MAX_TURNS_S * 60.0 / GEAR_RATIO
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
                    try:
                        self.axis.controller.input_vel = 0.0
                    except Exception:
                        pass
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

            step_vel_ts_motor = self.ref_vels_ts[self.idx_ref_vel]
            step_rpm_out = step_vel_ts_motor * 60.0 / GEAR_RATIO
            logging.info(
                f"Step index {self.idx_ref_vel}: command {step_rpm_out:.2f} rpm (output)."
            )

        # Command current step velocity (within step hold window)
        cmd_vel_ts_motor = self.ref_vels_ts[self.idx_ref_vel]
        try:
            self.axis.controller.config.input_mode = InputMode.PASSTHROUGH
            self.axis.controller.input_vel = cmd_vel_ts_motor
        except Exception:
            pass

    # ---------------- Logging of current state ----------------

    def _log_and_monitor(self):
        if not self.logging_enabled:
            # Do not log until first SIGUSR1
            return

        try:
            measured_pos = self.axis.pos_estimate
            measured_vel_ts_motor = self.axis.vel_estimate
            cmd_vel_ts_motor = self.axis.controller.input_vel

            Iq_measured = self.axis.motor.foc.Iq_measured
            Kt = self.axis.config.motor.torque_constant
            tau_meas_Nm = Iq_measured * Kt

            # Convert motor turns/s -> output rpm for logging
            cmd_vel_rpm_out = cmd_vel_ts_motor * 60.0 / GEAR_RATIO
            measured_vel_rpm_out = measured_vel_ts_motor * 60.0 / GEAR_RATIO

            temp_fet = float("nan")
            temp_motor = float("nan")
            try:
                temp_fet = self.my_drive.fet_thermistor.temperature
            except Exception:
                pass
            try:
                temp_motor = self.axis.motor.motor_thermistor.temperature
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

            self.log_count += 1
            now = time.time()
            elapsed = now - self.log_last_time
            if elapsed >= 10.0:
                rate_hz = self.log_count / elapsed
                logging.info(f"Actual logging rate: {rate_hz:.2f} Hz")
                self.log_count = 0
                self.log_last_time = now

        except Exception as e:
            logging.warning(f"Non-fatal error in _log_and_monitor: {e}")

    # ---------------- Shutdown ----------------

    def _safe_shutdown(self):
        logging.warning("Initiating ODrive shutdown procedure (AxisState.IDLE).")
        if self.axis:
            try:
                self.axis.controller.input_vel = 0.0
                time.sleep(0.1)
                self.axis.requested_state = AxisState.IDLE
                t_disable_start = time.monotonic()
                while self.axis.current_state != AxisState.IDLE and (time.monotonic() - t_disable_start < 0.5):
                    time.sleep(0.05)
                if self.axis.current_state == AxisState.IDLE:
                    logging.info("Motor successfully disarmed (AxisState.IDLE).")
                else:
                    logging.warning("Motor did not reach IDLE state, exiting anyway.")
            except Exception as e:
                logging.error(f"Error during ODrive shutdown: {e}")
        logging.info("Controller process finished.")

# ---------------- main ----------------

if __name__ == "__main__":

    print_user_configuration()
    validate_single_step_ramp()

    # Write PID to file so the C program can find and delete the file once the process is done.
    try:
        with open(PID_FILE, "w") as f:
            f.write(str(os.getpid()))
    except Exception as e:
        logging.error(f"Failed to write PID file {PID_FILE}: {e}")

    controller = None
    rclpy.init(args=args)
    try:
        # controller = ODriveMotionController()
        # controller.run_controller()
        controller = CubemarsController()
        rclpy.spin(controller)

    except KeyboardInterrupt:
        # Just set global_interrupted; run_controller loop will see it and shut down cleanly
        logging.warning("KeyboardInterrupt received — requesting controlled shutdown.")
        global_interrupted = 1

    except Exception as e:
        logging.critical(f"Unhandled exception in main process: {e}")
        try:
            if controller and controller.my_drive:
                dump_errors(controller.my_drive)
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
