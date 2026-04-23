#!/usr/bin/env python3
"""Entry point for the CubeMars motor testbench.

Install the package first, then run via:
    python -m testbench.main
    # or, after pip install -e .:
    testbench
"""

from __future__ import annotations

import logging
import math
import os
import signal
import time
from typing import Callable

import numpy as np

from .controller import TestConfig, TestController
from .driver.cubemars import CubemarsDriver
from .plotting import plot_chirp, plot_ramp, plot_trajectory, show_plot
from .trajectories import generate_chirp, generate_ramp

# ============================================================
# Configuration
# ============================================================

JOINT_ID = 0
KD = 5.0
COMMAND_FREQ = 1000.0
MAX_DELTA_CMD = 1.0

PID_FILE = "/tmp/hilscher.pid"
LOGDIR = "/home/testbench/mtb-data"

# ---- Test selection (exactly one should be True) ----
CHIRP_TEST = False
TORQUE_RAMP = True
VEL_RAMP = False

# ---- Chirp parameters ----
CHIRP_AMPLITUDE = 1.0     # Nm
CHIRP_START_FREQ = 0.1    # Hz
CHIRP_END_FREQ = 100.0    # Hz
CHIRP_DURATION = 10.0     # s
CHIRP_LOGARITHMIC = True

# ---- Ramp parameters ----
SECS_PER_RAMP_STEP = 0.3
NUM_STEPS = 10
MIN_TORQUE = 0.0
MAX_TORQUE = 2.0
MIN_RPM = 5.0
MAX_RPM = 20.0

# ============================================================
# Helpers
# ============================================================

def _rpm_to_rad_s(rpm: float) -> float:
    return rpm * math.pi / 30.0


def _rad_s_to_rpm(rad_s: float) -> float:
    return rad_s * 30.0 / math.pi


def _build_trajectory() -> tuple[np.ndarray, np.ndarray, str, Callable]:
    """Return ``(torque_traj, vel_traj, test_name, post_plot_fn)``."""
    if CHIRP_TEST:
        name = (
            f"CHIRP_{CHIRP_AMPLITUDE}Nm"
            f"_{CHIRP_START_FREQ}Hz_{CHIRP_END_FREQ}Hz"
            f"_{CHIRP_DURATION}s"
        )
        torque = generate_chirp(
            CHIRP_AMPLITUDE, CHIRP_START_FREQ, CHIRP_END_FREQ,
            CHIRP_DURATION, COMMAND_FREQ, CHIRP_LOGARITHMIC,
        )
        vel = np.full(len(torque), np.nan)
        post = lambda cmd, state, meas: plot_chirp(cmd, state, meas, CHIRP_END_FREQ, name)

    elif TORQUE_RAMP:
        name = f"TORQUE_RAMP_{MAX_TORQUE}Nm_{NUM_STEPS}x_{SECS_PER_RAMP_STEP}s"
        torque = generate_ramp(
            MIN_TORQUE, MAX_TORQUE, NUM_STEPS,
            SECS_PER_RAMP_STEP, COMMAND_FREQ, MAX_DELTA_CMD,
        )
        vel = np.full(len(torque), np.nan)
        post = lambda cmd, state, meas: plot_ramp(cmd, state, meas, name)

    elif VEL_RAMP:
        name = f"VEL_RAMP_{MAX_RPM}RPM_{NUM_STEPS}x_{SECS_PER_RAMP_STEP}s"
        vel = generate_ramp(
            _rpm_to_rad_s(MIN_RPM), _rpm_to_rad_s(MAX_RPM), NUM_STEPS,
            SECS_PER_RAMP_STEP, COMMAND_FREQ, MAX_DELTA_CMD,
        )
        torque = np.full(len(vel), np.nan)
        post = lambda cmd, state, meas: plot_ramp(cmd, state, meas, name)

    else:
        raise ValueError("No test mode selected — set CHIRP_TEST, TORQUE_RAMP, or VEL_RAMP.")

    return torque, vel, name, post


def _log_configuration(test_name: str) -> None:
    logging.info("=" * 44)
    logging.info(f"Test   : {test_name}")
    logging.info(f"Joint  : {JOINT_ID}  |  KD: {KD}  |  {COMMAND_FREQ:.0f} Hz")
    logging.info(f"Log dir: {LOGDIR}")
    logging.info("=" * 44)


# ============================================================
# Entry point
# ============================================================

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    torque_traj, vel_traj, test_name, post_plot = _build_trajectory()
    _log_configuration(test_name)

    # Preview trajectory before the hardware connects
    plot_trajectory(torque_traj, _rad_s_to_rpm(vel_traj), 1.0 / COMMAND_FREQ)

    config = TestConfig(
        joint_id=JOINT_ID,
        default_kd=KD,
        logdir=LOGDIR,
        test_name=test_name,
        torque_traj=torque_traj,
        vel_traj=vel_traj,
        post_plot=post_plot,
    )

    driver = CubemarsDriver(joint_id=JOINT_ID, command_freq=COMMAND_FREQ, default_kd=KD)
    controller = TestController(driver, config)

    # ---- Signal handlers ----
    interrupted = False

    def _on_interrupt(*_):
        nonlocal interrupted
        interrupted = True
        logging.warning("Shutdown requested")

    signal.signal(signal.SIGINT,       _on_interrupt)
    signal.signal(signal.SIGTERM,      _on_interrupt)
    signal.signal(signal.SIGUSR1,      lambda *_: controller.arm())
    signal.signal(signal.SIGUSR2,      lambda *_: controller.disarm())
    signal.signal(signal.SIGRTMIN,     lambda *_: controller.start_logging())
    signal.signal(signal.SIGRTMIN + 1, lambda *_: controller.stop_logging())

    with open(PID_FILE, "w") as f:
        f.write(str(os.getpid()))

    driver.start(controller.step)
    try:
        while not interrupted:
            show_plot()
            time.sleep(0.1)
    finally:
        driver.shutdown()
        if os.path.exists(PID_FILE):
            os.remove(PID_FILE)
        logging.info("Exited cleanly")


if __name__ == "__main__":
    main()
