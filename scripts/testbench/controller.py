"""Transport-agnostic test sequence controller."""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable

import numpy as np

from .csv_logger import CsvLogger
from .driver.base import MotorCommand, MotorDriverBase, MotorState


def _rad_s_to_rpm(rad_s: float) -> float:
    return rad_s * 30.0 / math.pi


class _Phase(Enum):
    IDLE = auto()
    INIT = auto()
    ACTIVE = auto()
    DONE = auto()


@dataclass
class TestConfig:
    """All parameters needed to run one test sequence."""

    joint_id: int
    default_kd: float
    logdir: str
    test_name: str
    torque_traj: np.ndarray
    vel_traj: np.ndarray
    post_plot: Callable[[str, str, str], None] | None = None
    """Called with ``(cmd_csv, state_csv, meas_csv)`` when the test finishes."""


class TestController:
    """
    Drives the test state machine and manages CSV logging.

    :meth:`step` is called on every control tick by the driver's timer.
    All other public methods are intended to be invoked from signal
    handlers in the entry-point script.
    """

    def __init__(self, driver: MotorDriverBase, config: TestConfig) -> None:
        self._driver = driver
        self._cfg = config
        self._phase = _Phase.IDLE
        self._traj_idx = 0
        self._armed = False
        self._log_name: str | None = None
        self._cmd_logger: CsvLogger | None = None
        self._state_logger: CsvLogger | None = None

    # ------------------------------------------------------------------
    # Public API (called from signal handlers in main.py)
    # ------------------------------------------------------------------

    def arm(self) -> None:
        """Arm the controller so the next :meth:`step` starts a sequence (SIGUSR1)."""
        self._armed = True
        logging.info("Controller armed")

    def disarm(self) -> None:
        """Pause after the current step (SIGUSR2)."""
        self._armed = False
        logging.info("Controller disarmed")

    def start_logging(self) -> None:
        """Open CSV log files. Safe to call before or during a test."""
        self._log_name = time.strftime("%Y%m%d_%H%M%S")
        cfg = self._cfg
        prefix = f"{cfg.logdir}/{self._log_name}_Cubemars_{cfg.test_name}"

        self._cmd_logger = CsvLogger(
            filename=f"{prefix}_COMMAND.csv",
            header=["Time", "Torque (Nm)", "Velocity (RPM)", "Position (rad)", "KP", "KD"],
        )
        self._state_logger = CsvLogger(
            filename=f"{prefix}_STATE.csv",
            header=["Time", "Torque (Nm)", "Velocity (RPM)", "Position (rad)",
                    "Temp (°C)", "Can Cycle (Hz)"],
        )
        logging.info(f"Logging started → {prefix}")

    def stop_logging(self) -> None:
        """Close and flush all CSV log files."""
        for logger in (self._cmd_logger, self._state_logger):
            if logger is not None:
                logger.close()
        self._cmd_logger = None
        self._state_logger = None
        logging.info("Logging stopped")

    # ------------------------------------------------------------------
    # Control loop (called at command frequency by the driver)
    # ------------------------------------------------------------------

    def step(self) -> None:
        """Advance the state machine by one control tick and publish a command."""
        cfg = self._cfg
        idle_cmd = MotorCommand(kd=cfg.default_kd)

        if not self._armed:
            self._phase = _Phase.IDLE
            self._driver.send_command(idle_cmd)
            return

        if self._phase == _Phase.IDLE:
            self._phase = _Phase.INIT

        if self._phase == _Phase.INIT:
            self._traj_idx = 0
            self._phase = _Phase.ACTIVE
            logging.info("Sequence started")
            self._driver.bring_to_active()
            self._driver.send_command(idle_cmd)
            return

        if self._phase == _Phase.DONE:
            self._on_done()
            self._driver.send_command(idle_cmd)
            return

        # _Phase.ACTIVE
        if self._traj_idx >= len(cfg.torque_traj):
            logging.info("Test complete")
            self._phase = _Phase.DONE
            self._driver.bring_to_inactive()
            self._driver.send_command(idle_cmd)
            return

        cmd = MotorCommand()
        torque = cfg.torque_traj[self._traj_idx]
        vel = cfg.vel_traj[self._traj_idx]
        self._traj_idx += 1

        if not np.isnan(torque):
            cmd.effort = torque
        if not np.isnan(vel):
            cmd.velocity = vel
            cmd.kd = cfg.default_kd

        self._driver.send_command(cmd)
        self._log_step(cmd)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _log_step(self, cmd: MotorCommand) -> None:
        state = self._driver.get_latest_state()
        t = f"{time.time():.6f}"

        if self._cmd_logger:
            self._cmd_logger.log([
                t,
                f"{cmd.effort:.6f}",
                f"{_rad_s_to_rpm(cmd.velocity):.6f}",
                f"{cmd.position:.6f}",
                f"{cmd.kp:.6f}",
                f"{cmd.kd:.6f}",
            ])

        if self._state_logger:
            nan = math.nan
            temp = f"{state.temperature:.2f}" if not math.isnan(state.temperature) else "-1.00"
            can  = f"{state.can_cycle_freq:.2f}" if not math.isnan(state.can_cycle_freq) else "-1.00"
            self._state_logger.log([
                t,
                f"{state.effort:.6f}"                    if not math.isnan(state.effort)    else "nan",
                f"{_rad_s_to_rpm(state.velocity):.6f}"  if not math.isnan(state.velocity)  else "nan",
                f"{state.position:.6f}"                  if not math.isnan(state.position)  else "nan",
                temp,
                can,
            ])

    def _on_done(self) -> None:
        self.stop_logging()
        cfg = self._cfg
        if self._log_name and cfg.post_plot:
            prefix = f"{cfg.logdir}/{self._log_name}_Cubemars_{cfg.test_name}"
            meas_file = f"{cfg.logdir}/{self._log_name}_HilsherData.csv"
            cfg.post_plot(
                f"{prefix}_COMMAND.csv",
                f"{prefix}_STATE.csv",
                meas_file,
            )
        self._phase = _Phase.IDLE
        self._armed = False
