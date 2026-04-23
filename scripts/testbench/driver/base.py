"""Motor driver abstract base class and shared data types."""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable


@dataclass
class MotorState:
    """Snapshot of motor feedback at a single point in time."""

    effort: float = math.nan         # Nm
    velocity: float = math.nan       # rad/s
    position: float = math.nan       # rad
    temperature: float = math.nan    # °C
    can_cycle_freq: float = math.nan # Hz


@dataclass
class MotorCommand:
    """Command sent to the motor on each control tick."""

    effort: float = 0.0    # Nm
    velocity: float = 0.0  # rad/s
    position: float = 0.0  # rad
    kp: float = 0.0
    kd: float = 0.0


class MotorDriverBase(ABC):
    """
    Abstract interface for motor driver communication.

    Concrete subclasses handle the transport layer (ROS 2, EtherCAT, …)
    while the TestController stays transport-agnostic.
    """

    @abstractmethod
    def send_command(self, command: MotorCommand) -> None:
        """Publish *command* to the motor on the current control tick."""

    @abstractmethod
    def get_latest_state(self) -> MotorState:
        """Return the most recent motor state feedback."""

    @abstractmethod
    def bring_to_active(self) -> None:
        """Transition the hardware to the active (torque-enabled) state."""

    @abstractmethod
    def bring_to_inactive(self) -> None:
        """Transition the hardware to the inactive (safe) state."""

    @abstractmethod
    def start(self, step_callback: Callable[[], None]) -> None:
        """
        Start the background control loop.

        *step_callback* is called once per control tick (e.g. at 1 kHz).
        Returns immediately; the loop runs on a background thread.
        """

    @abstractmethod
    def shutdown(self) -> None:
        """Stop the control loop and release all driver resources."""
