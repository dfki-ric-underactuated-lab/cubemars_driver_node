"""
trajectory_generation.py
------------------------
Functions for generating torque and velocity trajectories used as
excitation signals for system identification.
"""

import math
import numpy as np
from itertools import product


def generate_chirp(amplitude: float, f_start: float, f_end: float,
                   duration: float, sample_rate: float,
                   logarithmic: bool = False) -> np.ndarray:
    """
    Generate a chirp (frequency-swept sine) torque signal.

    The instantaneous frequency sweeps from f_start to f_end over the
    given duration, either linearly or logarithmically.

    Parameters
    ----------
    amplitude : float
        Peak amplitude of the chirp signal (Nm).
    f_start : float
        Starting frequency of the sweep (Hz).
    f_end : float
        Ending frequency of the sweep (Hz).
    duration : float
        Total duration of the signal (s).
    sample_rate : float
        Number of samples per second (Hz).
    logarithmic : bool, optional
        If True, use a logarithmic (exponential) frequency sweep.
        If False (default), use a linear frequency sweep.

    Returns
    -------
    torque : ndarray
        Chirp signal samples of shape (N,).
    """
    n_samples = int(duration * sample_rate)
    t = np.arange(n_samples) / sample_rate

    if logarithmic:
        # Exponential sweep: f(t) = f_start * exp(beta * t)
        beta = math.log(f_end / f_start) / duration
        phase = 2 * math.pi * f_start * (np.exp(beta * t) - 1) / beta
    else:
        # Linear sweep: f(t) = f_start + k * t
        k = (f_end - f_start) / duration
        phase = 2 * math.pi * (f_start * t + 0.5 * k * t ** 2)

    torque = amplitude * np.sin(phase)
    return torque


def generate_ramp(min_velocity: float, max_velocity: float, num_velocity_steps: int,
                  min_torque: float, max_torque: float, num_torque_steps: int):
    """
    Generate a full Cartesian sweep of velocity and torque ramp trajectories.

    Each signal ramps up from its minimum to its maximum and back down,
    forming a triangle wave. The Cartesian product of the two ramps is then
    computed so that every (velocity, torque) combination is visited.

    Parameters
    ----------
    min_velocity : float
        Minimum velocity value (RPM).
    max_velocity : float
        Maximum velocity value (RPM).
    num_velocity_steps : int
        Number of steps in each half of the velocity ramp (up or down).
    min_torque : float
        Minimum torque value (Nm).
    max_torque : float
        Maximum torque value (Nm).
    num_torque_steps : int
        Number of steps in each half of the torque ramp (up or down).

    Returns
    -------
    velocity_trajectory : ndarray
        Velocity values for each point in the Cartesian sweep.
    torque_trajectory : ndarray
        Torque values for each point in the Cartesian sweep.

    Notes
    -----
    Output length is (2 * num_velocity_steps - 1) * (2 * num_torque_steps - 1).
    The duplicate at the turnaround point is removed with [1:] slicing.
    """
    # Build triangle ramps: up then down, avoiding duplicate peak value
    velocity_ramp = np.concatenate([
        np.linspace(min_velocity, max_velocity, num_velocity_steps),
        np.linspace(max_velocity, min_velocity, num_velocity_steps)[1:]
    ])
    torque_ramp = np.concatenate([
        np.linspace(min_torque, max_torque, num_torque_steps),
        np.linspace(max_torque, min_torque, num_torque_steps)[1:]
    ])

    # Cartesian product: every (velocity, torque) pair, then unzip into two arrays
    velocity_trajectory, torque_trajectory = zip(*product(velocity_ramp, torque_ramp))

    return np.array(velocity_trajectory), np.array(torque_trajectory)