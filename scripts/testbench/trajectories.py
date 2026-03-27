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

def generate_ramp(minimum: float, maximum: float, num_steps: int, secs_per_step: int, freq: float, startup_max_delta: float):
    samples_per_step = max(1, round(secs_per_step * freq))
    ramp = np.concatenate([
        np.linspace(minimum, maximum, num_steps),
        np.linspace(maximum, minimum, num_steps)[1:],
    ])
    ramp = np.repeat(ramp, samples_per_step)
    if minimum != 0.:        
        startup_steps = math.ceil(abs(minimum) / (startup_max_delta / freq))
        ramp = np.concatenate([np.linspace(0., minimum, startup_steps)[:-1], 
                               ramp, 
                               np.linspace(minimum, 0., startup_steps)[1:]])
    return ramp


def generate_combined_ramp(
    min_velocity: float, max_velocity: float, num_velocity_steps: int,
    min_torque: float,   max_torque: float,   num_torque_steps: int,
    secs_per_torque_step: float, fs: float,
):
    samples_per_step = max(1, round(secs_per_torque_step * fs))

    # Triangle ramps (avoid duplicate peak with [1:])
    velocity_ramp = np.concatenate([
        np.linspace(min_velocity, max_velocity if not math.isnan(max_velocity) else 0.0, num_velocity_steps),
        np.linspace(max_velocity if not math.isnan(max_velocity) else 0.0, min_velocity, num_velocity_steps)[1:],
    ])
    torque_ramp = np.concatenate([
        np.linspace(min_torque, max_torque, num_torque_steps),
        np.linspace(max_torque, min_torque, num_torque_steps)[1:],
    ])

    # Cartesian product → repeat each pair for the requested duration
    pairs = list(product(velocity_ramp, torque_ramp))
    velocity_trajectory = np.repeat([v for v, _ in pairs], samples_per_step)
    torque_trajectory   = np.repeat([t for _, t in pairs], samples_per_step)

    return velocity_trajectory, torque_trajectory