"""
plotting.py
------------------
Utilities for loading, aligning, and analyzing torque/velocity command
and measurement signals. Provides Bode plot estimation via Welch's method
and time-domain trajectory visualization.
"""

import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import csd, welch, coherence
from scipy.interpolate import interp1d


def load_and_align(cmd_file: str, meas_file: str):
    """
    Load command and measurement CSV files, then resample both onto a
    common time base using linear interpolation.

    Parameters
    ----------
    cmd_file : str
        Path to the command CSV. Expected columns:
        'Time', 'Torque (Nm)', 'Velocity (RPM)'.
    meas_file : str
        Path to the measurement CSV. Expected columns:
        'Time', 'Torque', 'Speed'.

    Returns
    -------
    t_common : ndarray
        Uniformly spaced time vector covering the overlapping time range.
    torque_cmd : ndarray
        Command torque resampled onto t_common.
    torque_mes : ndarray
        Measured torque resampled onto t_common.
    velocity_cmd : ndarray
        Command velocity resampled onto t_common.
    velocity_mes : ndarray
        Measured velocity resampled onto t_common.
    """
    cmd_df = pd.read_csv(cmd_file)
    meas_df = pd.read_csv(meas_file)

    # --- Extract raw signals ---
    t_cmd = cmd_df["Time"].values
    torque_cmd_raw = cmd_df["Torque (Nm)"].values
    velocity_cmd_raw = cmd_df["Velocity (RPM)"].values

    t_meas = meas_df["Time"].values
    torque_mes_raw = meas_df["Torque"].values
    velocity_mes_raw = meas_df["Speed"].values

    # --- Build common time base over the overlapping interval ---
    t_start = max(t_cmd.min(), t_meas.min())
    t_end = min(t_cmd.max(), t_meas.max())
    dt = np.median(np.diff(t_cmd))
    t_common = np.arange(t_start, t_end, dt)

    # --- Interpolate all signals onto the common time base ---
    torque_cmd = interp1d(t_cmd, torque_cmd_raw, fill_value="extrapolate")(t_common)
    velocity_cmd = interp1d(t_cmd, velocity_cmd_raw, fill_value="extrapolate")(t_common)
    torque_mes = interp1d(t_meas, torque_mes_raw, fill_value="extrapolate")(t_common)
    velocity_mes = interp1d(t_meas, velocity_mes_raw, fill_value="extrapolate")(t_common)

    return t_common, torque_cmd, torque_mes, velocity_cmd, velocity_mes


def estimate_transfer_function(t: np.ndarray, input_signal: np.ndarray,
                               output_signal: np.ndarray, nperseg: int = 2048):
    """
    Estimate the frequency response (transfer function) between an input and
    output signal using Welch's averaged periodogram method.

    H(f) = Pyu(f) / Puu(f)

    Parameters
    ----------
    t : ndarray
        Time vector (uniform spacing assumed).
    input_signal : ndarray
        Input (command) signal.
    output_signal : ndarray
        Output (measured) signal.
    nperseg : int, optional
        Number of samples per Welch segment. Default is 2048.

    Returns
    -------
    freqs : ndarray
        Frequency vector (Hz).
    magnitude_db : ndarray
        Magnitude of the transfer function in dB.
    phase_pi : ndarray
        Phase of the transfer function in units of π radians.
    coherence_vals : ndarray
        Magnitude-squared coherence between input and output (0 to 1).
    """
    fs = 1.0 / np.mean(np.diff(t))

    freqs, Puu = welch(input_signal, fs=fs, nperseg=nperseg)
    freqs, Pyu = csd(output_signal, input_signal, fs=fs, nperseg=nperseg)

    H = Pyu / Puu
    magnitude_db = 20 * np.log10(np.abs(H))
    phase_pi = np.angle(H) / math.pi

    _, coherence_vals = coherence(input_signal, output_signal, fs=fs, nperseg=nperseg)

    return freqs, magnitude_db, phase_pi, coherence_vals

def plot_trajectory(torque: np.ndarray, velocity: np.ndarray, dt: float):
    """
    Plot torque and velocity as step signals on a shared time axis.

    Each value is held constant until the next sample, reflecting the
    discrete nature of the command signals.

    Parameters
    ----------
    torque : ndarray
        Torque command samples (Nm).
    velocity : ndarray
        Velocity command samples (RPM).
    dt : float
        Time step between samples (s).
    """
    time = np.arange(len(torque)) * dt  # build time vector from sample count and dt

    fig, ax = plt.subplots(figsize=(10, 5))

    ax.plot(time, torque,   label="Torque (Nm)",    drawstyle="steps-post")
    ax.plot(time, velocity, label="Velocity (RPM)", drawstyle="steps-post")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Values")
    ax.set_title("Trajectory")
    ax.legend()
    ax.grid()

    plt.tight_layout()
    plt.show()

def plot_chirp(cmd_file: str, meas_file: str, max_freq: float, name: str = "Chirp"):
    """
    Plot a Bode diagram (magnitude, phase, coherence) for the torque
    input/output pair loaded from the given files.

    The trusted frequency band [0, max_freq] is highlighted in green.
    The -3 dB crossover frequency is marked with a red dashed line.

    Parameters
    ----------
    cmd_file : str
        Path to the command CSV file.
    meas_file : str
        Path to the measurement CSV file.
    max_freq : float
        Upper frequency limit of the reliable excitation band (Hz).
    name : str, optional
        Title displayed on the figure. Default is "Chirp".
    """
    t, torque_cmd, torque_mes, _, _ = load_and_align(cmd_file, meas_file)
    freqs, magnitude_db, phase_pi, coherence_vals = estimate_transfer_function(
        t, torque_cmd, torque_mes
    )

    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    fig.suptitle(name)

    # Shade the trusted frequency band on all subplots
    for ax in axes:
        ax.axvspan(freqs[0], max_freq, color="lightgreen", alpha=0.3)

    # --- Find the -3 dB crossover frequency ---
    indices_below_3db = np.where(magnitude_db <= -3)[0]
    freq_3db = freqs[indices_below_3db[0]] if len(indices_below_3db) > 0 else None

    # --- Magnitude ---
    axes[0].semilogx(freqs, magnitude_db, label="Magnitude")
    if freq_3db is not None:
        axes[0].axvline(freq_3db, color="red", linestyle="--",
                        label=f"-3 dB at {freq_3db:.2f} Hz")
    axes[0].set_ylabel("Magnitude (dB)")
    axes[0].legend()
    axes[0].grid(True, which="both")

    # --- Phase ---
    axes[1].semilogx(freqs, phase_pi, label="Phase")
    axes[1].set_ylabel("Phase (× π rad)")
    axes[1].grid(True, which="both")

    # --- Coherence ---
    axes[2].semilogx(freqs, coherence_vals, label="Coherence")
    axes[2].set_ylabel("Coherence")
    axes[2].set_xlabel("Frequency (Hz)")
    axes[2].set_ylim(0, 1)
    axes[2].grid(True, which="both")

    plt.tight_layout()
    plt.show()


def plot_ramp(cmd_file: str, meas_file: str, name: str = "Ramp"):
    """
    Plot command vs. measured torque and velocity as step signals over time.

    Two vertically stacked subplots share a common time axis:
      - Top:    Torque CMD vs. Torque MES
      - Bottom: Velocity CMD vs. Velocity MES

    Parameters
    ----------
    cmd_file : str
        Path to the command CSV file.
    meas_file : str
        Path to the measurement CSV file.
    name : str, optional
        Title displayed on the figure. Default is "Trajectory".
    """
    t, torque_cmd, torque_mes, velocity_cmd, velocity_mes = load_and_align(
        cmd_file, meas_file
    )

    fig, (ax_torque, ax_velocity) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(name)

    # --- Torque ---
    ax_torque.plot(t, torque_cmd, label="Torque command", drawstyle="steps-post")
    ax_torque.plot(t, torque_mes, label="Torque measured", drawstyle="steps-post")
    ax_torque.set_ylabel("Torque (Nm)")
    ax_torque.set_title("Torque")
    ax_torque.legend()
    ax_torque.grid()

    # --- Velocity ---
    ax_velocity.plot(t, velocity_cmd, label="Velocity command", drawstyle="steps-post")
    ax_velocity.plot(t, velocity_mes, label="Velocity measured", drawstyle="steps-post")
    ax_velocity.set_ylabel("Velocity (RPM)")
    ax_velocity.set_title("Velocity")
    ax_velocity.legend()
    ax_velocity.grid()

    ax_velocity.set_xlabel("Time (s)")
    plt.tight_layout()
    plt.show()