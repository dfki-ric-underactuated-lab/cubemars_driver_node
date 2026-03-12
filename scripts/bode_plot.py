import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import csd, welch, coherence
from scipy.interpolate import interp1d


def load_and_align(cmd_file, meas_file):
    # Load CSV files
    cmd_df = pd.read_csv(cmd_file)
    meas_df = pd.read_csv(meas_file)

    # Extract columns
    t_cmd = cmd_df["Time"].values
    u = cmd_df["TorqueCmd"].values

    t_meas = meas_df["Time"].values
    y = meas_df["Torque"].values

    # Create common time base
    t_start = max(t_cmd.min(), t_meas.min())
    t_end = min(t_cmd.max(), t_meas.max())

    dt = np.median(np.diff(t_cmd))
    t_common = np.arange(t_start, t_end, dt)

    # Interpolate signals
    u_interp = interp1d(t_cmd, u, fill_value="extrapolate")
    y_interp = interp1d(t_meas, y, fill_value="extrapolate")

    u_resampled = u_interp(t_common)
    y_resampled = y_interp(t_common)

    return t_common, u_resampled, y_resampled


def estimate_transfer(t, u, y, nperseg=2048):
    dt = np.mean(np.diff(t))
    fs = 1 / dt

    f, Puu = welch(u, fs=fs, nperseg=nperseg)
    f, Pyu = csd(y, u, fs=fs, nperseg=nperseg)

    H = Pyu / Puu

    mag_db = 20 * np.log10(np.abs(H))
    phase_rad = np.angle(H) / math.pi

    f_coh, coh = coherence(u, y, fs=fs, nperseg=nperseg)

    return f, mag_db, phase_rad, coh


def plot_bode(f, mag_db, phase_rad, coh, max_freq):
    fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

    # Highlight trusted frequency range with light shading
    for ax in axs:
        ax.axvspan(f[0], max_freq, color='lightgreen', alpha=0.3)

    # Magnitude plot
    axs[0].semilogx(f, mag_db, label="Magnitude")
    axs[0].set_ylabel("Magnitude (dB)")
    axs[0].grid(True, which="both")

    # Phase plot (in radians)
    axs[1].semilogx(f, phase_rad, label="Phase")
    axs[1].set_ylabel("Phase (rad)")
    axs[1].grid(True, which="both")

    # Coherence plot
    axs[2].semilogx(f, coh, label="Coherence")
    axs[2].set_ylabel("Coherence")
    axs[2].set_xlabel("Frequency (Hz)")
    axs[2].set_ylim(0, 1)
    axs[2].grid(True, which="both")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    cmd_file = "/home/dfki.uni-bremen.de/adanzglock/Temp/Adrian/testbench_logs/02.03/20260302_130222_Cubemars.csv"
    meas_file = "/home/dfki.uni-bremen.de/adanzglock/Temp/Adrian/testbench_logs/02.03/20260302_140222_HilsherData.csv"

    t, u, y = load_and_align(cmd_file, meas_file)

    f, mag_db, phase_rad, coh = estimate_transfer(t, u, y)
    max_freq = 100  # Hz

    plot_bode(f, mag_db, phase_rad, coh, max_freq)