import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import csd, welch, coherence
from scipy.interpolate import interp1d
import matplotlib

def load_and_align(cmd_file, meas_file):
    # Load CSV files
    cmd_df = pd.read_csv(cmd_file)
    meas_df = pd.read_csv(meas_file)

    # Extract columns
    t_cmd = cmd_df["Time"].values
    u = cmd_df["Torque (Nm)"].values

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


def plot_bode(f, mag_db, phase_rad, coh, max_freq, name="Chirp"):
    fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    fig.suptitle(name)

    # Highlight trusted frequency range with light shading
    for ax in axs:
        ax.axvspan(f[0], max_freq, color='lightgreen', alpha=0.3)

    # Find the first frequency where magnitude <= -3 dB
    idx_3db = np.where(mag_db <= -3)[0]
    if len(idx_3db) > 0:
        f_3db = f[idx_3db[0]]
    else:
        f_3db = None  # No crossing found

    # Magnitude plot
    axs[0].semilogx(f, mag_db, label="Magnitude")
    if f_3db is not None:
        axs[0].axvline(f_3db, color='red', linestyle='--', label=f'-3 dB at {f_3db:.2f} Hz')
    axs[0].set_ylabel("Magnitude (dB)")
    axs[0].grid(True, which="both")
    axs[0].legend()

    # Phase plot
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

def plot_trajectory(torque, velocity, dt):
    matplotlib.use('TkAgg')
    time = np.arange(0, len(torque) * dt, dt)
    plt.figure(figsize=(10, 5))
    plt.plot(time, torque, label='Torque (Nm)', marker='o')
    plt.plot(time, velocity, label='Velocity (rpm)', marker='x')
    plt.xlabel('Time (s)')
    plt.ylabel('Values')
    plt.title('Trajectory')
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    cmd_file = "/home/testbench/Documents/CubeMarsMotors/ros_ws/src/cubemars_driver_node/docker/mtb-data/20260313_133340_Cubemars.csv"
    meas_file = "/home/testbench/mtb-data/20260313_143340_HilsherData.csv"

    t, u, y = load_and_align(cmd_file, meas_file)

    f, mag_db, phase_rad, coh = estimate_transfer(t, u, y)
    max_freq = 500  # Hz

    plot_bode(f, mag_db, phase_rad, coh, max_freq)