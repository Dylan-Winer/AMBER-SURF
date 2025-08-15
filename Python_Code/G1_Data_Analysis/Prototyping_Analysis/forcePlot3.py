#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import find_peaks, savgol_filter
import os
from matplotlib import animation

# ==== USER SETTINGS ====
file_path = r"C:\Users\dywin\Documents\Caltech\Data\CoolTerm\G1_Tests\g1Test-Walk.txt"

# Force peak tuning (adjust these to be more/less picky)
min_prom_N       = 10.0    # how much a peak must stand out from neighbors (N)
min_height_N     = 15.0    # absolute minimum peak height (N)
min_distance_s   = 0.35    # minimum time between peaks (seconds)
min_width_s      = 0.05    # minimum peak width at half-prominence (seconds)

# Light smoothing for detection stability (not for plotting)
savgol_win_s     = 0.025   # window length in seconds (convert to odd samples)
savgol_poly      = 2       # polynomial order
# =======================

# Columns expected in the new CSV:
use_cols = [
    "time_s", "FSR1_N", "FSR2_N", "FSR3_N", "FSR4_N",
    "qw", "qx", "qy", "qz",
    "gx_rad_s", "gy_rad_s", "gz_rad_s"
]

# Read CSV
df = pd.read_csv(file_path, comment="#", skip_blank_lines=True)

# If file has all expected columns, reorder
if all(c in df.columns for c in use_cols):
    df = df[use_cols]
else:
    needed = ["time_s", "FSR1_N", "FSR2_N", "FSR3_N", "FSR4_N",
              "qw", "qx", "qy", "qz",
              "gx_rad_s", "gy_rad_s", "gz_rad_s"]
    missing = [c for c in needed if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}")

# Convert to numeric
for c in df.columns:
    df[c] = pd.to_numeric(df[c], errors="coerce")

# Drop NaNs in time
df = df.dropna(subset=["time_s"])

# --- Filter by time range (35–40 seconds) ---
df = df[(df["time_s"] >= 35) & (df["time_s"] <= 40)].reset_index(drop=True)

# --- Forces ---
force_cols = ["FSR1_N", "FSR2_N", "FSR3_N", "FSR4_N"]
df["FSRsum_N"] = df[force_cols].sum(axis=1)

# ---- Derive sampling info from time_s ----
if len(df) >= 2:
    dt = np.median(np.diff(df["time_s"]))
    fs = 1.0 / dt if dt > 0 else 1.0  # Hz
else:
    dt = 0.0
    fs = 1.0

def _odd(n):
    """Ensure a positive odd integer >= 5 and >= poly+2."""
    n = max(n, savgol_poly + 2)
    n = int(np.ceil(n))
    if n % 2 == 0:
        n += 1
    return max(n, 5)

distance_samples = max(1, int(round(min_distance_s * fs)))
width_samples    = max(1, int(round(min_width_s * fs)))
savgol_win       = _odd(int(round(savgol_win_s * fs)))

# --- Peak detection (run on smoothed copy; mark on raw) ---
series_for_peaks = ["FSR1_N", "FSR2_N", "FSR3_N", "FSR4_N", "FSRsum_N"]
peaks = {}
for k in series_for_peaks:
    y = df[k].to_numpy()
    if len(y) >= savgol_win:
        y_smooth = savgol_filter(y, window_length=savgol_win, polyorder=savgol_poly, mode="interp")
    else:
        y_smooth = y  # too short to smooth safely

    use_height = min_height_N

    idx, props = find_peaks(
        y_smooth,
        prominence=min_prom_N,
        height=use_height,
        distance=distance_samples,
        width=width_samples
    )
    peaks[k] = idx
    print(f"{k}: {len(idx)} peaks (fs≈{fs:.2f} Hz, distance≥{distance_samples} samp, width≥{width_samples} samp)")

# --- 3D VECTOR MOMENTS: r x F (N·m) ---
# r vectors from ankle to each FSR (meters)
r1 = np.array([-0.05461, -0.02667, 0.0])
r2 = np.array([-0.05461,  0.02667, 0.0])
r3 = np.array([ 0.13100, -0.02667, 0.0])
r4 = np.array([ 0.13100,  0.02667, 0.0])

# Force vectors are along +Z: F_i_vec = (0, 0, F_i)
F1 = df["FSR1_N"].to_numpy()
F2 = df["FSR2_N"].to_numpy()
F3 = df["FSR3_N"].to_numpy()
F4 = df["FSR4_N"].to_numpy()

# Cross products r x F; with F=(0,0,F) -> (ry*F, -rx*F, 0)
def rxF_components(r, Fz):
    rx, ry, rz = r
    Mx = ry * Fz
    My = -rx * Fz
    Mz = np.zeros_like(Fz)
    return Mx, My, Mz

Mx1, My1, Mz1 = rxF_components(r1, F1)
Mx2, My2, Mz2 = rxF_components(r2, F2)
Mx3, My3, Mz3 = rxF_components(r3, F3)
Mx4, My4, Mz4 = rxF_components(r4, F4)

# Store per-sensor vector moments (optional for debugging)
df["Mx_FSR1_Nm"], df["My_FSR1_Nm"], df["Mz_FSR1_Nm"] = Mx1, My1, Mz1
df["Mx_FSR2_Nm"], df["My_FSR2_Nm"], df["Mz_FSR2_Nm"] = Mx2, My2, Mz2
df["Mx_FSR3_Nm"], df["My_FSR3_Nm"], df["Mz_FSR3_Nm"] = Mx3, My3, Mz3
df["Mx_FSR4_Nm"], df["My_FSR4_Nm"], df["Mz_FSR4_Nm"] = Mx4, My4, Mz4

# Heel and Toe vector sums
df["Mx_heel_Nm"] = df["Mx_FSR1_Nm"] + df["Mx_FSR2_Nm"]
df["My_heel_Nm"] = df["My_FSR1_Nm"] + df["My_FSR2_Nm"]
df["Mz_heel_Nm"] = df["Mz_FSR1_Nm"] + df["Mz_FSR2_Nm"]

df["Mx_toe_Nm"]  = df["Mx_FSR3_Nm"] + df["Mx_FSR4_Nm"]
df["My_toe_Nm"]  = df["My_FSR3_Nm"] + df["My_FSR4_Nm"]
df["Mz_toe_Nm"]  = df["Mz_FSR3_Nm"] + df["Mz_FSR4_Nm"]

# Total vector moment components (sum of all sensors)
df["Mx_total_Nm"] = df[["Mx_FSR1_Nm","Mx_FSR2_Nm","Mx_FSR3_Nm","Mx_FSR4_Nm"]].sum(axis=1)
df["My_total_Nm"] = df[["My_FSR1_Nm","My_FSR2_Nm","My_FSR3_Nm","My_FSR4_Nm"]].sum(axis=1)
df["Mz_total_Nm"] = df[["Mz_FSR1_Nm","Mz_FSR2_Nm","Mz_FSR3_Nm","Mz_FSR4_Nm"]].sum(axis=1)

# --- Magnitudes of heel, toe, and total moment vectors ---
df["Mmag_heel_Nm"]  = np.sqrt(df["Mx_heel_Nm"]**2  + df["My_heel_Nm"]**2  + df["Mz_heel_Nm"]**2)
df["Mmag_toe_Nm"]   = np.sqrt(df["Mx_toe_Nm"]**2   + df["My_toe_Nm"]**2   + df["Mz_toe_Nm"]**2)
df["Mmag_total_Nm"] = np.sqrt(df["Mx_total_Nm"]**2 + df["My_total_Nm"]**2 + df["Mz_total_Nm"]**2)

# --- Euler angles (roll, pitch, yaw) from quaternion (degrees) ---
# Convention: intrinsic Z-Y-X (yaw-pitch-roll). roll about X, pitch about Y, yaw about Z.
qw = df["qw"].to_numpy()
qx = df["qx"].to_numpy()
qy = df["qy"].to_numpy()
qz = df["qz"].to_numpy()

# Normalize (stability)
norm = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
norm[norm == 0] = 1.0
qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm

# Radians first
roll_rad  = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
pitch_rad = np.arcsin(np.clip(2*(qw*qy - qz*qx), -1.0, 1.0))
yaw_rad   = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

# --- Unwrap roll to remove ±π jumps, then (optionally) smooth ---
roll_unwrapped_rad = np.unwrap(roll_rad)

# Convert to degrees
df["roll_deg"]  = np.degrees(roll_unwrapped_rad)
df["pitch_deg"] = np.degrees(pitch_rad)  # typically within [-90, 90]
df["yaw_deg"]   = np.degrees(yaw_rad)

# Optional light smoothing (on continuous roll)
unwrap_win_s = 0.05  # ~50 ms window; tune as needed
unwrap_win   = _odd(int(round(unwrap_win_s * fs)))
if len(df) >= unwrap_win:
    df["roll_deg"] = savgol_filter(df["roll_deg"], unwrap_win, 2, mode="interp")

print(df.head())
print("\nSamples in 35–40 s window:", len(df))

# --- Colors ---
colors = {
    "FSR1_N": "tab:blue",
    "FSR2_N": "tab:orange",
    "FSR3_N": "tab:green",
    "FSR4_N": "tab:red",
    "FSRsum_N": "black",
    "gx_rad_s": "tab:blue",
    "gy_rad_s": "tab:orange",
    "gz_rad_s": "tab:green",
    "qw": "tab:purple",
    "qx": "tab:brown",
    "qy": "tab:pink",
    "qz": "tab:gray",
    "Mx_total_Nm": "tab:blue",
    "My_total_Nm": "tab:orange",
    "Mz_total_Nm": "tab:green",
    "Mmag_heel_Nm": "tab:purple",
    "Mmag_toe_Nm": "tab:cyan",
    "Mmag_total_Nm": "black",
    "roll_deg": "tab:blue",
    "pitch_deg": "tab:orange",
    "yaw_deg": "tab:green",
}

# --- Plotting FSR forces ---
plt.figure(figsize=(10, 6))
for k in ["FSR1_N", "FSR2_N", "FSR3_N", "FSR4_N"]:
    plt.plot(df["time_s"], df[k], label=k.replace("_N", ""), color=colors[k], linewidth=1.8)
    if len(peaks[k]) > 0:
        plt.plot(df["time_s"].iloc[peaks[k]], df[k].iloc[peaks[k]], "x",
                 color=colors[k], markersize=8)

plt.plot(df["time_s"], df["FSRsum_N"], label="FSR Sum",
         color=colors["FSRsum_N"], linewidth=2.4, linestyle="--")
if len(peaks["FSRsum_N"]) > 0:
    plt.plot(df["time_s"].iloc[peaks["FSRsum_N"]],
             df["FSRsum_N"].iloc[peaks["FSRsum_N"]],
             "^", color=colors["FSRsum_N"], markersize=8)

plt.title("FSR Force vs Time (35–40 s) with Strict Peaks")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.grid(True, alpha=0.3)
plt.legend(ncol=2)
plt.tight_layout()

# --- Plotting angular velocities (rad/s) ---
plt.figure(figsize=(10, 6))
plt.plot(df["time_s"], df["gx_rad_s"], label="Gyro X (rad/s)", color=colors["gx_rad_s"], linewidth=1.8)
plt.plot(df["time_s"], df["gy_rad_s"], label="Gyro Y (rad/s)", color=colors["gy_rad_s"], linewidth=1.8)
plt.plot(df["time_s"], df["gz_rad_s"], label="Gyro Z (rad/s)", color=colors["gz_rad_s"], linewidth=1.8)
plt.title("Angular Velocity vs Time (35–40 s)")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()

# --- Plotting quaternion components ---
plt.figure(figsize=(10, 6))
for k in ["qw", "qx", "qy", "qz"]:
    plt.plot(df["time_s"], df[k], label=k, linewidth=1.8, color=colors[k])
plt.title("Quaternion Components vs Time (35–40 s)")
plt.xlabel("Time (s)")
plt.ylabel("Value")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()

# --- Plotting TOTAL vector moments (Mx, My, Mz) ---
plt.figure(figsize=(10, 6))
plt.plot(df["time_s"], df["Mx_total_Nm"], label="Mx total (N·m)", linewidth=2.0, color=colors["Mx_total_Nm"])
plt.plot(df["time_s"], df["My_total_Nm"], label="My total (N·m)", linewidth=2.0, color=colors["My_total_Nm"])
plt.plot(df["time_s"], df["Mz_total_Nm"], label="Mz total (N·m)", linewidth=2.0, color=colors["Mz_total_Nm"])
plt.title("Ankle Moment Vector Components vs Time (35–40 s)")
plt.xlabel("Time (s)")
plt.ylabel("Moment (N·m)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()

# --- Plotting magnitudes of heel, toe, and total moments ---
plt.figure(figsize=(10, 6))
plt.plot(df["time_s"], df["Mmag_heel_Nm"],  label="‖M_heel‖ (N·m)",  linewidth=2.2, color=colors["Mmag_heel_Nm"])
plt.plot(df["time_s"], df["Mmag_toe_Nm"],   label="‖M_toe‖ (N·m)",   linewidth=2.2, color=colors["Mmag_toe_Nm"])
plt.plot(df["time_s"], df["Mmag_total_Nm"], label="‖M_total‖ (N·m)", linewidth=2.2, color=colors["Mmag_total_Nm"])
plt.title("Moment Vector Magnitudes vs Time (35–40 s)")
plt.xlabel("Time (s)")
plt.ylabel("‖M‖ (N·m)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()

# --- Plotting Euler angles (roll, pitch, yaw) ---
plt.figure(figsize=(10, 6))
plt.plot(df["time_s"], df["roll_deg"],  label="Roll (°)",  linewidth=1.8, color=colors["roll_deg"])
plt.plot(df["time_s"], df["pitch_deg"], label="Pitch (°)",             linewidth=1.8, color=colors["pitch_deg"])
plt.plot(df["time_s"], df["yaw_deg"],   label="Yaw (°)",               linewidth=1.8, color=colors["yaw_deg"])
plt.title("Euler Angles (Roll, Pitch, Yaw) vs Time (35–40 s)")
plt.xlabel("Time (s)")
plt.ylabel("Angle (°)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()

plt.show()


def make_replay_video(
    df,
    time_col,
    series,
    labels=None,
    colors=None,
    ylabel="",
    title="",
    outfile="out.mp4",
    fps=30,
    ypad=0.05,
    linewidth=1.8,
    dpi=150,
):
    t = df[time_col].to_numpy().astype(float)
    t = t - t[0]
    duration = float(t[-1])
    n_frames = int(np.floor(duration * fps)) + 1
    frame_times = np.linspace(0.0, duration, n_frames)

    fig, ax = plt.subplots(figsize=(10, 6))

    # Fix y-limits so view doesn't jump
    ymins, ymaxs = [], []
    for s in series:
        y = df[s].to_numpy()
        y = y[np.isfinite(y)]
        ymins.append(float(np.min(y)) if y.size else 0.0)
        ymaxs.append(float(np.max(y)) if y.size else 1.0)
    ylo, yhi = min(ymins), max(ymaxs)
    if yhi == ylo: yhi = ylo + 1.0
    pad = (yhi - ylo) * ypad
    ax.set_ylim(ylo - pad, yhi + pad)
    ax.set_xlim(0, duration)

    if labels is None: labels = series
    line_objs = []
    for i, s in enumerate(series):
        color = None if colors is None else colors.get(s, None)
        line, = ax.plot([], [], label=labels[i], linewidth=linewidth, color=color)
        line_objs.append(line)

    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2)

    # --- writer selection (MP4 if ffmpeg, else GIF) ---
    try:
        has_ffmpeg = animation.writers.is_available("ffmpeg")
    except Exception:
        has_ffmpeg = bool(_ffmpeg_path) or (which("ffmpeg") is not None)

    if has_ffmpeg:
        writer = animation.FFMpegWriter(
            fps=fps, metadata={'title': title},
            codec='libx264', bitrate=4000,
            extra_args=['-pix_fmt', 'yuv420p']
        )
        if not outfile.lower().endswith(".mp4"):
            base, _ = os.path.splitext(outfile)
            outfile = base + ".mp4"
    else:
        writer = animation.PillowWriter(fps=fps)
        if not outfile.lower().endswith(".gif"):
            base, _ = os.path.splitext(outfile)
            outfile = base + ".gif"

    os.makedirs(os.path.dirname(outfile) or ".", exist_ok=True)
    with writer.saving(fig, outfile, dpi=dpi):
        for ft in frame_times:
            idx = int(np.searchsorted(t, ft, side="right"))
            for line, s in zip(line_objs, series):
                y = df[s].to_numpy()
                line.set_data(t[:idx], y[:idx])
            fig.canvas.draw()
            writer.grab_frame()

    plt.close(fig)
    print(f"Saved: {outfile}")


# ======== MAKE VIDEOS ========
# Choose an output folder
out_dir = r"C:\Users\dywin\Documents\Caltech\Data\CoolTerm\videos"

# 1) Forces
make_replay_video(
    df,
    time_col="time_s",
    series=["FSR1_N","FSR2_N","FSR3_N","FSR4_N","FSRsum_N"],
    labels=["FSR1","FSR2","FSR3","FSR4","FSR Sum"],
    colors=None,  # or your existing `colors` dict
    ylabel="Force (N)",
    title="FSR Force vs Time (Replay)",
    outfile=os.path.join(out_dir, "forces_replay.mp4"),
    fps=30,
)

# 2) Moment components (TOTAL Mx/My/Mz)
make_replay_video(
    df,
    time_col="time_s",
    series=["Mx_total_Nm","My_total_Nm","Mz_total_Nm"],
    labels=["Mx total","My total","Mz total"],
    colors=None,
    ylabel="Moment (N·m)",
    title="Ankle Moment Components (Replay)",
    outfile=os.path.join(out_dir, "moments_components_replay.mp4"),
    fps=30,
)

# 3) Moment magnitudes (heel, toe, total)
make_replay_video(
    df,
    time_col="time_s",
    series=["Mmag_heel_Nm","Mmag_toe_Nm","Mmag_total_Nm"],
    labels=["‖M_heel‖","‖M_toe‖","‖M_total‖"],
    colors=None,
    ylabel="‖M‖ (N·m)",
    title="Moment Magnitudes (Replay)",
    outfile=os.path.join(out_dir, "moments_magnitude_replay.mp4"),
    fps=30,
)

# 4) Euler angles (roll, pitch, yaw)
make_replay_video(
    df,
    time_col="time_s",
    series=["roll_deg","pitch_deg","yaw_deg"],
    labels=["Roll (°) — unwrapped","Pitch (°)","Yaw (°)"],
    colors=None,
    ylabel="Angle (°)",
    title="Euler Angles (Replay)",
    outfile=os.path.join(out_dir, "euler_replay.mp4"),
    fps=30,
)
