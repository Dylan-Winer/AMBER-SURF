#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

# ==== USER SETTINGS ====
file_path = r"C:\Users\dywin\Documents\Caltech\Data\CoolTerm\forceTest1\ForceCapture-1-3.txt"
peak_prominence = 2.0   # Newtons
peak_distance   = 5     # samples
# =======================

# Columns expected in the new CSV:
use_cols = [
    "time_s", "FSR1_N", "FSR2_N", "FSR3_N", "FSR4_N",
    "qw", "qx", "qy", "qz", "angAccX", "angAccY", "angAccZ"
]

# Read CSV
df = pd.read_csv(file_path, comment="#", skip_blank_lines=True)

# If file has all expected columns, reorder
if all(c in df.columns for c in use_cols):
    df = df[use_cols]
else:
    # Require at least force + time + angAcc cols
    needed = ["time_s", "FSR1_N", "FSR2_N", "FSR3_N", "FSR4_N", "angAccX", "angAccY", "angAccZ"]
    missing = [c for c in needed if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}")

# Convert to numeric
for c in df.columns:
    df[c] = pd.to_numeric(df[c], errors="coerce")

# Drop NaNs in time
df = df.dropna(subset=["time_s"])

# Compute sum of forces
force_cols = ["FSR1_N", "FSR2_N", "FSR3_N", "FSR4_N"]
df["FSRsum_N"] = df[force_cols].sum(axis=1)

print(df.head())
print("\nSamples:", len(df))

# --- Peak detection ---
peaks = {}
for k in ["FSR1_N", "FSR2_N", "FSR3_N", "FSR4_N", "FSRsum_N"]:
    y = df[k].to_numpy()
    idx, _ = find_peaks(y, prominence=peak_prominence, distance=peak_distance)
    peaks[k] = idx
    print(f"{k}: {len(idx)} peaks detected")

# --- Colors ---
colors = {
    "FSR1_N": "tab:blue",
    "FSR2_N": "tab:orange",
    "FSR3_N": "tab:green",
    "FSR4_N": "tab:red",
    "FSRsum_N": "black"
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

plt.title("FSR Force vs Time with Peaks")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.grid(True, alpha=0.3)
plt.legend(ncol=2)
plt.tight_layout()

# --- Plotting angular accelerations ---
plt.figure(figsize=(10, 6))
plt.plot(df["time_s"], df["angAccX"], label="Ang Acc X", color="tab:blue", linewidth=1.8)
plt.plot(df["time_s"], df["angAccY"], label="Ang Acc Y", color="tab:orange", linewidth=1.8)
plt.plot(df["time_s"], df["angAccZ"], label="Ang Acc Z", color="tab:green", linewidth=1.8)

plt.title("Angular Acceleration vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Angular Acceleration (deg/s²)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()

plt.show()
