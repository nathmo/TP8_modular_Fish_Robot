import os
import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

FOLDER_PATH = "."  # Current folder

# Regex to extract float values from filename
FILENAME_PATTERN = re.compile(
    r"robot_position_\d+_\d+_\d+_\d+_\d+_\d+_freq_([-+]?\d*\.\d+|\d+)"
    r"_amp_([-+]?\d*\.\d+|\d+)_lag_([-+]?\d*\.\d+|\d+)_off_([-+]?\d*\.\d+|\d+)\.csv"
)

# Store lag and speed for speed plot
speed_data = []

plt.figure(figsize=(10, 6))

for filename in os.listdir(FOLDER_PATH):
    if filename.endswith(".csv"):
        print("found a CSV")
        print(filename)
        match = FILENAME_PATTERN.match(filename)
        if match:
            freq, amp, lag, offset = match.groups()
            label = f"f:{freq}Hz, A:{amp}, φ:{lag}, δ:{offset}"
            filepath = os.path.join(FOLDER_PATH, filename)
            df = pd.read_csv(filepath)

            # Plot trajectory
            plt.plot(df["X"], df["Y"], label=label, linewidth=1)

            # Compute speed in selected window (between t0+3000ms and t0+5000ms)
            t_start = df["Timestamp"].iloc[0]
            t1 = t_start + 3000
            t2 = t_start + 5000
            window_df = df[(df["Timestamp"] >= t1) & (df["Timestamp"] <= t2)].copy()

            if len(window_df) > 1:
                # Compute distance between consecutive points
                dx = window_df["X"].diff()
                dy = window_df["Y"].diff()
                dt = window_df["Timestamp"].diff() / 1000  # convert to seconds
                inst_speed = np.sqrt(dx**2 + dy**2) / dt
                avg_speed = inst_speed[1:].mean()  # skip NaN
                speed_data.append((float(lag), avg_speed))
        else:
            print(f"⚠️ Could not parse: {filename}")

# Save trajectory plot
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Robot Trajectories Over Time")
plt.xlim(0, 6)
plt.ylim(0, 2)
plt.legend(fontsize='small', loc='upper right')
plt.grid(True)
plt.tight_layout()
plt.savefig("test.png")
plt.close()

# --- Speed vs Lag Plot ---
if speed_data:
    speed_data.sort(key=lambda x: x[0])  # sort by lag
    lags, speeds = zip(*speed_data)

    plt.figure(figsize=(8, 5))
    plt.plot(lags, speeds, 'o-', color='orange')
    plt.xlabel("Lag φ")
    plt.ylabel("Average Speed (m/s)")
    plt.title("Average Speed vs Lag (from 3s to 5s)")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("speed.png")
    plt.close()
else:
    print("⚠️ No valid speed data found for speed.png.")
