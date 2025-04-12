import os
import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

FOLDER_PATH = "."  # Current folder

# Updated regex to also capture date and time components
FILENAME_PATTERN = re.compile(
    r"robot_position_(\d+)_(\d+)_(\d+)_(\d+)_(\d+)_(\d+)_freq_([-+]?\d*\.\d+|\d+)"
    r"_amp_([-+]?\d*\.\d+|\d+)_lag_([-+]?\d*\.\d+|\d+)_off_([-+]?\d*\.\d+|\d+)\.csv"
)

# Store speeds grouped by lag value
lag_speeds = defaultdict(list)

plt.figure(figsize=(10, 6))

for filename in os.listdir(FOLDER_PATH):
    if filename.endswith(".csv"):
        print("found a CSV")
        print(filename)
        match = FILENAME_PATTERN.match(filename)
        if match:
            (
                year,
                month,
                day,
                hour,
                minute,
                second,
                freq,
                amp,
                lag,
                offset,
            ) = match.groups()
            time_str = f"{hour}:{minute}:{second}"
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

                # Group speeds by lag value
                lag_float = float(lag)
                lag_speeds[lag_float].append(avg_speed)
        else:
            print(f"⚠️ Could not parse: {filename}")

# Save trajectory plot
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Robot Trajectories Over Time")
plt.xlim(0, 6)
plt.ylim(0, 2)
plt.legend(fontsize="small", loc="upper right")
plt.grid(True)
plt.tight_layout()
plt.savefig("trajectory.svg", format="svg")
plt.close()

# --- Speed vs Lag Plot with error bars ---
if lag_speeds:
    # Calculate mean and std for each lag value
    lags = sorted(lag_speeds.keys())
    mean_speeds = [np.mean(lag_speeds[lag]) for lag in lags]
    std_speeds = [np.std(lag_speeds[lag]) for lag in lags]

    # Count number of samples for each lag
    sample_counts = [len(lag_speeds[lag]) for lag in lags]

    plt.figure(figsize=(10, 6))

    # Plot with error bars
    plt.errorbar(
        lags,
        mean_speeds,
        yerr=std_speeds,
        fmt="o-",
        color="orange",
        ecolor="gray",
        capsize=5,
        label="Mean ± Std Dev",
    )

    # Annotate with sample count
    for i, (lag, mean, count) in enumerate(zip(lags, mean_speeds, sample_counts)):
        plt.annotate(
            f"n={count}",
            (lag, mean),
            textcoords="offset points",
            xytext=(0, 8),
            ha="center",
            fontsize=8,
        )

    plt.xlabel("Lag φ")
    plt.ylabel("Average Speed (m/s)")
    plt.title("Average Speed vs Lag with Standard Deviation")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("speed_with_std.svg", format="svg")
    plt.close()
else:
    print("⚠️ No valid speed data found for speed plot.")
