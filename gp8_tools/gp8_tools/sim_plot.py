import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# File names
fn = ["PTPrun1.csv", "LINrun1.csv"]

# File path - to data folder
data_dir = Path(__file__).resolve().parents[2] / "data"

tol = 1e-8

joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]

# Column names
position_cols = [
    "joint_1_s_position_rad",
    "joint_2_l_position_rad",
    "joint_3_u_position_rad",
    "joint_4_r_position_rad",
    "joint_5_b_position_rad",
    "joint_6_t_position_rad",
]

velocity_cols = [
    "joint_1_s_velocity_rad_s",
    "joint_2_l_velocity_rad_s",
    "joint_3_u_velocity_rad_s",
    "joint_4_r_velocity_rad_s",
    "joint_5_b_velocity_rad_s",
    "joint_6_t_velocity_rad_s",
]

# Create one figure for positions and one for velocities
fig_pos, axes_pos = plt.subplots(3, 2, figsize=(14, 7))
fig_vel, axes_vel = plt.subplots(3, 2, figsize=(14, 7))

# Convert matrix of figures to 1D array
axes_pos = axes_pos.flatten()
axes_vel = axes_vel.flatten()


# Clean files and plot
for file in fn:
    df = pd.read_csv(data_dir / file)

    moving_rows = (df[velocity_cols].abs() > tol).any(axis=1)

    if not moving_rows.any():
        print(f"No moving rows found in {file}")
        continue

    first_moving_row = np.flatnonzero(moving_rows.to_numpy())[0]

    # Keep two rows before the first moving row
    start_row = max(first_moving_row - 2, 0)

    df_trimmed = df.iloc[start_row:].reset_index(drop=True)

    time = df_trimmed["time (s)"]
    norm_time = time - time[0]
    
    file_name = file.replace("run1.csv", "")

    for i in range(len(position_cols)):
        position_name = position_cols[i].replace("_position_rad", "")
        velocity_name = velocity_cols[i].replace("_velocity_rad_s", "")

        # Position subplot
        axes_pos[i].plot(
            norm_time,
            df_trimmed[position_cols[i]],
            label=file_name
        )

        # Velocity subplot
        axes_vel[i].plot(
            norm_time,
            df_trimmed[velocity_cols[i]],
            label=file_name
        )

# Format position plots
for i, ax in enumerate(axes_pos):
    ax.set_title(joint_names[i])
    ax.set_ylabel("Position (rad)")
    ax.grid(True)
    ax.legend()

axes_pos[4].set_xlabel("Time (s)")
axes_pos[5].set_xlabel("Time (s)")

fig_pos.suptitle("Joint Positions", fontsize=16)
fig_pos.tight_layout()


# Format velocity plots
for i, ax in enumerate(axes_vel):
    ax.set_title(joint_names[i])
    ax.set_ylabel("Velocity (rad/s)")
    ax.axhline(0, color="black", linewidth=0.8)
    ax.grid(True)
    ax.legend()

axes_vel[4].set_xlabel("Time (s)")
axes_vel[5].set_xlabel("Time (s)")

fig_vel.suptitle("Joint Velocities", fontsize=16)
fig_vel.tight_layout()

plt.show(block=False)

plt.pause(0.5)
input("Press Enter to close...")