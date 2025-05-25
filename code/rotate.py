import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load CSV
df = pd.read_csv("code/rotate_45_360.csv")

print(df.columns.tolist())

# Constants
L = 60.0  # cm distance between two points (rear and front)
r = L / 2  # Radius of rotation (centered between the two points)

# Estimate actual yaw from front and rear displacement
def estimate_yaw(row):
    # Compute displacement vector from rear and front point
    rear = np.array([row['dx1_measured'], row['dy1_measured']])
    front = np.array([row['dx2_measured'], row['dy2_measured']])
    
    # Vector difference (from rear to front)
    measured_vec = front - rear

    # Original vector is [0, 60]
    original_vec = np.array([0, 60])
    
    # Compute angle between the two vectors
    dot = np.dot(original_vec, measured_vec)
    cross = np.cross(original_vec, measured_vec)
    
    angle_rad = np.arctan2(cross, dot)  # Signed angle
    return angle_rad  # radians

# Compute expected yaw
df['dyaw_expected'] = df.apply(estimate_yaw, axis=1)

# Estimate center displacement as average of front/rear
df['dx_expected'] = (df['dx1_measured'] + df['dx2_measured']) / 2
df['dy_expected'] = (df['dy1_measured'] + df['dy2_measured']) / 2

# ---- PLOTS ----

# 1. dx
plt.figure(figsize=(10, 3))
plt.subplot(1, 3, 1)
plt.plot(df['dx_expected'], label='Expected')
plt.plot(df['dx_odom'], label='Odometry')
plt.title('dx comparison')
plt.xlabel('Trial')
plt.ylabel('dx (cm)')
plt.legend()

# 2. dy
plt.subplot(1, 3, 2)
plt.plot(df['dy_expected'], label='Expected')
plt.plot(df['dy_odom'], label='Odometry')
plt.title('dy comparison')
plt.xlabel('Trial')
plt.ylabel('dy (cm)')
plt.legend()

# 3. dyaw
plt.subplot(1, 3, 3)
plt.plot(np.degrees(df['dyaw_expected']), label='Expected')
plt.plot(np.degrees(df['dyaw_odom']), label='Odometry')
plt.title('dyaw comparison')
plt.xlabel('Trial')
plt.ylabel('Yaw (°)')
plt.legend()

plt.tight_layout()
plt.show()
