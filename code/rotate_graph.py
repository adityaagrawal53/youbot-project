import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from glob import glob

# === Config toggles ===
PRINT_COLUMN_DATA = True
PRINT_STATS = True
SAVE_FIG = True
SHOW_FIG = False

CSV_DIR = 'code/csv/'  # Adjust as needed
DATA_DIR = 'code/images/'

plt.style.use('fivethirtyeight')

def compute_pose_errors_and_print(df, rotation_deg, speed):
    theta = np.radians(rotation_deg)
    expected_left = np.array([30 * np.cos(theta), 30 * np.sin(theta)])
    expected_right = np.array([-30 * np.cos(theta), -30 * np.sin(theta)])
    expected_center = (expected_left + expected_right) / 2
    expected_yaw = theta % (2 * np.pi)

    x_errors, y_errors, yaw_errors = [], [], []

    if PRINT_COLUMN_DATA:
        print(f"\nResults for {speed} deg/s, {rotation_deg} deg:")
        print(f"{'Row':<5} {'Center_X':>10} {'Center_Y':>10} {'Yaw_deg':>10} "
              f"{'dx_odom':>8} {'dy_odom':>8} {'dyaw_odom_deg':>14}")

    for idx, row in df.iterrows():
        actual_left = expected_left + np.array([row['dx1_measured'], row['dy1_measured']])
        actual_right = expected_right + np.array([row['dx2_measured'], row['dy2_measured']])
        center = (actual_left + actual_right) / 2

        vector_left_to_right = actual_left - actual_right
        yaw_rad = np.arctan2(vector_left_to_right[1], vector_left_to_right[0])
        yaw_diff = (yaw_rad - expected_yaw + np.pi) % (2 * np.pi) - np.pi

        x_err = center[0] - expected_center[0]
        y_err = center[1] - expected_center[1]

        x_errors.append(x_err)
        y_errors.append(y_err)
        yaw_errors.append(yaw_diff)

        if PRINT_COLUMN_DATA:
            print(f"{idx:<5} {center[0]:10.3f} {center[1]:10.3f} {np.degrees(yaw_rad):10.2f} "
                  f"{row['dx_odom']:8.3f} {row['dy_odom']:8.3f} {np.degrees(row['dyaw_odom']):14.2f}")

    if PRINT_STATS:
        mean_yaw_deg = np.degrees(np.mean(yaw_errors))
        var_yaw_deg = np.var(np.degrees(yaw_errors))
        print(f"\nStatistics for {speed} deg/s, {rotation_deg} deg:")
        print(f"  Mean X Error: {np.mean(x_errors):.4f}, Variance: {np.var(x_errors):.4f}")
        print(f"  Mean Y Error: {np.mean(y_errors):.4f}, Variance: {np.var(y_errors):.4f}")
        print(f"  Mean Yaw Error (deg): {mean_yaw_deg:.4f}, Variance (deg²): {var_yaw_deg:.4f}")

    return np.array(x_errors), np.array(y_errors), np.array(yaw_errors)

def plot_rotational_errors_by_speed():
    file_list = sorted(glob(os.path.join(CSV_DIR, 'rotate_*.csv')))

    grouped_data = {}

    for file_path in file_list:
        filename = os.path.splitext(os.path.basename(file_path))[0]
        try:
            _, speed_str, deg_str = filename.split('_')
            speed = int(speed_str)
            rotation_deg = int(deg_str)
        except ValueError:
            print(f"Skipping invalid filename format: {filename}")
            continue

        df = pd.read_csv(file_path)
        x_err, y_err, yaw_err = compute_pose_errors_and_print(df, rotation_deg, speed)

        grouped_data.setdefault(speed, []).append((rotation_deg, x_err, y_err, np.degrees(yaw_err)))

    for speed in sorted(grouped_data.keys()):
        group = sorted(grouped_data[speed], key=lambda x: x[0])

        x_err_all = [item[1] for item in group]
        y_err_all = [item[2] for item in group]
        yaw_err_all = [item[3] for item in group]
        labels = [f"{item[0]}°" for item in group]

        fig, axes = plt.subplots(1, 3, figsize=(18, 6))

        axes[0].boxplot(x_err_all, labels=labels)
        axes[0].set_title('X Deviation')
        axes[0].set_ylabel('Error (cm)')
        axes[0].tick_params(axis='x', rotation=45, labelsize=9)

        axes[1].boxplot(y_err_all, labels=labels)
        axes[1].set_title('Y Deviation')
        axes[1].set_ylabel('Error (cm)')
        axes[1].tick_params(axis='x', rotation=45, labelsize=9)

        axes[2].boxplot(yaw_err_all, labels=labels)
        axes[2].set_title('Yaw Deviation')
        axes[2].set_ylabel('Error (degrees)')
        axes[2].tick_params(axis='x', rotation=45, labelsize=9)

        plt.suptitle(f'Robot Pose Deviations - Speed: {speed:.0f} deg/s')
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        if SAVE_FIG:
            if not os.path.exists(DATA_DIR):
                os.makedirs(DATA_DIR)
            filename = f'rotational_errors_speed_{int(speed):02d}.png'
            plt.savefig(os.path.join(DATA_DIR, filename), dpi=300)

        if SHOW_FIG:
            plt.show()
        else:
            plt.close()

if __name__ == "__main__":
    plot_rotational_errors_by_speed()
