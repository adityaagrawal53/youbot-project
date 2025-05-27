import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def compute_pose_errors_and_print(df, rotation_deg, label):
    theta = np.radians(rotation_deg)
    
    expected_left = np.array([30 * np.cos(theta), 30 * np.sin(theta)])
    expected_right = np.array([-30 * np.cos(theta), -30 * np.sin(theta)])
    expected_center = (expected_left + expected_right) / 2
    expected_yaw = theta % (2 * np.pi)
    
    x_errors = []
    y_errors = []
    yaw_errors = []
    
    print(f"\nResults for {label} (Rotation: {rotation_deg} deg):")
    print(f"{'Row':<5} {'Center_X':>10} {'Center_Y':>10} {'Yaw_rad':>10} {'Yaw_deg':>10} "
          f"{'X_Error':>10} {'Y_Error':>10} {'Yaw_Error_deg':>15} "
          f"{'dx_odom':>8} {'dy_odom':>8} {'dz_odom':>8} {'dyaw_odom':>10}")
    
    for idx, row in df.iterrows():
        actual_left = expected_left + np.array([row['dx1_measured'], row['dy1_measured']])
        actual_right = expected_right + np.array([row['dx2_measured'], row['dy2_measured']])
        center = (actual_left + actual_right) / 2
        
        vector_left_to_right = actual_left - actual_right
        yaw_rad = np.arctan2(vector_left_to_right[1], vector_left_to_right[0])
        
        # Compute errors
        x_err = center[0] - expected_center[0]
        y_err = center[1] - expected_center[1]
        yaw_diff = yaw_rad - expected_yaw
        yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi  # wrap to [-pi, pi]
        
        x_errors.append(x_err)
        y_errors.append(y_err)
        yaw_errors.append(yaw_diff)
        
        print(f"{idx:<5} {center[0]:10.3f} {center[1]:10.3f} {yaw_rad:10.3f} {np.degrees(yaw_rad):10.2f} "
      f"{x_err:10.3f} {y_err:10.3f} {np.degrees(yaw_diff):15.3f} "
      f"{row['dx_odom']:8.3f} {row['dy_odom']:8.3f} {row['dz_odom']:8.3f} {np.degrees(row['dyaw_odom']):10.2f}")

    
    # Summary statistics
    print(f"\nMean X Error: {np.mean(x_errors):.4f}, Variance: {np.var(x_errors):.4f}")
    print(f"Mean Y Error: {np.mean(y_errors):.4f}, Variance: {np.var(y_errors):.4f}")
    print(f"Mean Yaw Error (deg): {np.degrees(np.mean(yaw_errors)):.4f}, Variance (deg^2): {np.degrees(np.var(yaw_errors))**2:.4f}")
    
    return np.array(x_errors), np.array(y_errors), np.array(yaw_errors)

def plot_errors_boxplots(file_info_list):
    x_err_all = []
    y_err_all = []
    yaw_err_all = []
    labels = []
    
    for filename, rotation_deg, label in file_info_list:
        df = pd.read_csv(filename)
        x_err, y_err, yaw_err = compute_pose_errors_and_print(df, rotation_deg, label)
        x_err_all.append(x_err)
        y_err_all.append(y_err)
        yaw_err_all.append(np.degrees(yaw_err))
        labels.append(label)
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    
    axes[0].boxplot(x_err_all, labels=labels)
    axes[0].set_title('X Deviation')
    axes[0].set_ylabel('Error (units)')
    
    axes[1].boxplot(y_err_all, labels=labels)
    axes[1].set_title('Y Deviation')
    axes[1].set_ylabel('Error (units)')
    
    axes[2].boxplot(yaw_err_all, labels=labels)
    axes[2].set_title('Yaw Deviation')
    axes[2].set_ylabel('Error (degrees)')
    
    plt.suptitle('Robot Pose Deviations Across Different Conditions')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

# Example usage:
files = [
    ('code/rotate_45_45.csv', 45, '45 deg @ Speed A'),
    ('code/rotate_75_45.csv', 45, '45 deg @ Speed A'),
    ('code/rotate_45_90.csv', 90, '90 deg @ Speed B'),
    ('code/rotate_75_90.csv', 90, '90 deg @ Speed B'),
    ('code/rotate_45_180.csv', 180, '180 deg @ Speed B'),
    ('code/rotate_75_180.csv', 180, '180 deg @ Speed B'),
    ('code/rotate_45_360.csv', 360, '360 deg @ Speed C'),
    ('code/rotate_75_360.csv', 360, '360 deg @ Speed C'),
]

plot_errors_boxplots(files)
