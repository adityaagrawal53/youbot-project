import pandas as pd
import numpy as np

def compute_robot_pose_with_errors(df, rotation_deg):
    theta = np.radians(rotation_deg)

    # Expected positions of the two ends after rotation
    # Left end corresponds to +30 on x-axis before rotation
    expected_left = np.array([30 * np.cos(theta), 30 * np.sin(theta)])   
    # Right end corresponds to -30 on x-axis before rotation
    expected_right = np.array([-30 * np.cos(theta), -30 * np.sin(theta)]) 

    # Print header for output columns
    print(f"{'Row':<5} {'Center_X':>10} {'Center_Y':>10} {'Yaw_rad':>10} {'Yaw_deg':>10} "
          f"{'Left_X':>10} {'Left_Y':>10} {'Right_X':>10} {'Right_Y':>10}")

    for idx, row in df.iterrows():
        # Extract measured deviations/errors from expected points
        error_left_x, error_left_y = row['dx1_measured'], row['dy1_measured']
        error_right_x, error_right_y = row['dx2_measured'], row['dy2_measured']

        # Calculate actual measured positions by adding errors to expected positions
        actual_left = expected_left + np.array([error_left_x, error_left_y])
        actual_right = expected_right + np.array([error_right_x, error_right_y])

        # Compute robot center as midpoint between left and right ends
        center = (actual_left + actual_right) / 2

        # Calculate vector from right end to left end
        vector_left_to_right = actual_left - actual_right

        # Compute robot yaw angle (orientation) from this vector
        yaw_rad = np.arctan2(vector_left_to_right[1], vector_left_to_right[0])
        if yaw_rad < 1:
            yaw_rad += 2 * np.pi
        yaw_deg = np.degrees(yaw_rad)

        # Print results formatted nicely
        print(f"{idx:<5} {center[0]:10.3f} {center[1]:10.3f} {yaw_rad:10.3f} {yaw_deg:10.2f} "
              f"{actual_left[0]:10.3f} {actual_left[1]:10.3f} {actual_right[0]:10.3f} {actual_right[1]:10.3f}")

# Usage example
df = pd.read_csv('code/rotate_75_360.csv')
rotation_angle = 360
compute_robot_pose_with_errors(df, rotation_angle)
