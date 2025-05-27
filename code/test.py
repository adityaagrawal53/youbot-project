import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

def compute_pose_errors(df, rotation_deg):
    theta = np.radians(rotation_deg)

    expected_left = np.array([30 * np.cos(theta), 30 * np.sin(theta)])   
    expected_right = np.array([-30 * np.cos(theta), -30 * np.sin(theta)]) 

    center_x_errors = []
    center_y_errors = []
    yaw_errors_deg = []

    for idx, row in df.iterrows():
        error_left_x, error_left_y = row['dx1_measured'], row['dy1_measured']
        error_right_x, error_right_y = row['dx2_measured'], row['dy2_measured']

        actual_left = expected_left + np.array([error_left_x, error_left_y])
        actual_right = expected_right + np.array([error_right_x, error_right_y])

        center = (actual_left + actual_right) / 2
        vector_left_to_right = actual_left - actual_right

        yaw_rad = np.arctan2(vector_left_to_right[1], vector_left_to_right[0])
        yaw_deg = np.degrees(yaw_rad)

        expected_center = (expected_left + expected_right) / 2

        center_x_error = center[0] - expected_center[0]
        center_y_error = center[1] - expected_center[1]
        yaw_error = yaw_deg - rotation_deg

        center_x_errors.append(center_x_error)
        center_y_errors.append(center_y_error)
        yaw_errors_deg.append(yaw_error)

    return pd.DataFrame({
        'center_x_error': center_x_errors,
        'center_y_error': center_y_errors,
        'yaw_error_deg': yaw_errors_deg
    })

def plot_comparison_combined(errors_dict):
    # Combine data for plotting
    combined_df = []
    for label, df_errors in errors_dict.items():
        df = df_errors.copy()
        df['Dataset'] = label
        combined_df.append(df)
    combined_df = pd.concat(combined_df, ignore_index=True)

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    sns.set_style("whitegrid")

    # Box plots (top row)
    sns.boxplot(x='Dataset', y='center_x_error', data=combined_df, palette='Set2', ax=axes[0,0])
    axes[0,0].set_title('Center X Error (box plot)')
    axes[0,0].set_xlabel('')
    axes[0,0].set_ylabel('Error (m)')

    sns.boxplot(x='Dataset', y='center_y_error', data=combined_df, palette='Set2', ax=axes[0,1])
    axes[0,1].set_title('Center Y Error (box plot)')
    axes[0,1].set_xlabel('')
    axes[0,1].set_ylabel('Error (m)')

    sns.boxplot(x='Dataset', y='yaw_error_deg', data=combined_df, palette='Set2', ax=axes[0,2])
    axes[0,2].set_title('Yaw Error (box plot)')
    axes[0,2].set_xlabel('')
    axes[0,2].set_ylabel('Error (degrees)')

    # Histograms (bottom row)
    sns.histplot(data=combined_df, x='center_x_error', hue='Dataset', bins=20, kde=True,
                 palette='Set2', element='step', ax=axes[1,0])
    axes[1,0].set_title('Center X Error (histogram)')
    axes[1,0].set_xlabel('Error (m)')
    axes[1,0].set_ylabel('Count')

    sns.histplot(data=combined_df, x='center_y_error', hue='Dataset', bins=20, kde=True,
                 palette='Set2', element='step', ax=axes[1,1])
    axes[1,1].set_title('Center Y Error (histogram)')
    axes[1,1].set_xlabel('Error (m)')
    axes[1,1].set_ylabel('Count')

    sns.histplot(data=combined_df, x='yaw_error_deg', hue='Dataset', bins=20, kde=True,
                 palette='Set2', element='step', ax=axes[1,2])
    axes[1,2].set_title('Yaw Error (histogram)')
    axes[1,2].set_xlabel('Error (degrees)')
    axes[1,2].set_ylabel('Count')

    plt.tight_layout()
    plt.show()


# --- Usage example ---

files_and_angles = {
    'rotate_45_45': ( 'code/rotate_45_45.csv', 45 ),
    'rotate_75_45': ( 'code/rotate_75_45.csv', 45 ),
    # add more here as needed
}

errors_results = {}

for label, (filename, rot_angle) in files_and_angles.items():
    df = pd.read_csv(filename)
    errors_results[label] = compute_pose_errors(df, rot_angle)

plot_comparison_combined(errors_results)
