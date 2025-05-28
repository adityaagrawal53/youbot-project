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

def print_column_data(label, dx_meas, dy_meas, dx_odom, dy_odom):
    print(f"\nResults for {label}:")
    print(f"{'Row':<5} {'dx_meas':>10} {'dy_meas':>10} {'dx_odom':>10} {'dy_odom':>10}")
    for i in range(len(dx_meas)):
        print(f"{i:<5} {dx_meas[i]:10.3f} {dy_meas[i]:10.3f} {dx_odom[i]:10.3f} {dy_odom[i]:10.3f}")


def print_stats(label, errors_dict):
    print(f"\nStatistics for {label}:")
    for key, errors in errors_dict.items():
        mean = np.mean(errors)
        median = np.median(errors)
        var = np.var(errors)
        print(f"  {key} -> Mean: {mean:.4f}, Median: {median:.4f}, Variance: {var:.4f}")

def plot_linear_errors():
    file_list = sorted(glob(os.path.join(CSV_DIR, 'linear_*.csv')))

    dx_errors_by_exp = []
    dy_errors_by_exp = []
    labels = []

    for file_path in file_list:
        df = pd.read_csv(file_path)
        data = list(df.itertuples(index=False, name=None))

        dx_meas = np.array([t[0] for t in data])
        dy_meas = np.array([t[1] for t in data])
        dx_odom = np.array([t[2] for t in data])
        dy_odom = np.array([t[3] for t in data])

        error_dx = dx_odom - dx_meas
        error_dy = dy_odom - dy_meas

        dx_errors_by_exp.append(error_dx)
        dy_errors_by_exp.append(error_dy)

        filename = os.path.splitext(os.path.basename(file_path))[0]
        parts = filename.split('_')
        speed = float(parts[1]) / 10
        distance = int(parts[2])
        label = f"{speed:.1f} m/s, {distance} m"
        labels.append(label)

        if PRINT_COLUMN_DATA:
            print_column_data(label, dx_meas, dy_meas, dx_odom, dy_odom)


        if PRINT_STATS:
            print_stats(label, {'dx_error': error_dx, 'dy_error': error_dy})

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    axes[0].boxplot(dx_errors_by_exp, labels=labels)
    axes[0].set_title('dx Error by Experiment')
    axes[0].set_xlabel('Experiment')
    axes[0].set_ylabel('dx Error (cm)')
    axes[0].grid(True)
    axes[0].tick_params(axis='x', rotation=45, labelsize=9)

    axes[1].boxplot(dy_errors_by_exp, labels=labels)
    axes[1].set_title('dy Error by Experiment')
    axes[1].set_xlabel('Experiment')
    axes[1].set_ylabel('dy Error (cm)')
    axes[1].grid(True)
    axes[1].tick_params(axis='x', rotation=45, labelsize=9)

    plt.tight_layout()

    if SAVE_FIG:
        save_path = os.path.join(DATA_DIR, 'linear_error_boxplots.png')
        plt.savefig(save_path, dpi=600)
    if SHOW_FIG:
        plt.show()

if __name__ == "__main__":
    plot_linear_errors()
