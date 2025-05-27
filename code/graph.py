import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
from glob import glob

# Use a pre-made style
plt.style.use('fivethirtyeight')  # Alternatives: 'seaborn', 'bmh', 'fivethirtyeight', etc.

# Directory and file pattern
data_dir = 'code/'
file_pattern = os.path.join(data_dir, 'linear_*.csv')
file_list = sorted(glob(file_pattern))

dx_errors_by_file = []
dy_errors_by_file = []
labels = []

for file_path in file_list:
    df = pd.read_csv(file_path)
    data = list(df.itertuples(index=False, name=None))

    dx_measured = np.array([t[0] for t in data])
    dy_measured = np.array([t[1] for t in data])
    dx_odom = np.array([t[2] for t in data])
    dy_odom = np.array([t[3] for t in data])

    error_dx = dx_odom - dx_measured
    error_dy = dy_odom - dy_measured

    dx_errors_by_file.append(error_dx)
    dy_errors_by_file.append(error_dy)

    filename = os.path.splitext(os.path.basename(file_path))[0]
    labels.append(filename)

# Plot boxplots using default appearance of the chosen style
fig, axes = plt.subplots(1, 2, figsize=(14, 6))

axes[0].boxplot(dx_errors_by_file, labels=labels)
axes[0].set_title('dx Error by File')
axes[0].set_xlabel('File')
axes[0].set_ylabel('dx Error')
axes[0].grid(True)

axes[1].boxplot(dy_errors_by_file, labels=labels)
axes[1].set_title('dy Error by File')
axes[1].set_xlabel('File')
axes[1].set_ylabel('dy Error')
axes[1].grid(True)

plt.tight_layout()
plt.show()
