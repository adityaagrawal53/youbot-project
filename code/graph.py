import matplotlib.pyplot as plt

import pandas as pd

import os
print(os.getcwd())

df = pd.read_csv('code/linear_06_2.csv')
data = list(df.itertuples(index=False, name=None))

# Now `data` is a list of tuples, same format as before
print(data)


def plot_xy_comparison(data):
    """
    data: list of tuples like (dx_measured, dy_measured, dx_odom, dy_odom, dyaw_odom)
    """
    dx_measured = [t[0] for t in data]
    dy_measured = [t[1] for t in data]
    dx_odom = [t[2] for t in data]
    dy_odom = [t[3] for t in data]

    # Plot dx comparison
    plt.figure(figsize=(10, 5))
    
    plt.subplot(1, 2, 1)
    plt.scatter(dx_measured, dx_odom, color='blue', label='dx')
    plt.plot(dx_measured, dx_measured, 'k--', label='Perfect match')
    plt.xlabel('Measured dx')
    plt.ylabel('Odometry dx')
    plt.title('dx: Measured vs. Odometry')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')

    # Plot dy comparison
    plt.subplot(1, 2, 2)
    plt.scatter(dy_measured, dy_odom, color='green', label='dy')
    plt.plot(dy_measured, dy_measured, 'k--', label='Perfect match')
    plt.xlabel('Measured dy')
    plt.ylabel('Odometry dy')
    plt.title('dy: Measured vs. Odometry')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')

    plt.tight_layout()
    plt.show()


plot_xy_comparison(data)
