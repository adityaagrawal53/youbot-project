import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import matplotlib as mpl

# Font settings
mpl.rcParams['font.family'] = 'Times New Roman'

def draw_robot(ax, center, angle_deg, color, annotate_points=False):
    width, height = 60, 30
    hw, hh = width / 2, height / 2
    corners = {
        'A': (-hw, 0),
        'B': ( hw, 0),
        'O': (0, 0)
    }

    angle_rad = np.radians(angle_deg)
    R = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                  [np.sin(angle_rad),  np.cos(angle_rad)]])
    points_transformed = {}

    for name, (x, y) in corners.items():
        p = np.array([x, y])
        p_rot = R @ p + np.array(center)
        points_transformed[name] = p_rot
        if annotate_points:
            ax.plot(*p_rot, 'ko', markersize=4)
            label = name + "'" if name != 'O' else "O′"
            offset = (-2, -2)
            ax.text(p_rot[0] + offset[0], p_rot[1] + offset[1], label, fontsize=11)

    # Robot body
    rect = patches.FancyBboxPatch(
        (center[0] - hw, center[1] - hh),
        width, height,
        boxstyle="round,pad=0.02,rounding_size=5",
        linewidth=2,
        edgecolor=color,
        facecolor='none'
    )
    t = plt.matplotlib.transforms.Affine2D().rotate_deg_around(center[0], center[1], angle_deg) + ax.transData
    rect.set_transform(t)
    ax.add_patch(rect)
    return points_transformed

def draw_measurement(ax, actual, expected, x_label, y_label, color='blue', offset=3):
    x0, y0 = expected
    x1, y1 = actual

    # Horizontal measurement
    # Extension lines perpendicular to x axis (vertical), extending OUTSIDE points by offset
    ax.plot([x0, x0], [y0, y0 + offset], color=color, linewidth=1)
    ax.plot([x1, x1], [y0, y0 + offset], color=color, linewidth=1)
    # Connecting horizontal line between extension line ends
    ax.plot([x0, x1], [y0 + offset, y0 + offset], linestyle='--', color=color, linewidth=1)
    # Label
    ax.text((x0 + x1)/2, y0 + offset + 1, x_label, color=color, fontsize=10, ha='center')

    # Vertical measurement
    # Extension lines perpendicular to y axis (horizontal), extending OUTSIDE points by offset
    ax.plot([x1, x1 + offset], [y0, y0], color=color, linewidth=1)
    ax.plot([x1, x1 + offset], [y1, y1], color=color, linewidth=1)
    # Connecting vertical line between extension line ends
    ax.plot([x1 + offset, x1 + offset], [y0, y1], linestyle='--', color=color, linewidth=1)
    # Label
    ax.text(x1 + offset + 1, (y0 + y1)/2, y_label, color=color, fontsize=10, va='center')

# Setup figure and axes
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_aspect('equal')
ax.set_xlim(-70, 70)
ax.set_ylim(-50, 50)

# Dotted grid lines
for x in [-60, 0, 60]:
    ax.plot([x, x], [-40, 40], linestyle=':', color='black', linewidth=1)
for y in [-40, 0, 40]:
    ax.plot([-60, 60], [y, y], linestyle=':', color='black', linewidth=1)

# Axes with arrows both ways
arrow_props = dict(head_width=2, head_length=3, fc='black', ec='black', length_includes_head=True)
ax.arrow(-60, 0, 120, 0, **arrow_props)
ax.arrow(60, 0, -120, 0, **arrow_props)
ax.arrow(0, -40, 0, 80, **arrow_props)
ax.arrow(0, 40, 0, -80, **arrow_props)
ax.text(63, 2, 'x', fontsize=12)
ax.text(-67, 2, '−x', fontsize=12)
ax.text(2, 43, 'y', fontsize=12)
ax.text(2, -47, '−y', fontsize=12)

# Original center O
ax.plot(0, 0, 'ko', markersize=4)
ax.text(-2, -2, 'O', fontsize=11)

# Initial robot at origin
draw_robot(ax, center=(0, 0), angle_deg=0, color='black')
ax.plot(-30, 0, 'ko', markersize=4)
ax.text(-33, 1, 'A', fontsize=11)
ax.plot(30, 0, 'ko', markersize=4)
ax.text(31, 1, 'B', fontsize=11)

# Expected rotated points (45 deg rotation, no drift)
angle_rad = np.radians(45)
R = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
              [np.sin(angle_rad),  np.cos(angle_rad)]])
A_exp = R @ np.array([-30, 0])
B_exp = R @ np.array([30, 0])
ax.plot(*A_exp, 'o', color='gray', markersize=4)
ax.text(A_exp[0]-5, A_exp[1]+1, "A (exp)", color='gray', fontsize=11)
ax.plot(*B_exp, 'o', color='gray', markersize=4)
ax.text(B_exp[0]+1, B_exp[1]+1, "B (exp)", color='gray', fontsize=11)

# Rotated robot with drift: center at (5,6), rotation 35 degrees
drift_center = (5, 6)
points_rotated = draw_robot(ax, center=drift_center, angle_deg=35, color='blue', annotate_points=True)

# Label center O'
ax.plot(*points_rotated['O'], 'ko', markersize=4)
ax.text(points_rotated['O'][0] - 2, points_rotated['O'][1] - 2, "O′", fontsize=11)

# Draw measurement lines and labels with "extension" style
draw_measurement(ax, points_rotated['A'], A_exp, 'x1_meas', 'y1_meas')
draw_measurement(ax, points_rotated['B'], B_exp, 'x2_meas', 'y2_meas')

# Hide axes frame and ticks
ax.axis('off')
plt.tight_layout()
plt.show()
