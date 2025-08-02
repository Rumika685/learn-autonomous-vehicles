import numpy as np
import matplotlib.pyplot as plt

def rotate_vector(v, angle_deg):
    theta = np.deg2rad(angle_deg)
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    return rotation_matrix @ v

# 元のベクトル（右向き＝東向き）
v = np.array([1, 0])
angles = [0, 45, 90, 135, 180]

fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.grid(True)

# 🖤 元のベクトル（黒い矢印）
ax.arrow(0, 0, v[0], v[1], head_width=0.05, head_length=0.1, fc='black', ec='black', label="original")

# 🔄 各角度で回転させたベクトル
colors = ['red', 'orange', 'green', 'blue', 'purple']
for angle, color in zip(angles, colors):
    rotated = rotate_vector(v, angle)
    ax.arrow(0, 0, rotated[0], rotated[1], head_width=0.05, head_length=0.1, fc=color, ec=color)
    ax.text(rotated[0]*1.1, rotated[1]*1.1, f"{angle}°", color=color, ha="center")

plt.title("2D Vector Rotation")
plt.xlabel("X")
plt.ylabel("Y")
plt.tight_layout()
plt.show()
