import numpy as np
import matplotlib.pyplot as plt

# 基本センサーベクトル（車が北向きのとき：前=上, 左=左, 右=右）
sensor_vectors = {
    "front": np.array([0, -1]),
    "left":  np.array([-1, 0]),
    "right": np.array([1, 0])
}

def rotate_vector(v, angle_deg):
    theta = np.deg2rad(angle_deg)
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    return rotation_matrix @ v

# 車の向き（例：east = 0°, south = 90°, west = 180°, north = 270°）
car_heading_deg = 90  # 右が前（south）

# 回転後のセンサーベクトルを計算
rotated_vectors = {name: rotate_vector(vec, car_heading_deg) for name, vec in sensor_vectors.items()}

# 可視化
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.grid(True)

# 車の中心
ax.plot(0, 0, 'ko')  # 黒い点

# 各センサー方向を矢印で描画
for name, vec in rotated_vectors.items():
    ax.arrow(0, 0, vec[0], vec[1], head_width=0.1, head_length=0.2, label=name)
    ax.text(vec[0]*1.1, vec[1]*1.1, name, ha="center", fontsize=10)

plt.title(f"Sensor Directions at Heading {car_heading_deg}°")
plt.xlabel("X")
plt.ylabel("Y")
plt.tight_layout()
plt.show()