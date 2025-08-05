import matplotlib.pyplot as plt
import numpy as np

# グリッドサイズと初期状態
GRID_SIZE = 10
position = np.array([5, 5])
heading_deg = 90  # 北（上向き）

# センサーデータ（左・前・右）
sensor_values = [True, False, True]  # 左と右に障害物あり
sensor_vectors = {
    "left":  np.array([-1, 0]),
    "front": np.array([0, 1]),
    "right": np.array([1, 0]),
}

# 座標回転関数
def rotate(v, angle_deg):
    rad = np.radians(angle_deg)
    rot_matrix = np.array([
        [np.cos(rad), -np.sin(rad)],
        [np.sin(rad),  np.cos(rad)]
    ])
    return rot_matrix @ v

# 描画準備
fig, ax = plt.subplots()
ax.set_xlim(-0.5, GRID_SIZE - 0.5)
ax.set_ylim(-0.5, GRID_SIZE - 0.5)
ax.set_aspect("equal")
ax.invert_yaxis()
ax.grid(True)
ax.set_title("🧭 Sensor Projection with Rotation")

# 現在地の描画
ax.plot(position[0], position[1], "ro")
ax.text(position[0], position[1], "CAR", ha="center", va="bottom", fontsize=10)

# 方向ベクトルの描画（黒）
heading_vec = rotate(np.array([0, 1]), heading_deg)
ax.arrow(position[0], position[1], heading_vec[0]*0.5, heading_vec[1]*0.5,
         head_width=0.2, head_length=0.2, fc="black", ec="black")

# 障害物の描画
for direction, sensed in zip(["left", "front", "right"], sensor_values):
    if sensed:
        local_vec = sensor_vectors[direction]
        rotated_vec = rotate(local_vec, heading_deg)
        obs_pos = position + rotated_vec
        ax.text(obs_pos[0], obs_pos[1], "✕", color="red", fontsize=16, ha="center", va="center")

plt.show()
