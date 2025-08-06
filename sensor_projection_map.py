# sensor_projection_map.py
# 車の向きに応じてセンサー情報を地図に投影する基本デモ

import matplotlib.pyplot as plt
import numpy as np

# --- 設定 ---
grid_size = 10
pos = np.array([5, 5])  # 車の位置
angle_deg = 90          # 車の進行方向（0=東, 90=北, 180=西, 270=南）
sensor_values = [True, False, True]  # 左・前・右

# --- センサーオフセット（ローカル座標系） ---
sensor_local = {
    "left":  np.array([-1, 0]),
    "front": np.array([0, 1]),
    "right": np.array([1, 0])
}

# --- ヘディングに基づく回転行列を作成 ---
def rotation_matrix(angle_deg):
    theta = np.deg2rad(angle_deg)
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

# --- 向きに応じてセンサーをワールド座標に変換 ---
R = rotation_matrix(angle_deg)
sensor_world = {}
for (key, offset), active in zip(sensor_local.items(), sensor_values):
    if active:
        delta = R @ offset
        world_pos = pos + np.round(delta).astype(int)
        sensor_world[key] = tuple(world_pos)

# --- 描画 ---
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-0.5, grid_size - 0.5)
ax.set_ylim(-0.5, grid_size - 0.5)
ax.invert_yaxis()
ax.grid(True)

# グリッド
for x in range(grid_size):
    for y in range(grid_size):
        ax.plot(x, y, '.', color='lightgray')

# 車の位置
ax.plot(pos[0], pos[1], 'ro')

# センサー可視化
for key, (x, y) in sensor_world.items():
    ax.text(x, y, '✕', fontsize=16, ha='center', va='center', color='red')
    ax.text(x, y - 0.4, key, fontsize=8, ha='center', va='center', color='black')

plt.title("Sensor Projection on Map")
plt.show()
