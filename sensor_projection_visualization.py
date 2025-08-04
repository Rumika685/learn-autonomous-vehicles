import numpy as np
import matplotlib.pyplot as plt

# 地図サイズと車の状態
grid_size = 10
car_position = np.array([5, 5])
heading = 90  # 北向き（上）

# センサーの相対位置（左・前・右）
sensor_vectors = {
    "left":  np.array([-1, 0]),
    "front": np.array([0, 1]),
    "right": np.array([1, 0]),
}

# センサー値（True: 障害物あり）
sensor_readings = {
    "left": True,
    "front": False,
    "right": True,
}

# 描画準備
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-0.5, grid_size - 0.5)
ax.set_ylim(-0.5, grid_size - 0.5)
ax.grid(True)
ax.invert_yaxis()

# 車の描画
ax.plot(car_position[0], car_position[1], "ro")
ax.text(car_position[0], car_position[1], "CAR", ha="center", va="center")

# センサー方向の回転行列（角度をラジアンに）
rad = np.deg2rad(heading)
rot_matrix = np.array([
    [np.cos(rad), -np.sin(rad)],
    [np.sin(rad),  np.cos(rad)]
])

# センサーの方向をマップ上に反映
for direction, relative_vec in sensor_vectors.items():
    world_vec = rot_matrix @ relative_vec
    world_pos = car_position + np.round(world_vec).astype(int)

    if sensor_readings[direction]:
        ax.text(world_pos[0], world_pos[1], "✕", fontsize=16, color="red", ha="center", va="center")
    ax.arrow(
        car_position[0], car_position[1],
        world_vec[0]*0.5, world_vec[1]*0.5,
        head_width=0.2, head_length=0.2, color="gray"
    )

plt.title("Sensor Projection")
plt.show()