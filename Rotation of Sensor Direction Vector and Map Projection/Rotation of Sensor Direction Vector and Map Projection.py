import numpy as np
import matplotlib.pyplot as plt

# 自車位置と角度（θ = 0 は北向き）
car_pos = np.array([5, 5])
heading_deg = 90  # 右（東）を向いている
theta = np.radians(heading_deg)

# 相対センサーベクトル（前・左・右）
sensor_vectors = {
    "front": np.array([0, -1]),
    "left":  np.array([-1, 0]),
    "right": np.array([1, 0])
}

# 回転行列
rotation_matrix = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta),  np.cos(theta)]
])

# センサーデータ（左と右に障害物あり）
sensor_data = {
    "front": False,
    "left":  True,
    "right": True
}

# 描画
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(3, 7)
ax.set_ylim(3, 7)
ax.grid(True)

# 車の位置を描画
ax.plot(car_pos[0], car_pos[1], "ro")
ax.text(car_pos[0], car_pos[1] + 0.3, "Car", ha="center")

# センサーからの障害物位置を描画
for direction, vec in sensor_vectors.items():
    if sensor_data[direction]:
        rotated_vec = rotation_matrix @ vec
        obs_pos = car_pos + rotated_vec
        ax.text(obs_pos[0], obs_pos[1], "✕", color="red", fontsize=14, ha="center")

plt.title("Sensor Projection on Grid")
plt.show()
