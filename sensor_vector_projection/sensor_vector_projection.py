import matplotlib.pyplot as plt
import numpy as np

# 車の位置と向き（例：北向き）
pos = np.array([5, 5])
dir_index = 3  # 0:east, 1:south, 2:west, 3:north

# 相対ベクトル（左・前・右）
relative_vectors = [np.array([-1, 0]), np.array([0, -1]), np.array([1, 0])]

# 回転行列（90度ずつ）
def rotate(vec, dir_index):
    angle = np.radians(90 * dir_index)
    rot = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)]
    ])
    return np.round(rot @ vec).astype(int)

# 描画
fig, ax = plt.subplots()
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_aspect("equal")
ax.invert_yaxis()
ax.grid(True)

# 車の位置
ax.plot(pos[0], pos[1], "ro")
ax.text(pos[0], pos[1], "Car", ha="center", va="bottom")

# センサーベクトルを回転して表示
for vec, label in zip(relative_vectors, ["L", "F", "R"]):
    rotated = rotate(vec, dir_index)
    target = pos + rotated
    ax.text(target[0], target[1], "✕", fontsize=16, color="red", ha="center", va="center")
    ax.text(target[0], target[1]+0.3, label, ha="center")

plt.title("Sensor positions based on heading")
plt.show()
