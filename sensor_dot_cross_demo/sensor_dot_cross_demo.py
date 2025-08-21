import numpy as np
import matplotlib.pyplot as plt

# 車の位置と進行方向
car_pos = np.array([5, 5])
heading = np.array([0, -1])  # 北向き（上方向）

# いくつかの障害物
obstacles = [
    np.array([5, 2]),  # 前方
    np.array([8, 5]),  # 右側
    np.array([2, 5]),  # 左側
    np.array([5, 8])   # 後方
]

def classify_obstacle(car_pos, heading, obstacle):
    vec = obstacle - car_pos
    dot = np.dot(heading, vec)
    cross = np.cross(heading, vec)

    if dot > 0:   # 前方
        direction = "Front"
    else:         # 後方
        direction = "Back"

    if cross > 0:
        side = "Left"
    elif cross < 0:
        side = "Right"
    else:
        side = "Center"

    return direction, side

# 判定
for obs in obstacles:
    d, s = classify_obstacle(car_pos, heading, obs)
    print(f"Obstacle at {obs} → {d}-{s}")

# 可視化
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.grid(True)

# 車
ax.plot(car_pos[0], car_pos[1], "ro")
ax.arrow(car_pos[0], car_pos[1], heading[0], heading[1],
         head_width=0.2, color="red", length_includes_head=True)

# 障害物
for obs in obstacles:
    ax.plot(obs[0], obs[1], "bx")
    ax.text(obs[0]+0.1, obs[1]+0.1, str(obs))

plt.title("Dot & Cross for Obstacle Classification")
plt.show()
