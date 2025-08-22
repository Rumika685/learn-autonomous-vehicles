import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Car:
    def __init__(self, start=(5, 5), heading=(1, 0), sensor_range=3):
        self.pos = np.array(start, dtype=float)
        self.heading = np.array(heading, dtype=float)
        self.heading /= np.linalg.norm(self.heading)
        self.sensor_range = sensor_range
        self.detected_obstacles = []  # 検知した障害物を保存

    def classify_obstacle(self, obstacle):
        vec = np.array(obstacle, dtype=float) - self.pos
        dot = np.dot(self.heading, vec)
        cross = np.cross(self.heading, vec)
        eps = 1e-6

        if abs(dot) < eps:
            if cross > 0:
                return "Left"
            elif cross < 0:
                return "Right"
            else:
                return "Center"
        else:
            if dot > 0:
                direction = "Front"
            else:
                direction = "Back"

            if cross > 0:
                return f"{direction}-Left"
            elif cross < 0:
                return f"{direction}-Right"
            else:
                return direction

    def sense_environment(self, obstacles):
        detected = []
        for obs in obstacles:
            dist = np.linalg.norm(np.array(obs, dtype=float) - self.pos)
            if dist <= self.sensor_range:
                label = self.classify_obstacle(obs)
                detected.append((obs, label, round(dist, 2)))
        self.detected_obstacles = detected  # 更新
        return detected

# --- 描画シミュレーション ---
GRID_SIZE = 10
obstacles = [(7, 5), (6, 4), (4, 7), (2, 8)]  # 固定障害物
car = Car(start=(5, 5), heading=(1, 0), sensor_range=3)

fig, ax = plt.subplots()
ax.set_xlim(-0.5, GRID_SIZE-0.5)
ax.set_ylim(-0.5, GRID_SIZE-0.5)
ax.set_aspect("equal")
ax.grid(True)

def update(frame):
    ax.clear()
    ax.set_xlim(-0.5, GRID_SIZE-0.5)
    ax.set_ylim(-0.5, GRID_SIZE-0.5)
    ax.set_aspect("equal")
    ax.grid(True)

    # 障害物を描画（灰色の四角）
    for ox, oy in obstacles:
        ax.add_patch(plt.Rectangle((ox-0.5, oy-0.5), 1, 1, color="gray", alpha=0.5))

    # 車の位置
    ax.plot(car.pos[0], car.pos[1], "ro", markersize=12)
    ax.arrow(car.pos[0], car.pos[1], car.heading[0]*0.5, car.heading[1]*0.5,
             head_width=0.3, head_length=0.3, fc="red", ec="red")

    # センサー検知
    detected = car.sense_environment(obstacles)
    for obs, label, dist in detected:
        x, y = obs
        ax.text(x, y, "✕", ha="center", va="center", fontsize=14, color="red")
        ax.text(x, y-0.3, f"{label} ({dist})", ha="center", va="center", fontsize=6, color="blue")

    ax.set_title("🚗 LiDAR-like Sensor Visualization")

ani = animation.FuncAnimation(fig, update, frames=10, interval=1000)
plt.show()
