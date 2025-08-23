import matplotlib.pyplot as plt
import numpy as np
import random

GRID_SIZE = 10
START = (5, 5)

# 障害物生成
def generate_obstacles(count=15):
    obstacles = set()
    while len(obstacles) < count:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) != START:
            obstacles.add((x, y))
    return obstacles

# LiDARスキャン関数
def lidar_scan(pos, heading, obstacles, max_range=6, angle_step=45):
    """
    pos: (x, y) 車の位置
    heading: ラジアンでの向き
    obstacles: set((x,y))
    max_range: LiDARの距離
    angle_step: 角度分解能（45度なら前後左右と斜め）
    """
    detections = []
    for angle_deg in range(0, 360, angle_step):
        angle = np.radians(angle_deg) + heading
        dx, dy = np.cos(angle), np.sin(angle)
        for r in range(1, max_range+1):
            x = int(round(pos[0] + dx * r))
            y = int(round(pos[1] + dy * r))
            if (x, y) in obstacles:
                detections.append(((x, y), angle_deg))
                break
    return detections

# 可視化
def visualize(obstacles, detections, pos):
    fig, ax = plt.subplots()
    ax.set_xlim(-0.5, GRID_SIZE-0.5)
    ax.set_ylim(-0.5, GRID_SIZE-0.5)
    ax.set_aspect("equal")
    ax.grid(True)
    ax.invert_yaxis()

    # 障害物
    for (x, y) in obstacles:
        ax.text(x, y, "✕", ha="center", va="center", color="red")

    # 車
    ax.plot(pos[0], pos[1], "bo", markersize=12)

    # LiDAR検出点
    for (ox, oy), ang in detections:
        ax.plot([pos[0], ox], [pos[1], oy], "g--", alpha=0.6)
        ax.text(ox, oy, f"{ang}°", fontsize=8, color="green")

    plt.title("LiDAR variable range scan")
    plt.show()

# 実行例
random.seed(42)
obstacles = generate_obstacles(20)
pos = START
detections = lidar_scan(pos, heading=0, obstacles=obstacles, max_range=6, angle_step=45)
visualize(obstacles, detections, pos)