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

# LiDARスキャン（ノイズ付き）
def lidar_scan_with_noise(pos, heading, obstacles, max_range=6, angle_step=45, noise=1):
    """
    noise: ±マス数の測定誤差
    """
    detections = []
    for angle_deg in range(0, 360, angle_step):
        angle = np.radians(angle_deg) + heading
        dx, dy = np.cos(angle), np.sin(angle)
        for r in range(1, max_range+1):
            x = int(round(pos[0] + dx * r))
            y = int(round(pos[1] + dy * r))
            if (x, y) in obstacles:
                # ノイズを追加（±noise以内で揺らす）
                noisy_r = max(1, min(max_range, r + random.randint(-noise, noise)))
                nx = int(round(pos[0] + dx * noisy_r))
                ny = int(round(pos[1] + dy * noisy_r))
                detections.append(((nx, ny), angle_deg))
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

    # LiDAR検出点（ノイズ付き）
    for (ox, oy), ang in detections:
        ax.plot([pos[0], ox], [pos[1], oy], "g--", alpha=0.6)
        ax.text(ox, oy, f"{ang}°", fontsize=8, color="green")

    plt.title("Noisy LiDAR scan (±1 cell)")
    plt.show()

# 実行例
random.seed(42)
obstacles = generate_obstacles(20)
pos = START
detections = lidar_scan_with_noise(pos, heading=0, obstacles=obstacles, max_range=6, angle_step=45, noise=1)
visualize(obstacles, detections, pos)