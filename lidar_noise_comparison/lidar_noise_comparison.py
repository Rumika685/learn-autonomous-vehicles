import matplotlib.pyplot as plt
import numpy as np
import random
import heapq

GRID_SIZE = 10
START = (9, 0)
GOAL = (0, 9)

# --- A* 探索 ---
def a_star(grid, start, goal):
    h = lambda a, b: abs(a[0] - b[0]) + abs(a[1] - b[1])
    open_set = [(h(start, goal), 0, start, [])]
    visited = set()

    while open_set:
        _, cost, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        path = path + [current]
        if current == goal:
            return path
        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and grid[ny][nx] == 0:
                heapq.heappush(open_set, (cost+1+h((nx,ny), goal), cost+1, (nx,ny), path))
    return []

# --- 障害物生成 ---
def generate_grid(num_obstacles=20):
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    count = 0
    while count < num_obstacles:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in (START, GOAL) and grid[y][x] == 0:
            grid[y][x] = 1
            count += 1
    return grid

# --- LiDARで障害物を検知（ノイズあり）---
def lidar_observation(true_grid, pos, max_range=6, angle_step=45, noise=0):
    observed = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    for angle_deg in range(0, 360, angle_step):
        angle = np.radians(angle_deg)
        dx, dy = np.cos(angle), np.sin(angle)
        for r in range(1, max_range+1):
            x = int(round(pos[0] + dx * r))
            y = int(round(pos[1] + dy * r))
            if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                if true_grid[y][x] == 1:
                    # ノイズを追加
                    noisy_r = max(1, min(max_range, r + random.randint(-noise, noise)))
                    nx = int(round(pos[0] + dx * noisy_r))
                    ny = int(round(pos[1] + dy * noisy_r))
                    if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                        observed[ny][nx] = 1
                    break
    return observed

# --- 実験実行 ---
def run_experiment(noise, trials=100):
    success = 0
    for _ in range(trials):
        grid = generate_grid()
        observed_grid = lidar_observation(grid, START, noise=noise)
        path = a_star(observed_grid, START, GOAL)
        if path and path[-1] == GOAL:
            success += 1
    return success / trials

# --- 実行と可視化 ---
noises = [0, 1, 2]
results = [run_experiment(n, trials=100) for n in noises]

plt.bar([str(n) for n in noises], results, color="skyblue")
plt.xlabel("LiDAR Noise Level (±cells)")
plt.ylabel("Success Rate")
plt.title("Path Planning Success Rate under LiDAR Noise")
plt.ylim(0, 1.05)
plt.show()
