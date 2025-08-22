# lidar_noise_success_rate.py

import matplotlib.pyplot as plt
import heapq
import random
import numpy as np

GRID_SIZE = 10
START = (9, 0)
GOAL = (0, 9)

# ==== A* Pathfinding ====
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
                heapq.heappush(open_set, (cost + 1 + h((nx, ny), goal), cost + 1, (nx, ny), path))

    return []

# ==== Grid Generator ====
def generate_grid(num_obstacles=20):
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    count = 0
    while count < num_obstacles:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in (START, GOAL) and grid[y][x] == 0:
            grid[y][x] = 1
            count += 1
    return grid

# ==== LiDAR風センサー ====
def lidar_scan(true_grid, noise_prob=0.05, max_range=3):
    """ LiDAR風に全マスを走査し、ノイズ付きマップを返す """
    noisy_grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if (x, y) in (START, GOAL):
                continue
            true_value = true_grid[y][x]

            # ノイズを適用
            if random.random() < noise_prob:
                noisy_value = 1 - true_value  # 誤検知 or 見逃し
            else:
                noisy_value = true_value

            noisy_grid[y][x] = noisy_value
    return noisy_grid

# ==== 実験 ====
def run_experiment(noise_prob, trials=100):
    success = 0
    for _ in range(trials):
        grid = generate_grid()
        noisy_grid = lidar_scan(grid, noise_prob=noise_prob)

        path = a_star(noisy_grid, START, GOAL)
        if path and path[-1] == GOAL:
            success += 1
    return success / trials

# ==== 実行 ====
if __name__ == "__main__":
    noise_levels = [0.0, 0.05, 0.1, 0.2]
    results = []

    for noise in noise_levels:
        rate = run_experiment(noise)
        print(f"Noise {int(noise*100)}%: Success rate = {rate*100:.1f}%")
        results.append(rate)

    # グラフ化
    plt.bar([f"{int(n*100)}%" for n in noise_levels], results, color="skyblue")
    plt.ylim(0, 1)
    plt.ylabel("Success Rate")
    plt.title("Pathfinding Success Rate vs LiDAR Noise")
    plt.show()
