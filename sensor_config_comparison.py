import random
import matplotlib.pyplot as plt
import heapq
import numpy as np

# マップ設定
GRID_SIZE = 10
START = (9, 0)
GOAL = (0, 9)
OBSTACLE_COUNT = 20

# ====== A* 探索 ======
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
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                if grid[ny][nx] == 0:
                    heapq.heappush(open_set, (cost + 1 + h((nx, ny), goal), cost + 1, (nx, ny), path))
    return []

# ====== マップ生成 ======
def generate_grid():
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    count = 0
    while count < OBSTACLE_COUNT:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in (START, GOAL) and grid[y][x] == 0:
            grid[y][x] = 1
            count += 1
    return grid

# ====== センサーノイズ付き知覚 ======
def perceive_environment(grid, mode="front3", distance=3, false_pos=0.05, false_neg=0.05):
    perceived = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    
    if mode == "front3":
        directions = [(0, -1), (-1, 0), (1, 0)]  # 前・左・右
    elif mode == "lidar360":
        directions = [(dx, dy) for dx in [-1,0,1] for dy in [-1,0,1] if not(dx==0 and dy==0)]
    else:
        raise ValueError("Unknown sensor mode")

    # 各方向にビームを飛ばす
    for dx, dy in directions:
        x, y = START
        for _ in range(distance):
            x += dx
            y += dy
            if not (0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE):
                break
            # センサーが障害物を検知
            if grid[y][x] == 1:
                if random.random() > false_neg:  # 見逃し確率
                    perceived[y][x] = 1
                break
            else:
                if random.random() < false_pos:  # 誤検知確率
                    perceived[y][x] = 1
                    break
    return perceived

# ====== 実験関数 ======
def run_experiment(mode, trials=50, distance=3, false_pos=0.05, false_neg=0.05):
    success_count = 0
    for _ in range(trials):
        grid = generate_grid()
        perceived_grid = perceive_environment(grid, mode, distance, false_pos, false_neg)
        path = a_star(perceived_grid, START, GOAL)
        if path:
            success_count += 1
    return success_count / trials

# ====== 実験実行 ======
trials = 100
fp, fn = 0.05, 0.05

front3_success = run_experiment("front3", trials=trials, distance=3, false_pos=fp, false_neg=fn)
lidar_success = run_experiment("lidar360", trials=trials, distance=3, false_pos=fp, false_neg=fn)

# ====== 可視化 ======
plt.bar(["Front 3 Sensors", "360° LiDAR"], [front3_success, lidar_success], color=["orange", "blue"])
plt.ylim(0,1)
plt.ylabel("Success Rate")
plt.title(f"Sensor Config Comparison (Trials={trials}, FP={fp}, FN={fn})")
plt.show()