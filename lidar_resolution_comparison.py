import random
import matplotlib.pyplot as plt
import heapq
import numpy as np

# ====== マップ設定 ======
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
                    heapq.heappush(open_set, (cost+1+h((nx,ny),goal), cost+1, (nx,ny), path))
    return []

# ====== グリッド生成 ======
def generate_grid():
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    count = 0
    while count < OBSTACLE_COUNT:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in (START, GOAL) and grid[y][x] == 0:
            grid[y][x] = 1
            count += 1
    return grid

# ====== LiDAR分解能設定 ======
def get_directions(resolution):
    if resolution == 8:
        return [(dx, dy) for dx in [-1,0,1] for dy in [-1,0,1] if not(dx==0 and dy==0)]
    elif resolution == 4:
        return [(1,0), (-1,0), (0,1), (0,-1)]
    elif resolution == 16:
        dirs = []
        for angle in range(0, 360, 22):  # 16方向
            rad = angle * np.pi / 180
            dx, dy = np.cos(rad), np.sin(rad)
            dirs.append((dx, dy))
        return dirs
    else:
        raise ValueError("Unsupported resolution")

# ====== LiDAR perception ======
def perceive_lidar(grid, directions, distance=5, false_pos=0.05, false_neg=0.05):
    perceived = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    for dx, dy in directions:
        x, y = START
        for _ in range(distance):
            x += dx
            y += dy
            # 丸めて整数に
            ix, iy = round(x), round(y)
            if not (0 <= ix < GRID_SIZE and 0 <= iy < GRID_SIZE):
                break
            if grid[iy][ix] == 1:
                if random.random() > false_neg:
                    perceived[iy][ix] = 1
                break
            else:
                if random.random() < false_pos:
                    perceived[iy][ix] = 1
                    break
    return perceived

# ====== 実験 ======
def run_experiment(resolution, trials=50):
    directions = get_directions(resolution)
    success = 0
    for _ in range(trials):
        grid = generate_grid()
        perceived = perceive_lidar(grid, directions, distance=5)
        path = a_star(perceived, START, GOAL)
        if path:
            success += 1
    return success / trials

# ====== 実行 ======
trials = 100
resolutions = [4, 8, 16]
results = [run_experiment(r, trials) for r in resolutions]

plt.bar([str(r) for r in resolutions], results)
plt.ylim(0,1)
plt.xlabel("LiDAR Resolution (directions)")
plt.ylabel("Success Rate")
plt.title(f"LiDAR Resolution vs Success Rate (Trials={trials})")
plt.show()