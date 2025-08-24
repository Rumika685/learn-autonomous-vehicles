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

# ====== LiDAR方向生成 ======
def get_directions(resolution=8):
    # 360度を resolution 分割
    angles = np.linspace(0, 2*np.pi, resolution, endpoint=False)
    return [(np.cos(a), np.sin(a)) for a in angles]

# ====== LiDAR perception ======
def perceive_lidar(grid, directions, distance=5, false_pos=0.05, false_neg=0.05):
    perceived = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    for dx, dy in directions:
        x, y = START
        for _ in range(distance):
            x += dx
            y += dy
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
def run_experiment(resolution, trials=50, distance=5):
    directions = get_directions(resolution)
    success = 0
    for _ in range(trials):
        grid = generate_grid()
        perceived = perceive_lidar(grid, directions, distance=distance)
        path = a_star(perceived, START, GOAL)
        if path:
            success += 1
    return success / trials

# ====== 実行 ======
trials = 100
resolutions = [4, 8, 16, 32]
results = [run_experiment(r, trials) for r in resolutions]

plt.plot(resolutions, results, marker="o")
plt.ylim(0,1)
plt.xlabel("LiDAR Resolution (# of beams)")
plt.ylabel("Success Rate")
plt.title(f"LiDAR Resolution vs Success Rate (Trials={trials})")
plt.grid(True)
plt.show()