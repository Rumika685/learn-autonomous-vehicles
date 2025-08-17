import matplotlib.pyplot as plt
import heapq
import random
import copy

# マップ設定
GRID_SIZE = 10
START = (9, 0)
GOAL = (0, 9)

# A*探索関数
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

# グリッド生成（ランダム障害物20個）
def generate_grid():
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    count = 0
    while count < 20:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in (START, GOAL) and grid[y][x] == 0:
            grid[y][x] = 1
            count += 1
    return grid

# 🚀 経路保証付きグリッド生成
def generate_grid_with_path():
    while True:
        grid = generate_grid()
        if a_star(grid, START, GOAL):  # 経路がある場合のみ採用
            return grid

# センサー誤差の適用
def apply_sensor_noise(grid, noise_prob=0.05):
    noisy_grid = copy.deepcopy(grid)
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if (x, y) in (START, GOAL):
                continue
            if random.random() < noise_prob:
                noisy_grid[y][x] = 1 - noisy_grid[y][x]  # 0⇔1 を反転
    return noisy_grid

# 実験の実行
def run_experiment(noise_prob, ax):
    grid = generate_grid_with_path()   # ✅ 必ず経路あり
    noisy_grid = apply_sensor_noise(grid, noise_prob=noise_prob)
    path = a_star(noisy_grid, START, GOAL)

    # 描画
    ax.clear()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, GRID_SIZE - 0.5)
    ax.set_ylim(-0.5, GRID_SIZE - 0.5)
    ax.invert_yaxis()
    ax.grid(True)
    ax.set_title(f"Noise={int(noise_prob*100)}%")

    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if noisy_grid[y][x] == 1:
                ax.text(x, y, "✕", ha="center", va="center", fontsize=10, color="red")

    sx, sy = START
    gx, gy = GOAL
    ax.text(sx, sy, "START", color="green", ha="center", va="center")
    ax.text(gx, gy, "GOAL", color="blue", ha="center", va="center")

    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, "b--")
        ax.plot(xs[-1], ys[-1], "ro")
    else:
        ax.text(GRID_SIZE//2, GRID_SIZE//2, "No Path", color="black", ha="center", va="center")

# 実験実行
fig, axes = plt.subplots(1, 3, figsize=(15, 5))
for ax, noise in zip(axes, [0.0, 0.05, 0.15]):  # 誤差 0%, 5%, 15%
    run_experiment(noise, ax)

plt.tight_layout()
plt.show()
