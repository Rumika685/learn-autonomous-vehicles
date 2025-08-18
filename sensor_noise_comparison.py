import matplotlib.pyplot as plt
import heapq
import random

# ====== 設定 ======
GRID_SIZE = 10
START = (9, 0)
GOAL = (0, 9)
OBSTACLE_COUNT = 20

# ====== A* 探索関数 ======
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

# ====== センサー誤検知を適用 ======
def apply_sensor_noise(grid, noise_prob=0.05):
    noisy_grid = [row[:] for row in grid]  # deepcopy
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if (x, y) not in (START, GOAL):
                if random.random() < noise_prob:
                    noisy_grid[y][x] = 1 - noisy_grid[y][x]  # 0→1 or 1→0
    return noisy_grid

# ====== 実験実行 ======
def run_experiment(noise_prob, trials=100):
    success = 0
    for _ in range(trials):
        grid = generate_grid()
        noisy_grid = apply_sensor_noise(grid, noise_prob)
        path = a_star(noisy_grid, START, GOAL)
        if path:  # 経路が見つかれば成功
            success += 1
    return success / trials  # 成功率（0〜1）

# ====== 実験・結果表示 ======
noise_levels = [0.0, 0.05, 0.1, 0.2]  # 誤検知率
results = []

for noise in noise_levels:
    rate = run_experiment(noise, trials=100)
    results.append(rate)
    print(f"Noise {noise*100:.0f}% → Success rate: {rate*100:.1f}%")

# ====== グラフ描画 ======
plt.bar([f"{int(n*100)}%" for n in noise_levels], [r*100 for r in results], color="skyblue")
plt.ylabel("Success Rate (%)")
plt.xlabel("Sensor Noise Probability")
plt.title("Pathfinding Success Rate under Sensor Noise")
plt.ylim(0, 100)
plt.show()