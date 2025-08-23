import random
import matplotlib.pyplot as plt
import heapq

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
                if grid[ny][nx] == 0:  # 通行可能
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

# ====== センサーノイズ適用 ======
def apply_sensor_noise(grid, false_pos_prob=0.0, false_neg_prob=0.0):
    noisy_grid = [row[:] for row in grid]
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if (x, y) in (START, GOAL):
                continue
            # False Positive: 障害物がないのにあると誤検知
            if grid[y][x] == 0 and random.random() < false_pos_prob:
                noisy_grid[y][x] = 1
            # False Negative: 障害物を見逃す
            elif grid[y][x] == 1 and random.random() < false_neg_prob:
                noisy_grid[y][x] = 0
    return noisy_grid

# ====== 実験関数 ======
def run_experiment(false_pos_prob, false_neg_prob, trials=100):
    success_count = 0
    for _ in range(trials):
        grid = generate_grid()
        noisy_grid = apply_sensor_noise(grid, false_pos_prob, false_neg_prob)
        path = a_star(noisy_grid, START, GOAL)
        if path:
            success_count += 1
    return success_count / trials

# ====== 実行 ======
false_pos_probs = [0.0, 0.05, 0.1, 0.2]
false_neg_probs = [0.0, 0.05, 0.1, 0.2]

results_pos = [run_experiment(p, 0.0) for p in false_pos_probs]
results_neg = [run_experiment(0.0, n) for n in false_neg_probs]

# ====== 可視化 ======
plt.figure(figsize=(8, 5))
plt.plot(false_pos_probs, results_pos, "r-o", label="False Positive (ghost obstacles)")
plt.plot(false_neg_probs, results_neg, "b-o", label="False Negative (missed obstacles)")
plt.xlabel("Noise probability")
plt.ylabel("Success rate")
plt.title("Comparison: False Positive vs False Negative")
plt.legend()
plt.grid(True)
plt.show()