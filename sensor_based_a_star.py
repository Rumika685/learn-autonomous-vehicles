import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import heapq
import random

GRID_SIZE = 10
START = (9, 0)
GOAL = (0, 9)

# --- A* 探索関数 ---
def a_star(grid, start, goal):
    h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])
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
            nx, ny = current[0]+dx, current[1]+dy
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                if grid[ny][nx] == 0:  # 未知 or 空き
                    heapq.heappush(open_set, (cost+1+h((nx,ny), goal), cost+1, (nx,ny), path))

    return []

# --- Car クラス ---
class Car:
    def __init__(self, pos=START, heading=(0,-1), sensor_range=3):
        self.pos = np.array(pos, dtype=int)
        self.heading = np.array(heading, dtype=int)
        self.sensor_range = sensor_range
        self.known_grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]  # ローカル地図（初期は未知=0）

    def detect_obstacles(self, true_grid):
        """センサーで障害物を検知し、known_grid を更新"""
        detected = []
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                vec = np.array((x,y)) - self.pos
                dist = np.linalg.norm(vec)
                if 0 < dist <= self.sensor_range:  # センサー範囲内
                    if true_grid[y][x] == 1:
                        self.known_grid[y][x] = 1  # 障害物を記録
                        detected.append((x,y))
        return detected

# --- グリッド生成（真のマップ） ---
def generate_true_grid(num_obstacles=20):
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    count = 0
    while count < num_obstacles:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in (START, GOAL) and grid[y][x] == 0:
            grid[y][x] = 1
            count += 1
    return grid

# --- アニメーション ---
true_grid = generate_true_grid()
car = Car()
path = a_star(car.known_grid, tuple(car.pos), GOAL)

fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID_SIZE-0.5)
ax.set_ylim(-0.5, GRID_SIZE-0.5)
ax.invert_yaxis()
ax.grid(True)
history = []

def update(_):
    global path
    ax.clear()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, GRID_SIZE-0.5)
    ax.set_ylim(-0.5, GRID_SIZE-0.5)
    ax.invert_yaxis()
    ax.grid(True)

    # --- ゴール済みの場合 ---
    if tuple(car.pos) == GOAL:
        ax.set_title("✅ Goal reached! (Final state)")

        # 障害物（真の赤 ✕）
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                if true_grid[y][x] == 1:
                    ax.text(x, y, "✕", color="red", ha="center", va="center")

        # 検知済み障害物（青 ✕）
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                if car.known_grid[y][x] == 1:
                    ax.text(x, y, "✕", color="blue", ha="center", va="center")

        # 通過履歴（緑線）
        if history:
            xs, ys = zip(*history)
            ax.plot(xs, ys, "g-")

        # START & GOAL
        ax.text(*START, "START", color="green", ha="center", va="center")
        ax.text(*GOAL, "GOAL", color="blue", ha="center", va="center")

        return  # ← ここでは終了しても、最後の画面が保持される

    # --- 以降は通常の移動ロジック ---
    car.detect_obstacles(true_grid)
    path = a_star(car.known_grid, tuple(car.pos), GOAL)

    if not path:
        ax.set_title("❌ No path found")
        return

    if len(path) > 1:
        car.pos = np.array(path[1])
    history.append(tuple(car.pos))

    # 真の障害物（赤 ✕）
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if true_grid[y][x] == 1:
                ax.text(x, y, "✕", color="red", ha="center", va="center")

    # 検知済み障害物（青 ✕）
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if car.known_grid[y][x] == 1:
                ax.text(x, y, "✕", color="blue", ha="center", va="center")

    # 経路（黒線）
    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, "k--")

    # 通過履歴（緑線）
    if history:
        xs, ys = zip(*history)
        ax.plot(xs, ys, "g-")

    ax.text(*START, "START", color="green", ha="center", va="center")
    ax.text(*GOAL, "GOAL", color="blue", ha="center", va="center")

ani = animation.FuncAnimation(fig, update, interval=800)
plt.show()