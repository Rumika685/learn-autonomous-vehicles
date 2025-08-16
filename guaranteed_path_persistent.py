import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
import random

# === マップ設定 ===
GRID_SIZE = 10
START = (9, 0)   # 右下
GOAL = (0, 9)    # 左上

# === A* 探索 ===
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
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and grid[ny][nx] == 0:
                heapq.heappush(open_set, (cost+1+h((nx,ny), goal), cost+1, (nx,ny), path))
    return []

# === グリッド生成（ゴール保証） ===
def generate_grid():
    while True:
        grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        count = 0
        while count < 20:
            x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
            if (x, y) not in (START, GOAL) and grid[y][x] == 0:
                grid[y][x] = 1
                count += 1
        if a_star(grid, START, GOAL):
            return grid

# === 障害物追加（行き止まりなら1個消す） ===
def add_dynamic_obstacle(grid, path):
    while True:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in path and (x, y) != START and (x, y) != GOAL and grid[y][x] == 0:
            grid[y][x] = 1
            break
    if not a_star(grid, path[-1], GOAL):
        while True:
            x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
            if grid[y][x] == 1 and (x, y) not in (START, GOAL):
                grid[y][x] = 0
                break

# === 描画セットアップ ===
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID_SIZE - 0.5)
ax.set_ylim(-0.5, GRID_SIZE - 0.5)
ax.invert_yaxis()
ax.grid(True)

# === 初期状態 ===
grid = generate_grid()
path = a_star(grid, START, GOAL)
history = [START]   # ★ スタート地点を必ず追加
current_pos = [START]

# === アニメーション更新 ===
def update(_):
    global path
    ax.clear()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, GRID_SIZE - 0.5)
    ax.set_ylim(-0.5, GRID_SIZE - 0.5)
    ax.invert_yaxis()
    ax.grid(True)

    # 障害物描画
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if grid[y][x] == 1:
                ax.text(x, y, "✕", ha="center", va="center", fontsize=14, color="red")

    # スタート・ゴール
    sx, sy = START
    gx, gy = GOAL
    ax.text(sx, sy, "START", color="green", ha="center", va="center")
    ax.text(gx, gy, "GOAL", color="blue", ha="center", va="center")

    # ゴール到達チェック
    if current_pos[0] == GOAL:
        ax.set_title("✅ Goal Reached")
        xs, ys = zip(*history)
        ax.plot(xs, ys, "b--")
        ax.plot(current_pos[0][0], current_pos[0][1], "ro")
        return

    # 経路探索
    path = a_star(grid, current_pos[0], GOAL)
    if not path:
        add_dynamic_obstacle(grid, history+[current_pos[0]])
        path = a_star(grid, current_pos[0], GOAL)

    # 移動
    if len(path) > 1:
        current_pos[0] = path[1]
        history.append(current_pos[0])

    # 履歴描画（常に残す）
    if history:
        xs, ys = zip(*history)
        ax.plot(xs, ys, "b--")
    ax.plot(current_pos[0][0], current_pos[0][1], "ro")

# === 実行 ===
ani = animation.FuncAnimation(fig, update, interval=500)
plt.show()
