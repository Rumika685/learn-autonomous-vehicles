import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
import random

# マップ設定
GRID_SIZE = 10
START = (9, 0)
GOAL = (0, 9)

# A* 探索関数
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

# グリッド生成（最初の静的障害物20個）
def generate_grid():
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    count = 0
    while count < 20:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in (START, GOAL) and grid[y][x] == 0:
            grid[y][x] = 1
            count += 1
    return grid

# 描画セットアップ
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID_SIZE - 0.5)
ax.set_ylim(-0.5, GRID_SIZE - 0.5)
ax.invert_yaxis()
ax.grid(True)

# 障害物の追加
def add_dynamic_obstacle(grid, path):
    while True:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in path and (x, y) != START and (x, y) != GOAL and grid[y][x] == 0:
            grid[y][x] = 1
            break

# 初期状態
grid = generate_grid()
path = a_star(grid, START, GOAL)
history = []
point, = ax.plot([], [], "ro")
trail_line, = ax.plot([], [], "b--")

# アニメーション更新関数
current_index = [0]
def update(_):
    global path
    ax.clear()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, GRID_SIZE - 0.5)
    ax.set_ylim(-0.5, GRID_SIZE - 0.5)
    ax.invert_yaxis()
    ax.grid(True)

    # 障害物を描画
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if grid[y][x] == 1:
                ax.text(x, y, "✕", ha="center", va="center", fontsize=14, color="red")

    # スタート・ゴール
    sx, sy = START
    gx, gy = GOAL
    ax.text(sx, sy, "START", color="green", ha="center", va="center")
    ax.text(gx, gy, "GOAL", color="blue", ha="center", va="center")

    # 再探索
    path = a_star(grid, path[current_index[0]], GOAL)

    # ゴール済み or 行き止まり
    if not path or path[0] == GOAL:
        ax.set_title("✅ Goal reached or blocked")
        return trail_line, point

    # 移動
    current_index[0] = 1 if len(path) > 1 else 0
    current_pos = path[current_index[0]]
    history.append(current_pos)

    # 新しい障害物を1つ追加
    add_dynamic_obstacle(grid, history)

    # 履歴描画
    xs, ys = zip(*history)
    trail_line, = ax.plot(xs, ys, "b--")
    point, = ax.plot(current_pos[0], current_pos[1], "ro")

    return trail_line, point

# アニメーション起動
ani = animation.FuncAnimation(fig, update, interval=800)
plt.show()