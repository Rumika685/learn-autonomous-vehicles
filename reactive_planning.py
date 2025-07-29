import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq, random

GRID_SIZE = 10
START = (9, 9)
GOAL = (0, 0)
OBSTACLE_COUNT = 20
OBSTACLE_CHANCE = 0.3  # 各ステップごとに障害物追加の確率

# A* 探索
def a_star(grid, start, goal):
    h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])
    open_set = [(h(start, goal), 0, start, [])]
    visited = set()
    while open_set:
        _, cost, current, path = heapq.heappop(open_set)
        if current in visited: continue
        visited.add(current)
        path = path + [current]
        if current == goal: return path
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = current[0]+dx, current[1]+dy
            if 0<=nx<GRID_SIZE and 0<=ny<GRID_SIZE and grid[ny][nx]==0:
                heapq.heappush(open_set, (cost+1+h((nx,ny),goal), cost+1, (nx,ny), path))
    return []

# 初期グリッド生成
def generate_grid():
    grid = [[0]*GRID_SIZE for _ in range(GRID_SIZE)]
    placed = 0
    while placed < OBSTACLE_COUNT:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in (START, GOAL) and grid[y][x]==0:
            grid[y][x] = 1
            placed += 1
    return grid

# 障害物の追加
def add_obstacle(grid, path):
    attempts = 0
    while attempts < 100:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in path and (x, y) not in (START, GOAL) and grid[y][x]==0:
            grid[y][x] = 1
            break
        attempts += 1

# 描画用準備
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID_SIZE-0.5)
ax.set_ylim(-0.5, GRID_SIZE-0.5)
ax.invert_yaxis()
ax.grid(True)

grid = generate_grid()
path = a_star(grid, START, GOAL)
history = []
current_index = [0]

def update(_):
    global path
    ax.clear()
    ax.set_xlim(-0.5, GRID_SIZE-0.5)
    ax.set_ylim(-0.5, GRID_SIZE-0.5)
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.grid(True)

    # 障害物描画
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if grid[y][x] == 1:
                ax.text(x, y, "✕", color="red", ha="center", va="center")

    # スタート・ゴール
    ax.text(*START, "START", color="green", ha="center", va="center")
    ax.text(*GOAL, "GOAL", color="blue", ha="center", va="center")

    # ルートが塞がれていたら再探索
    if current_index[0] >= len(path):
        ax.set_title("Blocked or finished")
        return

    # 現在位置と履歴
    pos = path[current_index[0]]
    history.append(pos)
    xs, ys = zip(*history)
    ax.plot(xs, ys, "b--")
    ax.plot(pos[0], pos[1], "ro")

    if pos == GOAL:
        ax.set_title("✅ Goal reached!")
        current_index[0] = len(path)
        return

    # ランダムで障害物追加 → 再探索（リアクティブ）
    if random.random() < OBSTACLE_CHANCE:
        add_obstacle(grid, history)
        new_path = a_star(grid, pos, GOAL)
        if new_path:
            path[:] = new_path
            current_index[0] = 0
        else:
            ax.set_title("🛑 No path to goal!")
            current_index[0] = len(path)
            return
    else:
        current_index[0] += 1

ani = animation.FuncAnimation(fig, update, interval=800)
plt.show()
