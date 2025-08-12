import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
import random

GRID_SIZE = 10
START = (9, 0)
GOAL = (0, 9)

FALSE_POSITIVE_RATE = 0.05  # 誤検知 5%
FALSE_NEGATIVE_RATE = 0.05  # 見逃し 5%

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

def generate_grid():
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    count = 0
    while count < 20:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in (START, GOAL) and grid[y][x] == 0:
            grid[y][x] = 1
            count += 1
    return grid

def add_dynamic_obstacle(grid, path):
    while True:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in path and (x, y) != START and (x, y) != GOAL and grid[y][x] == 0:
            grid[y][x] = 1
            break

def sense_with_noise(true_state):
    """true_state: True = 障害物あり, False = 障害物なし"""
    import random
    if true_state:
        # 見逃し（false negative）
        if random.random() < FALSE_NEGATIVE_RATE:
            print("\033[93m[Sensor] False negative occurred!\033[0m")
            return False
    else:
        # 誤検知（false positive）
        if random.random() < FALSE_POSITIVE_RATE:
            print("\033[91m[Sensor] False positive occurred!\033[0m")
            return True
    return true_state

fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID_SIZE - 0.5)
ax.set_ylim(-0.5, GRID_SIZE - 0.5)
ax.invert_yaxis()
ax.grid(True)

grid = generate_grid()
path = a_star(grid, START, GOAL)
history = [START]
current_index = [0]

def update(_):
    global path
    ax.clear()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, GRID_SIZE - 0.5)
    ax.set_ylim(-0.5, GRID_SIZE - 0.5)
    ax.invert_yaxis()
    ax.grid(True)

    # 障害物表示
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if grid[y][x] == 1:
                ax.text(x, y, "✕", ha="center", va="center", fontsize=14, color="red")

    # START & GOAL
    sx, sy = START
    gx, gy = GOAL
    ax.text(sx, sy, "START", color="green", ha="center", va="center")
    ax.text(gx, gy, "GOAL", color="blue", ha="center", va="center")

    # 再探索
    path = a_star(grid, history[-1], GOAL)

    if not path:
        ax.set_title("No path - Adding obstacle avoidance...")
        return

    if history[-1] == GOAL:
        ax.set_title("✅ Goal Reached")
        return

    # 次の位置に移動
    if len(path) > 1:
        next_pos = path[1]
    else:
        next_pos = path[0]

    # センサーで障害物判定（true_state = 実際に障害物があるか）
    tx, ty = next_pos
    true_state = (grid[ty][tx] == 1)
    sensed_state = sense_with_noise(true_state)

    # 検知結果に応じて動く／障害物記録
    if sensed_state:
        grid[ty][tx] = 1  # 誤検知でも地図上に障害物として記録
        ax.set_title("Obstacle detected! Replanning...")
    else:
        history.append(next_pos)

    # 動的障害物を追加
    add_dynamic_obstacle(grid, history)

    # 描画
    xs, ys = zip(*history)
    ax.plot(xs, ys, "b--")
    ax.plot(xs[-1], ys[-1], "ro")

ani = animation.FuncAnimation(fig, update, interval=800)
plt.show()
