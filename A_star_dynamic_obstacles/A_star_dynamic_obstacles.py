import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
import random

# マップサイズと障害物数
GRID_SIZE = 10
NUM_OBSTACLES = 20

# スタートとゴール
start = (9, 0)
goal = (0, 9)

# 方向（上下左右）
moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# ランダムな障害物を生成
def generate_obstacles():
    obstacles = set()
    while len(obstacles) < NUM_OBSTACLES:
        ox = random.randint(0, GRID_SIZE - 1)
        oy = random.randint(0, GRID_SIZE - 1)
        if (ox, oy) != start and (ox, oy) != goal:
            obstacles.add((ox, oy))
    return obstacles

# A*探索
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, obstacles):
    frontier = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal:
            break

        for dx, dy in moves:
            next_node = (current[0] + dx, current[1] + dy)
            if 0 <= next_node[0] < GRID_SIZE and 0 <= next_node[1] < GRID_SIZE:
                if next_node in obstacles:
                    continue
                new_cost = cost_so_far[current] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(next_node, goal)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

    # 経路の復元
    if goal not in came_from:
        return None
    path = []
    node = goal
    while node != start:
        path.append(node)
        node = came_from[node]
    path.append(start)
    path.reverse()
    return path

# 障害物生成
obstacles = generate_obstacles()

# 経路探索
path = a_star(start, goal, obstacles)
if path is None:
    print("🛑 No path to goal!")
    path = [start]  # 最低限スタートだけ表示

# 描画準備
fig, ax = plt.subplots()
ax.set_xlim(-1, GRID_SIZE)
ax.set_ylim(-1, GRID_SIZE)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title("Goal reached or path blocked")

# 障害物描画
for (ox, oy) in obstacles:
    ax.text(ox, oy, "✕", color="red", fontsize=14, ha='center', va='center')

# START / GOAL ラベル
sx, sy = start
gx, gy = goal
ax.text(sx, sy, "START", color="green", fontsize=10, ha='center', va='center')
ax.text(gx, gy, "GOAL", color="blue", fontsize=10, ha='center', va='center')

# 軌跡・現在位置（Line2Dと点）
trail_line, = ax.plot([], [], 'b--')
point, = ax.plot([], [], 'ro')
passed_path = []
current_index = [0]

# アニメーション更新関数
def update(frame):
    if current_index[0] >= len(path):
        return trail_line, point

    x, y = path[current_index[0]]
    passed_path.append((x, y))

    xs, ys = zip(*passed_path)
    trail_line.set_data(xs, ys)
    point.set_data([x], [y])

    current_index[0] += 1
    return trail_line, point

# アニメーション実行
ani = animation.FuncAnimation(fig, update, interval=600)
plt.show()
