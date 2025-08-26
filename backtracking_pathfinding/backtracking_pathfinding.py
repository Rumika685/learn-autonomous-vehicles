import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

# ==== グリッド設定 ====
GRID_SIZE = 10
START = (9, 0)   # 右下
GOAL = (0, 9)    # 左上
OBSTACLE_COUNT = 20

# ==== グリッド生成 ====
def generate_grid():
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    count = 0
    while count < OBSTACLE_COUNT:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in (START, GOAL) and grid[y][x] == 0:
            grid[y][x] = 1
            count += 1
    return grid

# ==== 移動可能判定 ====
def is_free(grid, pos):
    x, y = pos
    return 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE and grid[y][x] == 0

# ==== 探索クラス ====
class BacktrackingCar:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.path = [start]        # 通過ルート
        self.visited = set([start]) # 訪問済み
        self.backtrack_mode = False

    def step(self):
        current = self.path[-1]

        # ゴール到達
        if current == self.goal:
            return True

        # 近傍候補
        neighbors = [(current[0]+1, current[1]),
                     (current[0]-1, current[1]),
                     (current[0], current[1]+1),
                     (current[0], current[1]-1)]

        # 未訪問かつ移動可能な候補を探す
        candidates = [n for n in neighbors if is_free(self.grid, n) and n not in self.visited]

        if candidates:
            # 前進
            next_pos = random.choice(candidates)  # ランダムに進む
            self.path.append(next_pos)
            self.visited.add(next_pos)
            self.backtrack_mode = False
        else:
            # バックトラック
            if len(self.path) > 1:
                self.path.pop()
                self.backtrack_mode = True
            else:
                return True  # スタートに戻って詰みの場合
        return False

# ==== 描画準備 ====
grid = generate_grid()
car = BacktrackingCar(grid, START, GOAL)

fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID_SIZE-0.5)
ax.set_ylim(-0.5, GRID_SIZE-0.5)
ax.invert_yaxis()
ax.grid(True)

# 描画要素
path_line, = ax.plot([], [], "b-")     # 通常ルート
backtrack_line, = ax.plot([], [], "r--") # バックトラックルート
point, = ax.plot([], [], "ro")

def update(_):
    done = car.step()
    ax.clear()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, GRID_SIZE-0.5)
    ax.set_ylim(-0.5, GRID_SIZE-0.5)
    ax.invert_yaxis()
    ax.grid(True)

    # 障害物
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if grid[y][x] == 1:
                ax.text(x, y, "✕", ha="center", va="center", fontsize=14, color="red")

    # スタート・ゴール
    ax.text(START[0], START[1], "START", color="green", ha="center", va="center")
    ax.text(GOAL[0], GOAL[1], "GOAL", color="blue", ha="center", va="center")

    # 経路描画
    if len(car.path) > 1:
        xs, ys = zip(*car.path)
        if car.backtrack_mode:
            ax.plot(xs, ys, "r--")  # バックトラック時は赤点線
        else:
            ax.plot(xs, ys, "b-")   # 通常は青線

    # 現在位置
    cx, cy = car.path[-1]
    ax.plot(cx, cy, "ro")

    if done:
        ax.set_title("✅ Goal Reached with Backtracking")
    return path_line, backtrack_line, point

ani = animation.FuncAnimation(fig, update, interval=500)
plt.show()
