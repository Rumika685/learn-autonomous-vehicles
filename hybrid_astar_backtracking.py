import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
import random

# ==== グリッド設定 ====
GRID_SIZE = 10
START = (9, 0)
GOAL = (0, 9)
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

# ==== A*探索 ====
def a_star(grid, start, goal):
    h = lambda a, b: abs(a[0] - b[0]) + abs(a[1] - b[1])
    open_set = [(h(start, goal), 0, start, [start])]
    visited = set()

    while open_set:
        _, cost, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            return path

        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            nx, ny = current[0]+dx, current[1]+dy
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and grid[ny][nx] == 0:
                new_path = path + [(nx, ny)]
                heapq.heappush(open_set, (cost+1+h((nx, ny), goal), cost+1, (nx, ny), new_path))
    return []

# ==== 探索クラス（ハイブリッド）====
class HybridCar:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.path = a_star(grid, start, goal)
        self.current = start
        self.history = [start]
        self.backtrack_mode = False

    def step(self):
        if self.current == self.goal:
            return True

        # A* ルートが残っていればその通りに進む
        if self.path and len(self.path) > 1:
            self.current = self.path[1]
            self.path = self.path[1:]
            self.history.append(self.current)
            self.backtrack_mode = False
        else:
            # A* が詰まったらバックトラック
            neighbors = [(self.current[0]+1, self.current[1]),
                         (self.current[0]-1, self.current[1]),
                         (self.current[0], self.current[1]+1),
                         (self.current[0], self.current[1]-1)]
            neighbors = [(x,y) for x,y in neighbors if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE and self.grid[y][x]==0 and (x,y) not in self.history]

            if neighbors:
                self.current = neighbors[0]
                self.history.append(self.current)
                self.backtrack_mode = True
            else:
                # 完全に詰まった場合 → 再探索
                self.path = a_star(self.grid, self.current, self.goal)
        return False

# ==== 実行 ====
grid = generate_grid()
car = HybridCar(grid, START, GOAL)

fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID_SIZE-0.5)
ax.set_ylim(-0.5, GRID_SIZE-0.5)
ax.invert_yaxis()
ax.grid(True)

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

    # 履歴
    if len(car.history) > 1:
        hx, hy = zip(*car.history)
        ax.plot(hx, hy, "gray", alpha=0.6)

    # 現在位置
    cx, cy = car.current
    ax.plot(cx, cy, "ro")

    # タイトル
    if done:
        ax.set_title("✅ Goal Reached (Hybrid A* + Backtracking)")
    elif car.backtrack_mode:
        ax.set_title("🔄 Backtracking...")
    else:
        ax.set_title("➡ Following A* Path")

    return []

ani = animation.FuncAnimation(fig, update, interval=500)
plt.show()