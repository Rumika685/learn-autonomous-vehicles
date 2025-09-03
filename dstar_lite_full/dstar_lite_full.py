import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
import random

# === グリッド設定 ===
GRID = 10
START = (9, 0)   # 右下
GOAL = (0, 9)    # 左上

# === D* Lite 用のデータ構造 ===
class DStarLite:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.g = {}
        self.rhs = {}
        self.U = []
        self.km = 0
        for y in range(GRID):
            for x in range(GRID):
                self.g[(x, y)] = float("inf")
                self.rhs[(x, y)] = float("inf")
        self.rhs[goal] = 0
        heapq.heappush(self.U, (self.calculateKey(goal), goal))

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def calculateKey(self, s):
        val = min(self.g[s], self.rhs[s])
        return (val + self.heuristic(self.start, s) + self.km, val)

    def get_neighbors(self, s):
        (x, y) = s
        nbrs = []
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < GRID and 0 <= ny < GRID and self.grid[ny][nx] == 0:
                nbrs.append((nx, ny))
        return nbrs

    def updateVertex(self, u):
        if u != self.goal:
            self.rhs[u] = min([self.g[s] + 1 for s in self.get_neighbors(u)] or [float("inf")])
        if u in [s for k, s in self.U]:
            self.U = [(k, s) for k, s in self.U if s != u]
            heapq.heapify(self.U)
        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.U, (self.calculateKey(u), u))

    def computeShortestPath(self):
        while self.U and (self.U[0][0] < self.calculateKey(self.start) or self.rhs[self.start] != self.g[self.start]):
            k_old, u = heapq.heappop(self.U)
            if k_old < self.calculateKey(u):
                heapq.heappush(self.U, (self.calculateKey(u), u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.get_neighbors(u):
                    self.updateVertex(s)
            else:
                g_old = self.g[u]
                self.g[u] = float("inf")
                for s in self.get_neighbors(u) + [u]:
                    self.updateVertex(s)

    def getPath(self):
        # 現在の g 値に基づいて最短経路を復元
        path = [self.start]
        current = self.start
        while current != self.goal:
            neighbors = self.get_neighbors(current)
            if not neighbors:
                return path
            current = min(neighbors, key=lambda s: self.g[s])
            if self.g[current] == float("inf"):
                break
            path.append(current)
        return path

# === 初期グリッド生成 ===
def generate_grid():
    grid = [[0 for _ in range(GRID)] for _ in range(GRID)]
    # ランダム障害物を20個
    count = 0
    while count < 20:
        x, y = random.randint(0, GRID-1), random.randint(0, GRID-1)
        if (x, y) not in (START, GOAL):
            grid[y][x] = 1
            count += 1
    return grid

# === 描画 ===
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID-0.5)
ax.set_ylim(-0.5, GRID-0.5)
ax.invert_yaxis()
ax.grid(True)

grid = generate_grid()
dstar = DStarLite(grid, START, GOAL)
dstar.computeShortestPath()
path = dstar.getPath()

history = []

def update(frame):
    global path
    ax.clear()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, GRID-0.5)
    ax.set_ylim(-0.5, GRID-0.5)
    ax.invert_yaxis()
    ax.grid(True)

    # 障害物を描画
    for y in range(GRID):
        for x in range(GRID):
            if grid[y][x] == 1:
                ax.add_patch(plt.Rectangle((x-0.5,y-0.5),1,1,color="black"))

    # START・GOALを毎回描画
    sx, sy = START
    gx, gy = GOAL
    ax.text(sx, sy, "START", color="green", ha="center", va="center", fontsize=10, fontweight="bold")
    ax.text(gx, gy, "GOAL", color="blue", ha="center", va="center", fontsize=10, fontweight="bold")

    # 経路を再計算
    dstar.computeShortestPath()
    path = dstar.getPath()

    if not path:
        ax.set_title("No path found")
        return []

    # 車の移動
    current_pos = path[min(frame, len(path)-1)]
    history.append(current_pos)

    # 経路と車を描画
    xs, ys = zip(*history)
    ax.plot(xs, ys, "b-")
    ax.plot(current_pos[0], current_pos[1], "ro")

    # ゴールチェック
    if current_pos == GOAL:
        ax.set_title("Goal Reached ✅")
    else:
        ax.set_title("D* Lite Pathfinding")

    return []

ani = animation.FuncAnimation(fig, update, interval=500, blit=False, save_count=200)
plt.show()
