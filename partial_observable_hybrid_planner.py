# partial_observable_hybrid_planner.py
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq, random, math

# ============ Config ============
GRID = 10
START = (9, 0)    # 右下(描画はy反転)
GOAL  = (0, 9)    # 左上
OBSTACLES = 20
MAX_LIDAR_RANGE = 5
LIDAR_BEAMS = 16         # 360° / 16本
SHOW_TRUTH_FAINT = False # True にすると真の障害物を薄く表示（デバッグ用）
RNG_SEED = None          # 例: 42 固定すると再現

# 既知マップ: -1 unknown / 0 free / 1 blocked
UNKNOWN, FREE, BLOCKED = -1, 0, 1

# ============ Utils ============
def manhattan(a, b): return abs(a[0]-b[0]) + abs(a[1]-b[1])

def in_bounds(x, y): return 0 <= x < GRID and 0 <= y < GRID

def neighbors4(x, y):
    for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
        nx, ny = x+dx, y+dy
        if in_bounds(nx, ny):
            yield nx, ny

# ============ True grid ============
def generate_true_grid():
    g = [[0]*GRID for _ in range(GRID)]
    cnt = 0
    while cnt < OBSTACLES:
        x, y = random.randrange(GRID), random.randrange(GRID)
        if (x, y) in (START, GOAL): continue
        if g[y][x] == 0:
            g[y][x] = 1
            cnt += 1
    return g

def astar(grid01, start, goal):
    # grid01: 1=blocked, 0=free
    h = lambda p: manhattan(p, goal)
    pq = [(h(start), 0, start)]
    came = {start: None}
    gscore = {start: 0}
    while pq:
        _, gc, cur = heapq.heappop(pq)
        if cur == goal:
            # reconstruct
            path = []
            c = cur
            while c is not None:
                path.append(c)
                c = came[c]
            return path[::-1]
        for nx, ny in neighbors4(*cur):
            if grid01[ny][nx] == 1: 
                continue
            ng = gc + 1
            if ng < gscore.get((nx,ny), 1e9):
                gscore[(nx,ny)] = ng
                came[(nx,ny)] = cur
                heapq.heappush(pq, (ng + h((nx,ny)), ng, (nx,ny)))
    return None

def ensure_solvable(grid_true, max_tries=30):
    tries = 0
    while tries < max_tries:
        # try true A* on ground truth
        if astar(grid_true, START, GOAL) is not None:
            return grid_true
        # regenerate
        grid_true = generate_true_grid()
        tries += 1
    return grid_true  # 諦めて返す（ほぼ無いはず）

# ============ Sensing (LiDAR-like) ============
def raycast(grid_true, origin, angle_rad, max_range):
    # 連続空間でビームを進めつつ、最も近い障害セルに当たるまで free を列挙
    x, y = origin
    fx, fy = x + 0.5, y + 0.5  # セル中心から発射
    dx, dy = math.cos(angle_rad), math.sin(angle_rad)
    free_cells = []
    for r in [i*0.2 for i in range(1, int(max_range/0.2)+1)]:
        rx = fx + dx * r
        ry = fy + dy * r
        cx, cy = int(rx), int(ry)
        if not in_bounds(cx, cy):
            break
        if grid_true[cy][cx] == 0:
            if (cx, cy) not in free_cells:
                free_cells.append((cx, cy))
        else:
            # 最初の衝突セル
            return free_cells, (cx, cy)
    return free_cells, None

def sense_and_update(known_map, grid_true, pos):
    # 全方位ビームで更新
    free_total = set()
    hit_obstacles = set()
    for k in range(LIDAR_BEAMS):
        ang = 2*math.pi * k / LIDAR_BEAMS
        frees, hit = raycast(grid_true, pos, ang, MAX_LIDAR_RANGE)
        for c in frees: free_total.add(c)
        if hit: hit_obstacles.add(hit)
    # 自位置はFree
    free_total.add(pos)
    # 更新
    for (x, y) in free_total:
        known_map[y][x] = FREE
    for (x, y) in hit_obstacles:
        known_map[y][x] = BLOCKED

# Frontier: 未知に隣接する free セル（探索すると視界が広がる）
def collect_frontiers(known_map):
    fronts = []
    for y in range(GRID):
        for x in range(GRID):
            if known_map[y][x] == FREE:
                for nx, ny in neighbors4(x, y):
                    if known_map[ny][nx] == UNKNOWN:
                        fronts.append((x, y))
                        break
    return fronts

# 既知マップを A* 用 0/1 に変換（UNKNOWN は 0 として通行可能扱い）
def known_to_grid01(known_map):
    g = [[0]*GRID for _ in range(GRID)]
    for y in range(GRID):
        for x in range(GRID):
            g[y][x] = 1 if known_map[y][x] == BLOCKED else 0
    return g

# ============ Agent ============
class HybridPOAgent:
    def __init__(self, grid_true):
        self.grid_true = grid_true
        self.known = [[UNKNOWN]*GRID for _ in range(GRID)]
        self.pos = START
        self.history = [START]
        self.forward_path = []   # 現在追従中のA*経路
        self.backtracking = False
        sense_and_update(self.known, self.grid_true, self.pos)

    def plan(self, target):
        grid01 = known_to_grid01(self.known)
        return astar(grid01, self.pos, target)

    def step(self):
        if self.pos == GOAL:
            return True

        # 1) まず GOAL へ計画
        path = self.plan(GOAL)
        if path is None:
            # 2) フロンティア（未知に隣接する free）へ
            fronts = collect_frontiers(self.known)
            if fronts:
                # 近いフロンティア順に試す
                fronts.sort(key=lambda c: manhattan(self.pos, c))
                planned = None
                for f in fronts[:8]:
                    p = self.plan(f)
                    if p:
                        planned = p
                        break
                if planned is None:
                    # 3) それでもダメ → バックトラック（隣接で未訪問freeがあれば進む）
                    self.backtracking = True
                    for nx, ny in neighbors4(*self.pos):
                        if in_bounds(nx, ny) and self.known[ny][nx] == FREE and (nx,ny) not in self.history:
                            self.pos = (nx, ny)
                            self.history.append(self.pos)
                            sense_and_update(self.known, self.grid_true, self.pos)
                            return False
                    # 全滅なら履歴から1手戻る（経路は灰色で残す）
                    if len(self.history) > 1:
                        self.pos = self.history[-2]
                        self.history.append(self.pos)
                        sense_and_update(self.known, self.grid_true, self.pos)
                        return False
                    return False
                else:
                    self.forward_path = planned[1:]  # 現在地を除いた後続
                    self.backtracking = False
            else:
                # 未知がない＝全探索済 & GOAL不可 → 動けない
                self.backtracking = True
                return False
        else:
            self.forward_path = path[1:]
            self.backtracking = False

        # 経路があれば1歩進む
        if self.forward_path:
            self.pos = self.forward_path.pop(0)
            self.history.append(self.pos)
            # 観測して既知マップ更新
            sense_and_update(self.known, self.grid_true, self.pos)
        return self.pos == GOAL

# ============ Run & Animate ============
def main():
    if RNG_SEED is not None:
        random.seed(RNG_SEED)

    grid_true = ensure_solvable(generate_true_grid())

    agent = HybridPOAgent(grid_true)

    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, GRID-0.5)
    ax.set_ylim(-0.5, GRID-0.5)
    ax.invert_yaxis()
    ax.grid(True)

    # 永続表示のため、描画要素は都度再作成
    def draw(ax):
        ax.clear()
        ax.set_aspect("equal")
        ax.set_xlim(-0.5, GRID-0.5)
        ax.set_ylim(-0.5, GRID-0.5)
        ax.invert_yaxis()
        ax.grid(True)

        # （オプション）真の障害物を薄く表示
        if SHOW_TRUTH_FAINT:
            for y in range(GRID):
                for x in range(GRID):
                    if grid_true[y][x] == 1:
                        ax.text(x, y, "×", color="lightcoral", ha="center", va="center", fontsize=10, alpha=0.4)

        # 既知障害物（赤 X）/ 既知 Free（薄グレー・ドット）
        for y in range(GRID):
            for x in range(GRID):
                if agent.known[y][x] == BLOCKED:
                    ax.text(x, y, "✕", ha="center", va="center", fontsize=14, color="red")
                elif agent.known[y][x] == FREE:
                    ax.plot(x, y, ".", color="0.7", markersize=4)

        # スタート・ゴール
        ax.text(START[0], START[1], "START", color="green", ha="center", va="center")
        ax.text(GOAL[0],  GOAL[1],  "GOAL",  color="blue",  ha="center", va="center")

        # 履歴（前進とバックトラック色分け）
        if len(agent.history) > 1:
            forward_xy = [agent.history[0]]
            back_xy = []
            seen = set()
            for i in range(1, len(agent.history)):
                cur = agent.history[i]
                prev = agent.history[i-1]
                # 既に訪れたセルへ戻る＝バックトラックとみなす
                if cur in seen and cur != prev:
                    back_xy.append(cur)
                else:
                    forward_xy.append(cur)
                seen.add(cur)

            if len(forward_xy) > 1:
                fx, fy = zip(*forward_xy)
                ax.plot(fx, fy, "-", color="tab:blue", linewidth=2, label="forward")
            if back_xy:
                bx, by = zip(*back_xy)
                ax.plot(bx, by, "o", color="orange", markersize=3, label="backtrack")

        # 現在位置
        ax.plot(agent.pos[0], agent.pos[1], "ro", markersize=8)

        # タイトル
        if agent.pos == GOAL:
            ax.set_title("✅ Goal Reached (Partial-Observable A* + Frontier + Backtrack)")
        elif agent.backtracking:
            ax.set_title("🔄 Backtracking / Exploring Frontier")
        else:
            ax.set_title("➡ Replanning to Goal")

    def update(_):
        done = agent.step()
        draw(ax)
        if done:
            # ゴール後も画面を残すため、アニメ停止（matplotlib上は図が残る）
            ani.event_source.stop()
        return []

    draw(ax)
    global ani
    ani = animation.FuncAnimation(fig, update, interval=400)
    plt.show()

if __name__ == "__main__":
    main()