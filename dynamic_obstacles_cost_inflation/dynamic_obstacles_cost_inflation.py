import random
import heapq
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ======== Config ========
GRID_SIZE = 10
START = (9, 9)   # 右下
GOAL  = (0, 0)   # 左上
STATIC_OBS = 15
DYNAMIC_OBS = 3
SEED = 42
INTERVAL_MS = 500
MAX_WAIT = 3
MAX_STEPS = 220

# Cost inflation settings
INFLATION_RADIUS = 2        # 近傍半径（マンハッタン or ユークリッドは下で切替）
INFLATION_METRIC = "euclid" # "manhattan" か "euclid"
INFLATION_WEIGHT = 3.0      # 係数（大きいほど避ける）
# コスト関数： base_cost + INFLATION_WEIGHT * f(distance)
# f(d) は 1/d or (R+1-d) など色々試せます。ここでは 1/(d+1) を採用。

random.seed(SEED)

# ======== Utilities ========
def empty_grid():
    return [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

def in_bounds(x, y):
    return 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE

def neighbors4(x, y):
    return [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]

def dist(a, b):
    if INFLATION_METRIC == "manhattan":
        return abs(a[0]-b[0]) + abs(a[1]-b[1])
    else:
        return math.hypot(a[0]-b[0], a[1]-b[1])

# ======== Cost map from dynamic obstacles ========
def build_inflation_cost(dynamic_obs):
    """動的障害物の周辺に連続コストを付与（距離が近いほど高コスト）"""
    cost = [[0.0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    if not dynamic_obs:
        return cost
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            c = 0.0
            for (ox, oy) in dynamic_obs:
                d = dist((x,y), (ox,oy))
                if d <= INFLATION_RADIUS:
                    c += INFLATION_WEIGHT * (1.0 / (d + 1.0))
            cost[y][x] = c
    return cost

# ======== A* (with inflation cost) ========
def a_star_with_cost(grid, start, goal, inflation_cost):
    """grid[y][x]==1 は静的障害物。inflation_cost は追加コスト（動的障害物の近傍ペナルティ）"""
    if start == goal:
        return [start]
    if grid[goal[1]][goal[0]] == 1 or grid[start[1]][start[0]] == 1:
        return []

    h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])  # Manhattan heuristic
    openq = [(h(start, goal), 0.0, start, None)]
    came = {}
    gcost = {start: 0.0}
    visited = set()

    while openq:
        _, g, cur, parent = heapq.heappop(openq)
        if cur in visited:
            continue
        visited.add(cur)
        came[cur] = parent

        if cur == goal:
            # reconstruct
            path = []
            n = cur
            while n is not None:
                path.append(n)
                n = came[n]
            return list(reversed(path))

        cx, cy = cur
        for nx, ny in neighbors4(cx, cy):
            if not in_bounds(nx, ny): 
                continue
            if grid[ny][nx] == 1:     # 静的障害物は通行不可
                continue
            step_cost = 1.0 + inflation_cost[ny][nx]  # ここで動的近傍ペナルティ
            ng = g + step_cost
            if (nx, ny) not in gcost or ng < gcost[(nx,ny)]:
                gcost[(nx,ny)] = ng
                f = ng + h((nx,ny), goal)
                heapq.heappush(openq, (f, ng, (nx,ny), cur))
    return []

# ======== Map generation ========
def generate_static_obstacles():
    """A*でSTART→GOALに経路があるまで静的障害物を再生成"""
    while True:
        grid = empty_grid()
        placed = 0
        while placed < STATIC_OBS:
            x = random.randint(0, GRID_SIZE-1)
            y = random.randint(0, GRID_SIZE-1)
            if (x,y) in (START, GOAL): 
                continue
            if grid[y][x]==0:
                grid[y][x] = 1
                placed += 1
        # 経路チェック（動的コストなしで確認）
        if a_star_with_cost(grid, START, GOAL, [[0]*GRID_SIZE for _ in range(GRID_SIZE)]):
            return grid

def spawn_dynamic_obstacles(grid, n):
    dyn = set()
    tries = 0
    while len(dyn) < n and tries < 1000:
        tries += 1
        x = random.randint(0, GRID_SIZE-1)
        y = random.randint(0, GRID_SIZE-1)
        if (x,y) in (START, GOAL): 
            continue
        if grid[y][x]==0 and (x,y) not in dyn:
            dyn.add((x,y))
    return list(dyn)

def move_dynamic_obstacles(grid, dyn_list, forbidden):
    """ランダムウォーク（上下左右 or その場）。他障害物・車位置・Start/Goalは侵入禁止。"""
    occupied = set(dyn_list)
    new_list = []
    for (x,y) in dyn_list:
        candidates = [(x,y),(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        random.shuffle(candidates)
        for nx, ny in candidates:
            if in_bounds(nx, ny) and grid[ny][nx]==0 and (nx,ny) not in occupied and (nx,ny) not in forbidden:
                new_list.append((nx,ny))
                occupied.add((nx,ny))
                break
        else:
            new_list.append((x,y))  # 動けないときはその場
    return new_list

# ======== Simulation state ========
grid = generate_static_obstacles()
dynamic_obs = spawn_dynamic_obstacles(grid, DYNAMIC_OBS)

car = START
path_taken = [car]       # 前進の軌跡（青）
backtracked = []         # 後退の軌跡（マゼンタ）: segment tuples ((from),(to))
cur_plan = []
wait_count = 0
step_count = 0
goal_reached = False

# ======== Plot setup ========
fig, ax = plt.subplots(figsize=(6,6))
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID_SIZE-0.5)
ax.set_ylim(-0.5, GRID_SIZE-0.5)
ax.set_xticks(range(GRID_SIZE))
ax.set_yticks(range(GRID_SIZE))
ax.grid(True)

static_texts = []
dyn_texts = []
start_text = ax.text(START[0], START[1], "START", ha="center", va="center", color="green", fontsize=10)
goal_text  = ax.text(GOAL[0],  GOAL[1],  "GOAL",  ha="center", va="center", color="blue",  fontsize=10)
forward_line, = ax.plot([], [], "b-", linewidth=2, label="forward")
back_line,    = ax.plot([], [], "m-", linewidth=2, label="backtrack")
car_dot,      = ax.plot([], [], "ro", markersize=8, label="car")
plan_line,    = ax.plot([], [], "c--", linewidth=1, alpha=0.6, label="A* plan")

ax.legend(loc="upper right")

def draw_static():
    for t in static_texts:
        t.remove()
    static_texts.clear()
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if grid[y][x] == 1:
                static_texts.append(ax.text(x, y, "■", ha="center", va="center", color="black", fontsize=12))

def draw_dynamic():
    for t in dyn_texts:
        t.remove()
    dyn_texts.clear()
    for (x,y) in dynamic_obs:
        dyn_texts.append(ax.text(x, y, "■", ha="center", va="center", color="orange", fontsize=12))

def recompute_plan():
    """動的障害物を壁にせず、コストとして“避け気味”にする"""
    infl = build_inflation_cost(dynamic_obs)
    return a_star_with_cost(grid, car, GOAL, infl)

def update(_):
    global car, dynamic_obs, cur_plan, wait_count, step_count, goal_reached

    if goal_reached:
        return forward_line, back_line, car_dot, plan_line

    step_count += 1
    if step_count > MAX_STEPS:
        ax.set_title("Stopped by safety brake (MAX_STEPS).")
        return forward_line, back_line, car_dot, plan_line

    # 1) 動的障害物を移動
    forbidden = set(path_taken) | {car, START, GOAL}
    dynamic_obs[:] = move_dynamic_obstacles(grid, dynamic_obs, forbidden)

    # 2) プラン再計算（コスト付き）
    cur_plan = recompute_plan()

    # 3) 経路がない → 待機、一定回数で後退
    if not cur_plan or len(cur_plan) < 2:
        wait_count += 1
        if wait_count >= MAX_WAIT and len(path_taken) > 1:
            prev = path_taken[-2]
            backtracked.append((car, prev))
            car = prev
            path_taken.pop()
            wait_count = 0
    else:
        # 4) 経路あり → 次の一歩へ。ただし次セルに動的障害物が居座ったら待機
        wait_count = 0
        nxt = cur_plan[1]
        if nxt in dynamic_obs:
            wait_count += 1
        else:
            car = nxt
            path_taken.append(car)

    # 5) ゴール判定
    if car == GOAL:
        goal_reached = True
        ax.set_title("✅ Goal reached (cost-inflated planner)")
    else:
        ax.set_title(f"Cost-inflated A* + Backtrack | wait={wait_count}")

    # 6) 可視化
    draw_static()
    draw_dynamic()

    if cur_plan and len(cur_plan) >= 2:
        px, py = zip(*cur_plan)
        plan_line.set_data(px, py)
    else:
        plan_line.set_data([], [])

    if len(path_taken) >= 1:
        fx, fy = zip(*path_taken)
        forward_line.set_data(fx, fy)

    if backtracked:
        bx = [seg[0][0] for seg in backtracked] + [backtracked[-1][1][0]]
        by = [seg[0][1] for seg in backtracked] + [backtracked[-1][1][1]]
        back_line.set_data(bx, by)

    car_dot.set_data([car[0]], [car[1]])
    return forward_line, back_line, car_dot, plan_line

# 初期描画
draw_static()
draw_dynamic()
forward_line.set_data([START[0]], [START[1]])
car_dot.set_data([START[0]], [START[1]])
ax.set_title("Cost-inflated A* + Backtrack (start)")

ani = animation.FuncAnimation(fig, update, interval=INTERVAL_MS, blit=False)
plt.tight_layout()
plt.show()
