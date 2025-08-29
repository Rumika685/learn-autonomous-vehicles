import random
import heapq
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ======== Config ========
GRID_SIZE = 10
START = (9, 9)   # 右下（yは下に行くほど大きい表示にするため invert_yaxis はOFFにする）
GOAL  = (0, 0)   # 左上
STATIC_OBS = 15  # 静的障害物の個数
DYNAMIC_OBS = 15  # 動的障害物の個数
SEED = 42        # 乱数固定（再現性）
INTERVAL_MS = 500
MAX_WAIT = 3     # A*失敗が連続したら後退に切替
MAX_STEPS = 200  # セーフティブレーキ（無限ループ防止）

random.seed(SEED)

# ======== A* ========
def a_star(grid, start, goal):
    """grid[y][x]==1 は障害物。4近傍でマンハッタンヒューリスティック。"""
    if start == goal:
        return [start]
    if grid[goal[1]][goal[0]] == 1 or grid[start[1]][start[0]] == 1:
        return []

    h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])
    openq = [(h(start, goal), 0, start, None)]
    came = {}
    cost = {start: 0}
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
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nx, ny = cx+dx, cy+dy
            if 0<=nx<GRID_SIZE and 0<=ny<GRID_SIZE and grid[ny][nx]==0:
                ng = g+1
                if (nx,ny) not in cost or ng < cost[(nx,ny)]:
                    cost[(nx,ny)] = ng
                    f = ng + h((nx,ny), goal)
                    heapq.heappush(openq, (f, ng, (nx,ny), cur))
    return []

# ======== Map generation ========
def empty_grid():
    return [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

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
        # 経路チェック
        if a_star(grid, START, GOAL):
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
    """動的障害物をランダムウォーク（上下左右かその場）。他障害物・車位置・Start/Goalに侵入禁止。"""
    occupied = set(dyn_list)  # 現在の占有
    new_list = []
    for (x,y) in dyn_list:
        candidates = [(x,y),(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
        random.shuffle(candidates)
        for nx, ny in candidates:
            if 0<=nx<GRID_SIZE and 0<=ny<GRID_SIZE:
                if grid[ny][nx]==0 and (nx,ny) not in occupied and (nx,ny) not in forbidden:
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
backtracked = []         # 後退の軌跡（マゼンタ）
cur_plan = a_star_with_dynamic = []  # 現在のA*経路
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
# NOTE: invert_yaxis() を使わず、「(0,0)が左上」の見た目にはしない（y上向き）→ (9,9)が右下。

# Persistent artists
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
    """動的障害物を壁扱いしてA*再計算"""
    temp = [row[:] for row in grid]
    for (dx,dy) in dynamic_obs:
        temp[dy][dx] = 1
    return a_star(temp, car, GOAL)

def update(_):
    global car, dynamic_obs, cur_plan, wait_count, step_count, goal_reached

    if goal_reached:
        # 到達後は静止表示を維持
        return forward_line, back_line, car_dot, plan_line

    step_count += 1
    if step_count > MAX_STEPS:
        ax.set_title("Stopped by safety brake (MAX_STEPS).")
        return forward_line, back_line, car_dot, plan_line

    # 1) 動的障害物を移動（車位置/Start/Goal/静的障害物は侵入不可）
    forbidden = set(path_taken) | set([car, START, GOAL])
    dynamic_obs[:] = move_dynamic_obstacles(grid, dynamic_obs, forbidden)

    # 2) 経路が無い or 次の一歩がふさがれたらA*再計算
    cur_plan = recompute_plan()

    # 3) 経路が無い → 待機カウント。一定回数超えたら後退。
    if not cur_plan or len(cur_plan) < 2:
        wait_count += 1
        if wait_count >= MAX_WAIT and len(path_taken) > 1:
            # バックトラック（一歩戻る）
            prev = path_taken[-2]
            backtracked.append((car, prev))
            car = prev
            path_taken.pop()
            wait_count = 0
        # 何もできない（待機）
    else:
        # 4) 経路あり → 次の一歩へ
        wait_count = 0
        nxt = cur_plan[1]
        # 動的障害物がちょうど次の一歩に来たら、無理せず待機
        if nxt in dynamic_obs:
            wait_count += 1
        else:
            car = nxt
            path_taken.append(car)

    # 5) ゴール判定
    if car == GOAL:
        goal_reached = True
        ax.set_title("✅ Goal reached")
    else:
        ax.set_title("Hybrid planner: A* + Backtrack + Dynamic obstacles")

    # 6) 再描画（消さずに上書き）
    draw_static()
    draw_dynamic()

    # A*プラン（点線）
    if cur_plan and len(cur_plan) >= 2:
        px, py = zip(*cur_plan)
        plan_line.set_data(px, py)
    else:
        plan_line.set_data([], [])

    # 前進軌跡（青）
    if len(path_taken) >= 1:
        fx, fy = zip(*path_taken)
        forward_line.set_data(fx, fy)

    # 後退軌跡（マゼンタ）
    if backtracked:
        bx = [seg[0][0] for seg in backtracked] + [backtracked[-1][1][0]]
        by = [seg[0][1] for seg in backtracked] + [backtracked[-1][1][1]]
        back_line.set_data(bx, by)

    # 車位置
    car_dot.set_data([car[0]], [car[1]])

    return forward_line, back_line, car_dot, plan_line

# 初回描画
draw_static()
draw_dynamic()
forward_line.set_data([START[0]], [START[1]])
car_dot.set_data([START[0]], [START[1]])
ax.set_title("Hybrid planner: A* + Backtrack + Dynamic obstacles")

ani = animation.FuncAnimation(fig, update, interval=INTERVAL_MS, blit=False)
plt.tight_layout()
plt.show()
