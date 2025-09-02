# Filename: probabilistic_future_cost_pid_logging.py
import random, heapq, math, csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# --------------------------
# Config
# --------------------------
GRID = 10
START = (9, 9)   # 右下
GOAL  = (0, 0)   # 左上

STATIC_OBS = 18
DYNAMIC_OBS = 5

SEED = 42
random.seed(SEED)

# ---- Step 1: Probabilistic future prediction ----
PRED_HORIZON = 3        # 未来何コマ先まで予測するか
SPACE_SIGMA = 0.9       # 空間拡散（σ）大きいほど広く薄く
TIME_DECAY = 0.7        # 時間減衰（tが進むと弱く）
VEL_NOISE_P = { -1:0.2, 0:0.6, 1:0.2 }  # 速度ノイズ確率（各軸）

BASE_INFLATION = 5.0    # 確率→コストの重み

# ---- Step 2: 擬似 D* Lite（軽量インクリメンタル）----
LOOKAHEAD_L = 3
COST_HIGH_THRESHOLD = 2.5  # これ以上のコストセルが計画先頭に来たら再計画

# ---- Step 3: PID 制御 ----
P_GAIN = 0.7
I_GAIN = 0.05
D_GAIN = 0.12
DT     = 0.08

# ---- Step 4: ログ & 実行 ----
MAX_STEPS = 600
MAX_WAIT  = 3
CSV_PATH  = "run_log.csv"

# --------------------------
# Utils
# --------------------------
def in_bounds(x,y): return 0 <= x < GRID and 0 <= y < GRID
def neighbors4(x,y): return [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
def manhattan(a,b): return abs(a[0]-b[0])+abs(a[1]-b[1])
def euclid(a,b): return math.hypot(a[0]-b[0], a[1]-b[1])

# --------------------------
# Static map
# --------------------------
def empty_grid(): return [[0]*GRID for _ in range(GRID)]

def a_star_block_only(grid, start, goal):
    """静的障害物のみでパス確認（存在保証用）"""
    if start==goal: return [start]
    openq=[(manhattan(start,goal),0,start,None)]
    came={}; g={start:0}; vis=set()
    while openq:
        f,gv,cur,par = heapq.heappop(openq)
        if cur in vis: continue
        vis.add(cur); came[cur]=par
        if cur==goal:
            path=[]; n=cur
            while n is not None:
                path.append(n); n=came[n]
            return list(reversed(path))
        for nx,ny in neighbors4(*cur):
            if not in_bounds(nx,ny) or grid[ny][nx]==1: continue
            ng=gv+1
            if (nx,ny) not in g or ng<g[(nx,ny)]:
                g[(nx,ny)]=ng
                heapq.heappush(openq,(ng+manhattan((nx,ny),goal),ng,(nx,ny),cur))
    return []

def gen_static():
    while True:
        g=empty_grid(); k=0
        while k<STATIC_OBS:
            x,y=random.randrange(GRID),random.randrange(GRID)
            if (x,y) in (START,GOAL) or g[y][x]==1: continue
            g[y][x]=1; k+=1
        if a_star_block_only(g, START, GOAL):
            return g

# --------------------------
# A* (with soft costs)
# --------------------------
def a_star_soft(grid, start, goal, cost):
    if start==goal: return [start]
    if grid[goal[1]][goal[0]]==1 or grid[start[1]][start[0]]==1: return []
    openq=[(manhattan(start,goal),0.0,start,None)]
    came={}; g={start:0.0}; vis=set()
    while openq:
        f,gv,cur,par = heapq.heappop(openq)
        if cur in vis: continue
        vis.add(cur); came[cur]=par
        if cur==goal:
            path=[]; n=cur
            while n is not None:
                path.append(n); n=came[n]
            return list(reversed(path))
        for nx,ny in neighbors4(*cur):
            if not in_bounds(nx,ny) or grid[ny][nx]==1: continue
            step = 1.0 + cost[ny][nx]
            ng = gv + step
            if (nx,ny) not in g or ng<g[(nx,ny)]:
                g[(nx,ny)] = ng
                heapq.heappush(openq,(ng+manhattan((nx,ny),goal),ng,(nx,ny),cur))
    return []

# --------------------------
# Step 1: Probabilistic future prediction
# --------------------------
def gaussian_weight(dx,dy, sigma):
    return math.exp(-0.5*(dx*dx+dy*dy)/(sigma*sigma))

def build_prob_cost(dynamic_list):
    """未来予測に基づく占有確率→コスト"""
    prob = [[0.0]*GRID for _ in range(GRID)]
    for (x,y,vx,vy) in dynamic_list:
        # t=0..H の確率分布を合成
        # 速度ノイズを各軸で畳み込み：vx+nvx, vy+nvy
        for t in range(PRED_HORIZON+1):
            # ノイズ付き速度候補
            vel_candidates=[]
            for nvx,pvx in VEL_NOISE_P.items():
                for nvy,pvy in VEL_NOISE_P.items():
                    vel_candidates.append((vx+nvx, vy+nvy, pvx*pvy))
            # 各候補の将来位置（x+v*t, y+v*t）
            for tvx,tvy,pv in vel_candidates:
                px = x + tvx*t
                py = y + tvy*t
                # 位置は格子上に拡散（ガウス窓）
                for gy in range(GRID):
                    for gx in range(GRID):
                        if not in_bounds(gx,gy): continue
                        w = gaussian_weight(gx-px, gy-py, SPACE_SIGMA)
                        # 時間減衰
                        prob[gy][gx] += pv * (TIME_DECAY**t) * w

    # 正規化はせずスカラー化（ソフトコスト）
    cost = [[BASE_INFLATION*prob[y][x] for x in range(GRID)] for y in range(GRID)]
    return cost, prob

# --------------------------
# Dynamics
# --------------------------
def spawn_dynamic(grid, n):
    dyn=[]
    tries=0
    while len(dyn)<n and tries<5000:
        tries+=1
        x,y = random.randrange(GRID), random.randrange(GRID)
        if (x,y) in (START,GOAL) or grid[y][x]==1: continue
        vx,vy = random.choice([-1,0,1]), random.choice([-1,0,1])
        dyn.append([x,y,vx,vy])
    return dyn

def move_dynamic(grid, dyn, car_cell):
    occupied=set()
    for d in dyn: occupied.add((d[0],d[1]))
    new=[]
    for (x,y,vx,vy) in dyn:
        # まず現在の速度で候補
        cand=[(x+vx,y+vy,vx,vy)]
        # ランダム速度候補も少し
        for _ in range(4):
            dvx, dvy = random.choice([-1,0,1]), random.choice([-1,0,1])
            cand.append((x+dvx,y+dvy,dvx,dvy))
        random.shuffle(cand)
        chosen=(x,y,vx,vy)
        for nx,ny,nvx,nvy in cand:
            if not in_bounds(nx,ny): continue
            if grid[ny][nx]==1: continue
            if (nx,ny) in (START,GOAL) or (nx,ny)==car_cell: continue
            if (nx,ny) in occupied: continue
            chosen=(nx,ny,nvx,nvy)
            occupied.add((nx,ny))
            break
        new.append([chosen[0],chosen[1],chosen[2],chosen[3]])
    return new

# --------------------------
# Incremental replanning (pseudo D* Lite)
# --------------------------
def need_replan(plan, cost, dynamics, cur_cell):
    """先頭 L ステップの安全性/コストを評価し、必要なら再計画"""
    if not plan or len(plan)<2:
        return True
    # 現在位置を plan[0] に合わせる
    if plan[0] != cur_cell:
        return True
    look = min(LOOKAHEAD_L, len(plan)-1)
    dyn_now = {(dx,dy) for (dx,dy,_,_) in dynamics}
    for i in range(1, look+1):
        x,y = plan[i]
        if (x,y) in dyn_now:      # 直近に動的がいれば再計画
            return True
        if cost[y][x] >= COST_HIGH_THRESHOLD:
            return True
    return False

# --------------------------
# Logging
# --------------------------
class RunLogger:
    def __init__(self):
        self.rows=[]
        self.replans=0
        self.waits=0
    def step(self, step, car, dynamics, plan, cost):
        # 最小動的距離
        mind=999.0
        for (dx,dy,_,_) in dynamics:
            mind=min(mind, euclid(car,(dx,dy)))
        # 先頭 L ステップの平均コスト
        avgc=0.0; cnt=0
        if plan:
            for i in range(1, min(LOOKAHEAD_L+1, len(plan))):
                x,y=plan[i]; avgc+=cost[y][x]; cnt+=1
        avgc = (avgc/cnt) if cnt else 0.0
        self.rows.append([step, car[0], car[1], f"{mind:.3f}", f"{avgc:.3f}", self.replans, self.waits])
    def inc_replan(self): self.replans+=1
    def inc_wait(self): self.waits+=1
    def save(self, path):
        with open(path, "w", newline="", encoding="utf-8") as f:
            w=csv.writer(f)
            w.writerow(["step","car_x","car_y","min_dyn_dist","lookahead_avg_cost","replans","waits"])
            w.writerows(self.rows)

# --------------------------
# Main sim
# --------------------------
grid = gen_static()
dynamic = spawn_dynamic(grid, DYNAMIC_OBS)

car_cell = START
car_pos  = [float(START[0]), float(START[1])]
ex_int   = [0.0, 0.0]      # 積分
ex_prev  = [0.0, 0.0]      # 微分用

# 初期計画
infl_cost, _ = build_prob_cost(dynamic)
plan = a_star_soft(grid, car_cell, GOAL, infl_cost)

forward_hist = [car_cell]
back_hist    = []
wait = 0
goal = False
step = 0

logger = RunLogger()

# Plot
fig, ax = plt.subplots(figsize=(6,6))
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID-0.5); ax.set_ylim(-0.5, GRID-0.5)
ax.set_xticks(range(GRID)); ax.set_yticks(range(GRID))
ax.grid(True)

static_marks=[]; dyn_marks=[]
f_line, = ax.plot([], [], "b-", lw=2)
b_line, = ax.plot([], [], "m-", lw=2)
p_line, = ax.plot([], [], "c--", lw=1, alpha=0.7)
car_pt, = ax.plot([], [], "ro", ms=8)
ax.text(START[0], START[1], "START", color="green", ha="center", va="center")
ax.text(GOAL[0],  GOAL[1],  "GOAL",  color="blue",  ha="center", va="center")

def draw_static():
    global static_marks
    for t in static_marks: t.remove()
    static_marks=[]
    for y in range(GRID):
        for x in range(GRID):
            if grid[y][x]==1:
                static_marks.append(ax.text(x,y,"■",ha="center",va="center", color="black"))

def draw_dynamic():
    global dyn_marks
    for t in dyn_marks: t.remove()
    dyn_marks=[]
    for (x,y,_,_) in dynamic:
        dyn_marks.append(ax.text(x,y,"■",ha="center",va="center", color="orange"))

def update(_):
    global dynamic, infl_cost, plan, car_cell, car_pos, ex_int, ex_prev
    global wait, goal, step

    if goal:
        ax.set_title("Goal reached")
        return f_line, b_line, p_line, car_pt

    step += 1
    if step > MAX_STEPS:
        ax.set_title("Safety stop (max steps)")
        logger.save(CSV_PATH)
        return f_line, b_line, p_line, car_pt

    # 動的更新 → 予測→コスト
    dynamic[:] = move_dynamic(grid, dynamic, car_cell)
    infl_cost, _ = build_prob_cost(dynamic)

    # 擬似 D* Lite: 必要時のみ再計画
    if need_replan(plan, infl_cost, dynamic, car_cell):
        plan = a_star_soft(grid, car_cell, GOAL, infl_cost)
        logger.inc_replan()

    if not plan or len(plan) < 2:
        # 待機 or バックトラック
        wait += 1; logger.inc_wait()
        if wait >= MAX_WAIT and len(forward_hist) > 1:
            prev = forward_hist[-2]
            back_hist.append((car_cell, prev))
            car_cell = prev
            car_pos[0], car_pos[1] = float(car_cell[0]), float(car_cell[1])
            forward_hist.pop()
            wait = 0
    else:
        wait = 0
        nxt = plan[1]
        occ_now = {(dx,dy) for (dx,dy,_,_) in dynamic}
        if nxt in occ_now:
            wait += 1; logger.inc_wait()
        else:
            car_cell = nxt
            forward_hist.append(car_cell)

    # PID 追従
    tx, ty = float(car_cell[0]), float(car_cell[1])
    errx, erry = tx-car_pos[0], ty-car_pos[1]
    ex_int[0] += errx*DT; ex_int[1] += erry*DT
    dx = (errx - ex_prev[0])/DT; dy = (erry - ex_prev[1])/DT
    ex_prev[0], ex_prev[1] = errx, erry

    ux = P_GAIN*errx + I_GAIN*ex_int[0] + D_GAIN*dx
    uy = P_GAIN*erry + I_GAIN*ex_int[1] + D_GAIN*dy
    car_pos[0] += ux*DT; car_pos[1] += uy*DT

    # ログ
    logger.step(step, car_cell, dynamic, plan, infl_cost)

    # ゴール判定
    if car_cell == GOAL:
        goal = True
        logger.save(CSV_PATH)

    # 描画
    draw_static(); draw_dynamic()
    if plan:
        px,py = zip(*plan); p_line.set_data(px,py)
    else:
        p_line.set_data([],[])

    if forward_hist:
        fx,fy = zip(*forward_hist); f_line.set_data(fx,fy)
    if back_hist:
        bx = [seg[0][0] for seg in back_hist] + [back_hist[-1][1][0]]
        by = [seg[0][1] for seg in back_hist] + [back_hist[-1][1][1]]
        b_line.set_data(bx,by)

    car_pt.set_data([car_pos[0]],[car_pos[1]])
    ax.set_title(f"step={step} waits={wait} replans={logger.replans}")
    return f_line, b_line, p_line, car_pt

draw_static(); draw_dynamic()
f_line.set_data([START[0]],[START[1]])
car_pt.set_data([START[0]],[START[1]])
ani = animation.FuncAnimation(fig, update, interval=int(1000*DT), blit=False)
plt.tight_layout()
plt.show()

# print summary when window closes (best-effort)
try:
    import os
    if os.path.exists(CSV_PATH):
        print(f"[LOG] saved: {CSV_PATH}")
except:
    pass
