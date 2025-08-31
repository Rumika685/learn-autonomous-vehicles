# Filename suggestion: soft_obstacle_future_cost_incremental_pid.py
import random, heapq, math, time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --------------------------
# Config
# --------------------------
GRID = 10
START = (9, 9)   # right-bottom
GOAL  = (0, 0)   # left-top

STATIC_OBS = 18
DYNAMIC_OBS = 4

SEED = 123
random.seed(SEED)

# Future prediction horizon (frames) and inflation params
PRED_HORIZON = 3
INFLATION_RADIUS = 2.0       # influence radius
INFLATION_WEIGHT = 4.0       # larger -> avoid more
INFLATION_METRIC = "euclid"  # "euclid" or "manhattan"

# incremental replanning parameters
LOOKAHEAD_CHECK_K = 2  # if planned path remains valid for next k steps, reuse
REPLAN_ON_HIGH_COST = True  # if next cell cost too high, trigger replanning
HIGH_COST_THRESHOLD = 2.5

# PID-like smooth motion params (simple P-term)
P_GAIN = 0.6
DT = 0.08  # pause time between rendering updates (sec)

MAX_STEPS = 500
MAX_WAIT = 3

# --------------------------
# Utilities
# --------------------------
def in_bounds(x,y): return 0 <= x < GRID and 0 <= y < GRID
def neighbors4(x,y):
    return [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
def manhattan(a,b): return abs(a[0]-b[0])+abs(a[1]-b[1])
def euclid(a,b): return math.hypot(a[0]-b[0], a[1]-b[1])
def dist(a,b):
    return manhattan(a,b) if INFLATION_METRIC=="manhattan" else euclid(a,b)

# --------------------------
# Grid generation
# --------------------------
def empty_grid(): return [[0]*GRID for _ in range(GRID)]
def generate_static():
    while True:
        g = empty_grid()
        p = 0
        while p < STATIC_OBS:
            x = random.randrange(GRID); y = random.randrange(GRID)
            if (x,y) in (START,GOAL): continue
            if g[y][x]==0:
                g[y][x]=1
                p+=1
        # ensure at least one path exists ignoring dynamic costs:
        if a_star(g, START, GOAL): 
            return g

# --------------------------
# A* plain (no inflation) used for validation
# --------------------------
def a_star(grid, start, goal):
    if start==goal: return [start]
    h = lambda a,b: manhattan(a,b)
    openq = [(h(start,goal), 0, start, None)]
    came = {}
    costg = {start:0}
    visited=set()
    while openq:
        f,gc,cur,parent = heapq.heappop(openq)
        if cur in visited: continue
        visited.add(cur); came[cur]=parent
        if cur==goal:
            path=[]
            n=cur
            while n is not None:
                path.append(n); n=came[n]
            return list(reversed(path))
        for nx,ny in neighbors4(*cur):
            if not in_bounds(nx,ny) or grid[ny][nx]==1: continue
            ng = gc + 1
            if (nx,ny) not in costg or ng < costg[(nx,ny)]:
                costg[(nx,ny)] = ng
                heapq.heappush(openq, (ng + h((nx,ny),goal), ng, (nx,ny), cur))
    return []

# --------------------------
# A* with inflation cost (soft obstacles)
# --------------------------
def a_star_with_inflation(grid, start, goal, inflation_cost):
    if start==goal: return [start]
    if grid[goal[1]][goal[0]]==1 or grid[start[1]][start[0]]==1: return []
    h = lambda a,b: manhattan(a,b)
    openq = [(h(start,goal), 0.0, start, None)]
    came = {}
    gvals = {start:0.0}
    visited=set()
    while openq:
        f,gc,cur,parent = heapq.heappop(openq)
        if cur in visited: continue
        visited.add(cur); came[cur]=parent
        if cur==goal:
            path=[]; n=cur
            while n is not None:
                path.append(n); n=came[n]
            return list(reversed(path))
        for nx,ny in neighbors4(*cur):
            if not in_bounds(nx,ny): continue
            if grid[ny][nx]==1: continue  # static block
            step_cost = 1.0 + inflation_cost[ny][nx]
            ng = gc + step_cost
            if (nx,ny) not in gvals or ng < gvals[(nx,ny)]:
                gvals[(nx,ny)] = ng
                heapq.heappush(openq, (ng + h((nx,ny),goal), ng, (nx,ny), cur))
    return []

# --------------------------
# Inflation cost builder with future prediction
# --------------------------
def build_inflation_cost(dynamic_list):
    C = [[0.0]*GRID for _ in range(GRID)]
    for (ox,oy, vx,vy) in dynamic_list:
        # for each predicted future step
        for t in range(PRED_HORIZON+1):
            px = ox + vx*t
            py = oy + vy*t
            # round to nearest integer cell (we allow subcell if desired)
            for y in range(GRID):
                for x in range(GRID):
                    d = dist((x,y),(px,py))
                    if d <= INFLATION_RADIUS:
                        # simple attenuation
                        C[y][x] += INFLATION_WEIGHT * (1.0/(d+1.0)) * (1.0/(t+1.0))  # future-decay
    return C

# --------------------------
# Dynamic obstacles logic (with velocity)
# --------------------------
def spawn_dynamic(grid, n):
    dyn=[]
    tries=0
    while len(dyn)<n and tries<2000:
        tries+=1
        x,y = random.randrange(GRID), random.randrange(GRID)
        if (x,y) in (START,GOAL) or grid[y][x]==1: continue
        # give small velocity -1/0/1 each axis
        vx,vy = random.choice([-1,0,1]), random.choice([-1,0,1])
        dyn.append([x,y,vx,vy])
    return dyn

def move_dyn(grid, dyn, car_pos):
    occupied=set()
    for d in dyn: occupied.add((d[0],d[1]))
    newdyn=[]
    for (x,y,vx,vy) in dyn:
        # propose next using velocity plus small randomness
        cand_positions = []
        # keep same velocity attempt
        nx,ny = x+vx, y+vy
        cand_positions.append((nx,ny,vx,vy))
        # try to jitter velocity
        for dvx in (-1,0,1):
            for dvy in (-1,0,1):
                cand_positions.append((x+dvx, y+dvy, dvx, dvy))
        random.shuffle(cand_positions)
        chosen = (x,y,vx,vy)
        for nx,ny,nvx,nvy in cand_positions:
            if not in_bounds(nx,ny): continue
            if grid[ny][nx]==1: continue
            if (nx,ny) in (START,GOAL): continue
            if (nx,ny)==car_pos: continue
            # avoid overlapping other dynamics in new positions
            if (nx,ny) in occupied: continue
            # accept
            chosen=(nx,ny,nvx,nvy)
            occupied.add((nx,ny))
            break
        newdyn.append([chosen[0],chosen[1], chosen[2], chosen[3]])
    return newdyn

# --------------------------
# Incremental replanning helper
# --------------------------
def is_plan_still_ok(plan, inflation_cost, dynamic_list, curidx, car_pos):
    """Check lookahead steps in plan: if next LOOKAHEAD_CHECK_K cells have very high cost or occupied by dynamic obstacle -> not ok"""
    # if plan too short -> not ok
    if not plan or curidx >= len(plan)-1: return False
    for k in range(1, LOOKAHEAD_CHECK_K+1):
        idx = curidx + k
        if idx >= len(plan): break
        cell = plan[idx]
        x,y = cell
        # occupied by dynamic now?
        for dx,dy,vx,vy in dynamic_list:
            if (x,y)==(dx,dy): return False
        if inflation_cost[y][x] >= HIGH_COST_THRESHOLD: 
            return False
    return True

# --------------------------
# Main simulation
# --------------------------
grid = generate_static()
dynamic = spawn_dynamic(grid, DYNAMIC_OBS)

# car state (continuous pos for smooth rendering)
car_cell = START
car_pos = [float(START[0]), float(START[1])]
path_plan = a_star(grid, car_cell, GOAL)  # initial plain plan
infl_cost = build_inflation_cost([(x,y,vx,vy) for x,y,vx,vy in dynamic])
path_plan = a_star_with_inflation(grid, car_cell, GOAL, infl_cost)

history_forward = [car_cell]
history_back = []
wait = 0
step = 0
goal_reached = False

# plotting setup
fig, ax = plt.subplots(figsize=(6,6))
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID-0.5); ax.set_ylim(-0.5, GRID-0.5)
ax.set_xticks(range(GRID)); ax.set_yticks(range(GRID))
ax.grid(True)
start_txt = ax.text(START[0], START[1], "START", color="green", ha="center", va="center")
goal_txt  = ax.text(GOAL[0],  GOAL[1],  "GOAL",  color="blue",  ha="center", va="center")
static_marks=[]
dyn_marks=[]
forward_line, = ax.plot([], [], "b-", lw=2)
back_line,    = ax.plot([], [], "m-", lw=2)
plan_line,    = ax.plot([], [], "c--", lw=1, alpha=0.6)
car_dot,      = ax.plot([], [], "ro", markersize=8)

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
    for (x,y,vx,vy) in dynamic:
        dyn_marks.append(ax.text(x,y,"■",ha="center",va="center", color="orange"))

def update_frame(_):
    global dynamic, path_plan, car_cell, car_pos, history_forward, history_back
    global wait, step, goal_reached

    if goal_reached:
        # keep final display static
        return forward_line, back_line, plan_line, car_dot

    step += 1
    if step > MAX_STEPS:
        ax.set_title("Safety stop: max steps")
        return forward_line, back_line, plan_line, car_dot

    # move dynamics first (they move independently)
    dynamic = move_dyn(grid, dynamic, car_cell)

    # build inflation cost including predicted future
    infl = build_inflation_cost([(x,y,vx,vy) for x,y,vx,vy in dynamic])

    # incremental replanning: check if existing plan is still ok
    if not is_plan_still_ok(path_plan, infl, dynamic, 0, car_cell):
        # recompute plan from current cell
        path_plan = a_star_with_inflation(grid, car_cell, GOAL, infl)

    # if no plan, try wait/backtrack policy
    if not path_plan or len(path_plan) < 2:
        wait += 1
        if wait >= MAX_WAIT and len(history_forward) > 1:
            # backtrack
            prev = history_forward[-2]
            history_back.append((car_cell, prev))
            car_cell = prev
            car_pos[0], car_pos[1] = float(car_cell[0]), float(car_cell[1])
            history_forward.pop()
            wait = 0
        else:
            # just wait
            pass
    else:
        wait = 0
        nxt = path_plan[1]
        # if next is currently occupied by a dynamic -> wait (reactive)
        occupied_now = any((nxt[0]==dx and nxt[1]==dy) for dx,dy,vx,vy in dynamic)
        if occupied_now:
            wait += 1
        else:
            # step into it
            car_cell = nxt
            history_forward.append(car_cell)
            # keep car_pos continuous; target cell center is car_cell
    # PID-like smooth interpolation to current cell
    target_x, target_y = float(car_cell[0]), float(car_cell[1])
    ex = target_x - car_pos[0]; ey = target_y - car_pos[1]
    car_pos[0] += P_GAIN * ex
    car_pos[1] += P_GAIN * ey

    # check goal
    if car_cell == GOAL:
        goal_reached = True
        ax.set_title("Goal reached (with soft-cost & future prediction)")
    else:
        ax.set_title(f"Step {step} wait={wait}")

    # draw everything
    draw_static(); draw_dynamic()

    # plan line
    if path_plan and len(path_plan) >= 1:
        px,py = zip(*path_plan); plan_line.set_data(px,py)
    else:
        plan_line.set_data([],[])

    if history_forward:
        fx,fy = zip(*history_forward); forward_line.set_data(fx,fy)
    if history_back:
        bx = [seg[0][0] for seg in history_back] + [history_back[-1][1][0]]
        by = [seg[0][1] for seg in history_back] + [history_back[-1][1][1]]
        back_line.set_data(bx,by)

    car_dot.set_data([car_pos[0]],[car_pos[1]])
    return forward_line, back_line, plan_line, car_dot

# initial draw
draw_static(); draw_dynamic()
forward_line.set_data([START[0]],[START[1]])
car_dot.set_data([START[0]],[START[1]])
ani = animation.FuncAnimation(fig, update_frame, interval=int(DT*1000), blit=False)
plt.tight_layout()
plt.show()
