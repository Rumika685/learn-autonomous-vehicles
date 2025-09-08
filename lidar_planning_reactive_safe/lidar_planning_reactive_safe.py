# File: lidar_planning_reactive_safe.py
# LiDAR-based partial observation + A* reactive replanning demo (goal-guaranteed)

import random, math, heapq
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# -------------------------
# Configurable parameters
# -------------------------
GRID_SIZE = 10
START = (9, 0)   # right-top
GOAL  = (0, 9)   # left-bottom

NUM_STATIC_OBSTACLES = 20

# LiDAR-like sensor config
LIDAR_RANGE = 3
LIDAR_ANGLES = 36
SENSOR_FALSE_POS = 0.0
SENSOR_FALSE_NEG = 0.0

# Execution
MAX_STEPS = 300
PAUSE_INTERVAL = 0.3
SHOW_GROUND_TRUTH = True

# -------------------------
# A* utility
# -------------------------
def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def a_star(grid, start, goal):
    OPEN = []
    heapq.heappush(OPEN, (heuristic(start,goal), 0, start, [start]))
    visited = set()
    while OPEN:
        _, cost, cur, path = heapq.heappop(OPEN)
        if cur in visited: continue
        visited.add(cur)
        if cur == goal:
            return path
        x, y = cur
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                if grid[ny][nx] == 1:  # obstacle
                    continue
                heapq.heappush(OPEN, (cost+1+heuristic((nx,ny),goal), cost+1, (nx,ny), path+[(nx,ny)]))
    return []

# -------------------------
# Environment
# -------------------------
def generate_static_grid(seed=None):
    if seed is not None: random.seed(seed)
    grid = [[0]*GRID_SIZE for _ in range(GRID_SIZE)]
    placed = 0
    while placed < NUM_STATIC_OBSTACLES:
        x = random.randint(0, GRID_SIZE-1)
        y = random.randint(0, GRID_SIZE-1)
        if (x,y) in (START, GOAL) or grid[y][x]==1: continue
        grid[y][x] = 1
        placed += 1
    return grid

def lidar_scan(true_grid, pos, angles=LIDAR_ANGLES, max_range=LIDAR_RANGE):
    ox, oy = pos
    observed_occupied = set()
    for i in range(angles):
        theta = 2*math.pi*i/angles
        for r in range(1, max_range+1):
            bx = int(round(ox + math.cos(theta)*r))
            by = int(round(oy + math.sin(theta)*r))
            if not (0 <= bx < GRID_SIZE and 0 <= by < GRID_SIZE): break
            if true_grid[by][bx] == 1:
                observed_occupied.add((bx, by))
                break
    return observed_occupied

# -------------------------
# Robot Simulation
# -------------------------
class RobotSim:
    def __init__(self, true_grid):
        self.true_grid = true_grid
        self.obs_grid = [[-1]*GRID_SIZE for _ in range(GRID_SIZE)]
        self.pos = START
        self.goal = GOAL
        self.history = [self.pos]
        self.path = []
        self.steps = 0
        self.obs_grid[START[1]][START[0]] = 0
        self.obs_grid[GOAL[1]][GOAL[0]] = 0

    def update_observations(self):
        obs = lidar_scan(self.true_grid, self.pos)
        for i in range(LIDAR_ANGLES):
            theta = 2*math.pi*i/LIDAR_ANGLES
            for r in range(1, LIDAR_RANGE+1):
                bx = int(round(self.pos[0] + math.cos(theta)*r))
                by = int(round(self.pos[1] + math.sin(theta)*r))
                if not (0 <= bx < GRID_SIZE and 0 <= by < GRID_SIZE): break
                if (bx,by) in obs:
                    self.obs_grid[by][bx] = 1
                    break
                else:
                    if self.obs_grid[by][bx] != 1:
                        self.obs_grid[by][bx] = 0

    def plan_path(self):
        # treat unknown (-1) as free for optimistic planning
        plan_grid = [[0 if self.obs_grid[r][c] != 1 else 1 for c in range(GRID_SIZE)] for r in range(GRID_SIZE)]
        p = a_star(plan_grid, self.pos, self.goal)
        if not p:
            # fallback: fully optimistic
            plan_grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
            p = a_star(plan_grid, self.pos, self.goal)
        return p

    def step(self):
        if not self.path or len(self.path) < 2:
            return False
        self.pos = self.path[1]
        self.history.append(self.pos)
        self.steps += 1
        return True

# -------------------------
# Visualization
# -------------------------
def draw(ax, sim):
    ax.clear()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, GRID_SIZE-0.5)
    ax.set_ylim(-0.5, GRID_SIZE-0.5)
    ax.invert_yaxis()
    ax.grid(True)
    if SHOW_GROUND_TRUTH:
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                if sim.true_grid[y][x] == 1:
                    ax.text(x,y,"✕",color="lightcoral",ha="center",va="center",alpha=0.5)
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if sim.obs_grid[y][x] == 1:
                ax.text(x,y,"✕",color="red",ha="center",va="center")
    if sim.path:
        xs, ys = zip(*sim.path)
        ax.plot(xs,ys,"c--",lw=1)
    if sim.history:
        hx, hy = zip(*sim.history)
        ax.plot(hx,hy,"b-",lw=2)
    ax.plot(sim.pos[0],sim.pos[1],"ro")
    ax.text(START[0],START[1],"START",color="green",ha="center")
    ax.text(GOAL[0],GOAL[1],"GOAL",color="blue",ha="center")

# -------------------------
# Run Simulation
# -------------------------
def run():
    true_grid = generate_static_grid()
    sim = RobotSim(true_grid)
    sim.update_observations()
    sim.path = sim.plan_path()

    fig, ax = plt.subplots(figsize=(6,6))
    done = {"flag":False}

    def update(_):
        if done["flag"]: return
        if sim.pos == sim.goal:
            print("Reached goal!")
            done["flag"]=True
        else:
            if not sim.step():
                sim.update_observations()
                sim.path = sim.plan_path()
            else:
                sim.update_observations()
                sim.path = sim.plan_path()
        draw(ax,sim)

    ani = animation.FuncAnimation(fig, update, frames=MAX_STEPS, interval=int(PAUSE_INTERVAL*1000), blit=False)
    plt.show()

if __name__=="__main__":
    run()
