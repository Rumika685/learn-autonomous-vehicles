import random
import matplotlib.pyplot as plt
import numpy as np
import heapq

# --- A* Pathfinding ---
def a_star(grid, start, goal):
    GRID_SIZE = len(grid)
    h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])
    open_set = [(h(start, goal), 0, start, [])]
    visited = set()
    while open_set:
        _, cost, current, path = heapq.heappop(open_set)
        if current in visited: continue
        visited.add(current)
        path = path + [current]
        if current == goal: return path
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = current[0]+dx, current[1]+dy
            if 0<=nx<GRID_SIZE and 0<=ny<GRID_SIZE and grid[ny][nx]==0:
                heapq.heappush(open_set,(cost+1+h((nx,ny),goal), cost+1,(nx,ny),path))
    return []

# --- Generate Grid ---
def generate_grid(size=20, density=0.2, start=(0,0), goal=None):
    if goal is None:
        goal = (size-1, size-1)
    grid = [[0]*size for _ in range(size)]
    for y in range(size):
        for x in range(size):
            if (x,y) not in (start,goal) and random.random() < density:
                grid[y][x] = 1
    return grid

# --- LiDAR Simulation ---
def lidar_scan(grid, pos, resolution=16, max_range=10,
               false_pos=0.01, false_neg=0.01):
    GRID_SIZE = len(grid)
    x0, y0 = pos
    noisy_grid = [[0]*GRID_SIZE for _ in range(GRID_SIZE)]

    for i in range(resolution):
        angle = 2*np.pi * i/resolution
        dx, dy = np.cos(angle), np.sin(angle)
        for r in range(1, max_range+1):
            x, y = int(round(x0+dx*r)), int(round(y0+dy*r))
            if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                if grid[y][x] == 1:  # 実際に障害物
                    if random.random() > false_neg:
                        noisy_grid[y][x] = 1
                    break
                else:  # 空きマスに誤検知
                    if random.random() < false_pos:
                        noisy_grid[y][x] = 1
    return noisy_grid

# --- Run Experiment ---
def run_experiment(size, resolution, trials=30):
    success = 0
    for _ in range(trials):
        grid = generate_grid(size=size, density=0.2)
        start, goal = (0,0), (size-1, size-1)
        noisy_grid = lidar_scan(grid, start, resolution=resolution, max_range=size//4)
        path = a_star(noisy_grid, start, goal)
        if path and path[-1]==goal:
            success += 1
    return success/trials

# --- Main ---
map_sizes = [20, 50]
resolutions = [8, 16, 32]
results = {size: [run_experiment(size, r) for r in resolutions] for size in map_sizes}

# Plot
fig, ax = plt.subplots()
bar_width = 0.35
x = np.arange(len(resolutions))

for i, size in enumerate(map_sizes):
    ax.bar(x + i*bar_width, results[size], bar_width, label=f"{size}x{size}")

ax.set_xticks(x + bar_width/2)
ax.set_xticklabels([str(r) for r in resolutions])
ax.set_ylim(0,1)
ax.set_xlabel("LiDAR Resolution (beams)")
ax.set_ylabel("Success Rate")
ax.set_title("Effect of Map Scale on LiDAR Resolution")
ax.legend()
plt.show()
