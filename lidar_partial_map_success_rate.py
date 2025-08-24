import random
import matplotlib.pyplot as plt
import numpy as np
import heapq

# --- A* Pathfinding ---
def a_star(grid, start, goal):
    size = len(grid)
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
            if 0<=nx<size and 0<=ny<size and grid[ny][nx]==0:
                heapq.heappush(open_set,(cost+1+h((nx,ny),goal), cost+1,(nx,ny),path))
    return []

# --- Generate Ground Truth Grid ---
def generate_grid(size=20, density=0.25, start=(0,0), goal=None):
    if goal is None:
        goal = (size-1, size-1)
    grid = [[0]*size for _ in range(size)]
    for y in range(size):
        for x in range(size):
            if (x,y) not in (start,goal) and random.random() < density:
                grid[y][x] = 1
    return grid

# --- LiDAR Scan ---
def lidar_scan(true_grid, pos, radius=5, noise=False, p_false=0.05, p_miss=0.05):
    size = len(true_grid)
    x0,y0 = pos
    observed = []
    for y in range(max(0,y0-radius), min(size,y0+radius+1)):
        for x in range(max(0,x0-radius), min(size,x0+radius+1)):
            dist = np.sqrt((x-x0)**2+(y-y0)**2)
            if dist <= radius:
                val = true_grid[y][x]
                if noise:
                    if val==0 and random.random() < p_false: val = 1  # 誤検知
                    if val==1 and random.random() < p_miss:  val = 0  # 見逃し
                observed.append((x,y,val))
    return observed

# --- Run One Simulation ---
def simulate(size=20, density=0.25, lidar_radius=5, noise=False):
    start, goal = (0,0), (size-1,size-1)
    true_grid = generate_grid(size, density, start, goal)
    partial_grid = [[-1]*size for _ in range(size)]
    partial_grid[start[1]][start[0]] = 0
    partial_grid[goal[1]][goal[0]] = 0

    pos = start
    steps = 0
    max_steps = size*size*2

    while pos != goal and steps < max_steps:
        # LiDAR観測で部分マップ更新
        for x,y,val in lidar_scan(true_grid,pos,lidar_radius,noise):
            partial_grid[y][x] = val

        # 部分マップから推定グリッドを作成
        est_grid = [[1 if partial_grid[y][x]==1 else 0 for x in range(size)] for y in range(size)]

        path = a_star(est_grid,pos,goal)
        if not path: return False  # 経路断絶

        pos = path[1] if len(path)>1 else pos
        steps += 1

    return pos == goal

# --- Experiment across settings ---
def run_experiments(trials=100):
    radii = [3,5,10]
    settings = [("No Noise",False),("With Noise",True)]
    results = {}

    for label,noise in settings:
        results[label] = []
        for r in radii:
            success = sum(simulate(lidar_radius=r, noise=noise) for _ in range(trials))
            results[label].append(success/trials*100)

    # --- Visualization ---
    x = np.arange(len(radii))
    width = 0.35
    fig, ax = plt.subplots()
    ax.bar(x-width/2, results["No Noise"], width, label="No Noise")
    ax.bar(x+width/2, results["With Noise"], width, label="With Noise")

    ax.set_xticks(x)
    ax.set_xticklabels([f"r={r}" for r in radii])
    ax.set_ylabel("Success Rate (%)")
    ax.set_title("Partial Map Planning: LiDAR Range & Noise")
    ax.legend()
    plt.show()

if __name__ == "__main__":
    run_experiments()