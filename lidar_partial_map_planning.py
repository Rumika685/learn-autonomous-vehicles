import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
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
def generate_grid(size=20, density=0.2, start=(0,0), goal=None):
    if goal is None:
        goal = (size-1, size-1)
    grid = [[0]*size for _ in range(size)]
    for y in range(size):
        for x in range(size):
            if (x,y) not in (start,goal) and random.random() < density:
                grid[y][x] = 1
    return grid

# --- LiDAR Scan (detect within radius) ---
def lidar_scan(true_grid, pos, radius=5):
    size = len(true_grid)
    x0, y0 = pos
    observed = []
    for y in range(max(0,y0-radius), min(size,y0+radius+1)):
        for x in range(max(0,x0-radius), min(size,x0+radius+1)):
            dist = np.sqrt((x-x0)**2 + (y-y0)**2)
            if dist <= radius:
                observed.append((x,y,true_grid[y][x]))
    return observed

# --- Main Simulation ---
def simulate(size=20, density=0.2, lidar_radius=5):
    start, goal = (0,0), (size-1,size-1)
    true_grid = generate_grid(size, density, start, goal)

    # 初期状態：未知マップ（-1 = 未知）
    partial_grid = [[-1]*size for _ in range(size)]
    for y in range(size):
        for x in range(size):
            if (x,y) in (start,goal):
                partial_grid[y][x] = 0

    pos = start
    path_history = [pos]

    # 可視化準備
    fig, ax = plt.subplots()
    ax.set_aspect("equal")

    def update(_):
        nonlocal pos, partial_grid

        # LiDARで観測 → 部分マップ更新
        observations = lidar_scan(true_grid, pos, lidar_radius)
        for x,y,val in observations:
            partial_grid[y][x] = val

        # 経路計画（部分マップベース）
        est_grid = [[1 if partial_grid[y][x]==1 else 0 for x in range(size)] for y in range(size)]
        path = a_star(est_grid, pos, goal)

        ax.clear()
        ax.set_xlim(-0.5,size-0.5)
        ax.set_ylim(-0.5,size-0.5)
        ax.invert_yaxis()
        ax.grid(True)

        # 真の障害物（灰色で背景表示）
        for y in range(size):
            for x in range(size):
                if true_grid[y][x] == 1:
                    ax.add_patch(plt.Rectangle((x-0.5,y-0.5),1,1,color="lightgrey"))

        # 部分マップの障害物（赤✕）
        for y in range(size):
            for x in range(size):
                if partial_grid[y][x] == 1:
                    ax.text(x,y,"✕",ha="center",va="center",color="red")

        # ゴール
        gx,gy = goal
        ax.text(gx,gy,"GOAL",color="blue",ha="center",va="center")

        # 経路描画
        if path:
            xs,ys = zip(*path)
            ax.plot(xs,ys,"b--")
            pos = path[1] if len(path)>1 else pos
        else:
            ax.set_title("❌ No Path (so far)")
            return

        # 車
        ax.plot(pos[0],pos[1],"ro")
        path_history.append(pos)

        if pos == goal:
            ax.set_title("✅ Goal Reached")

    ani = animation.FuncAnimation(fig, update, interval=500)
    plt.show()

# 実行
if __name__ == "__main__":
    simulate(size=20, density=0.25, lidar_radius=5)