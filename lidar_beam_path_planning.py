# lidar_beam_path_planning.py
import random, heapq
import numpy as np
import matplotlib.pyplot as plt

SIZE = 20
START = (0, 0)
GOAL = (SIZE-1, SIZE-1)
OBSTACLE_DENSITY = 0.25

# ====== マップ生成 ======
def generate_grid():
    g = [[0]*SIZE for _ in range(SIZE)]
    for y in range(SIZE):
        for x in range(SIZE):
            if (x,y) not in (START,GOAL) and random.random() < OBSTACLE_DENSITY:
                g[y][x] = 1
    return g

# ====== A* ======
def a_star(grid, start, goal):
    h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])
    open_set = [(h(start, goal), 0, start, [])]
    visited = set()
    while open_set:
        _, cost, current, path = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        path = path + [current]
        if current == goal:
            return path
        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            nx, ny = current[0]+dx, current[1]+dy
            if 0<=nx<SIZE and 0<=ny<SIZE and grid[ny][nx]==0:
                heapq.heappush(open_set,(cost+1+h((nx,ny),goal), cost+1, (nx,ny), path))
    return []

# ====== Bresenham (ビーム用) ======
def bresenham_line(x0, y0, x1, y1):
    points=[]
    dx, dy = abs(x1-x0), abs(y1-y0)
    sx, sy = (1 if x0<x1 else -1), (1 if y0<y1 else -1)
    err = dx-dy
    while True:
        points.append((x0,y0))
        if x0==x1 and y0==y1: break
        e2=2*err
        if e2>-dy: err-=dy; x0+=sx
        if e2<dx: err+=dx; y0+=sy
    return points

# ====== LiDAR風ビーム ======
def lidar_beam_scan(true_grid, pos, angles, radius=10):
    detected=[]
    x0,y0=pos
    for theta in angles:
        x1=int(round(x0+radius*np.cos(theta)))
        y1=int(round(y0+radius*np.sin(theta)))
        line=bresenham_line(x0,y0,x1,y1)
        for (x,y) in line:
            if 0<=x<SIZE and 0<=y<SIZE:
                if true_grid[y][x]==1:
                    detected.append((x,y))
                    break
    return detected

# ====== メイン処理 ======
def main():
    true_grid=generate_grid()
    # 車が知っているマップ（初期は空）
    known_grid=[[0]*SIZE for _ in range(SIZE)]
    pos=START
    angles=np.linspace(0,2*np.pi,36,endpoint=False)

    path=[]
    history=[pos]

    for step in range(200):  # 最大ステップ数
        # センサー観測 → known_mapに追加
        detections=lidar_beam_scan(true_grid,pos,angles,radius=6)
        for (x,y) in detections:
            known_grid[y][x]=1

        # A*で再計画
        path=a_star(known_grid,pos,GOAL)
        if not path:
            print("No path found")
            break

        # 1ステップ進む
        if pos==GOAL:
            print("Goal reached!")
            break
        if len(path)>1:
            pos=path[1]
            history.append(pos)

    # ====== 可視化 ======
    fig,ax=plt.subplots(figsize=(6,6))
    ax.set_xlim(-0.5,SIZE-0.5)
    ax.set_ylim(-0.5,SIZE-0.5)
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.grid(True)

    # 真の障害物（灰色）
    for y in range(SIZE):
        for x in range(SIZE):
            if true_grid[y][x]==1:
                ax.add_patch(plt.Rectangle((x-0.5,y-0.5),1,1,color="lightgray"))

    # 車が検知した障害物（黒）
    for y in range(SIZE):
        for x in range(SIZE):
            if known_grid[y][x]==1:
                ax.add_patch(plt.Rectangle((x-0.5,y-0.5),1,1,color="black"))

    # 軌跡
    xs,ys=zip(*history)
    ax.plot(xs,ys,"b.-",label="Path")

    ax.plot(START[0],START[1],"go",markersize=10,label="Start")
    ax.plot(GOAL[0],GOAL[1],"ro",markersize=10,label="Goal")

    ax.set_title("Path Planning with LiDAR Beam Observations")
    ax.legend()
    plt.show()

if __name__=="__main__":
    main()