import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

GRID = 20
START, GOAL = (0, GRID-1), (GRID-1, 0)

def a_star(grid, start, goal):
    from heapq import heappush, heappop
    h = lambda a,b: abs(a[0]-b[0]) + abs(a[1]-b[1])
    open_set=[(h(start,goal),0,start,[])]
    visited=set()
    while open_set:
        _,cost,cur,path=heappop(open_set)
        if cur in visited: continue
        visited.add(cur)
        path=path+[cur]
        if cur==goal: return path
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx,ny=cur[0]+dx,cur[1]+dy
            if 0<=nx<GRID and 0<=ny<GRID and grid[ny][nx]==0:
                heappush(open_set,(cost+1+h((nx,ny),goal),cost+1,(nx,ny),path))
    return []

# ---- PID クラス ----
class PID:
    def __init__(self,kp,ki,kd):
        self.kp,self.ki,self.kd=kp,ki,kd
        self.prev_error=0
        self.integral=0
    def control(self,error,dt=1.0):
        self.integral+=error*dt
        derivative=(error-self.prev_error)/dt
        self.prev_error=error
        return self.kp*error+self.ki*self.integral+self.kd*derivative

# ---- 経路追従シミュレーション ----
def simulate_path(path, grid):
    fig, ax = plt.subplots()
    ax.set_xlim(-0.5, GRID-0.5)
    ax.set_ylim(-0.5, GRID-0.5)
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.grid(True)

    # 障害物を描画 🚧
    for y in range(GRID):
        for x in range(GRID):
            if grid[y][x] == 1:
                ax.text(x,y,"✕",color="red",ha="center",va="center")

    # スタート & ゴール
    ax.text(*START,"START",color="green",ha="center",va="center")
    ax.text(*GOAL,"GOAL",color="blue",ha="center",va="center")

    xs, ys = zip(*path)
    ax.plot(xs, ys, "b--", label="A* path")

    point, = ax.plot([], [], "ro", label="Car")
    head = np.array([float(xs[0]), float(ys[0])])

    pid = PID(0.5,0.0,0.1)
    idx=[0]

    def update(_):
        nonlocal head
        if idx[0] >= len(path): return point,
        target = np.array(path[idx[0]])
        error = np.linalg.norm(target - head)
        if error < 0.1:
            idx[0]+=1
        else:
            direction=(target-head)
            if np.linalg.norm(direction)>0:
                direction/=np.linalg.norm(direction)
            adjust=pid.control(error)
            head+=direction*0.2+adjust*0.01
        point.set_data(head[0],head[1])
        return point,

    ani = animation.FuncAnimation(fig, update, interval=100)
    plt.legend()
    plt.show()

# ===== Main =====
if __name__ == "__main__":
    while True:
        grid = [[0]*GRID for _ in range(GRID)]
        for _ in range(40):
            x,y=np.random.randint(0,GRID),np.random.randint(0,GRID)
            if (x,y) not in (START,GOAL):
                grid[y][x]=1
        path=a_star(grid,START,GOAL)
        if path: break

    simulate_path(path, grid)
