import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random, heapq

GRID = 20
START, GOAL = (0, GRID-1), (GRID-1, 0)

# ==== A* ====
def a_star(grid, start, goal):
    h=lambda a,b: abs(a[0]-b[0])+abs(a[1]-b[1])
    open=[(h(start,goal),0,start,[])]
    visited=set()
    while open:
        _,cost,cur,path=heapq.heappop(open)
        if cur in visited: continue
        visited.add(cur)
        path=path+[cur]
        if cur==goal: return path
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx,ny=cur[0]+dx,cur[1]+dy
            if 0<=nx<GRID and 0<=ny<GRID and grid[ny][nx]==0:
                heapq.heappush(open,(cost+1+h((nx,ny),goal),cost+1,(nx,ny),path))
    return []

# ==== Waypoint 抽出 ====
def extract_waypoints(path):
    if not path: return []
    waypoints=[path[0]]
    for i in range(1,len(path)-1):
        prev=np.array(path[i-1])
        curr=np.array(path[i])
        nxt=np.array(path[i+1])
        if not np.array_equal((curr-prev),(nxt-curr)):
            waypoints.append(tuple(curr))
    waypoints.append(path[-1])
    return waypoints

# ==== PID ====
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

# ==== シミュレーション ====
def simulate(grid,path,waypoints):
    fig,ax=plt.subplots()
    ax.set_xlim(-0.5,GRID-0.5)
    ax.set_ylim(-0.5,GRID-0.5)
    ax.set_aspect("equal")
    ax.invert_yaxis()
    ax.grid(True)

    # 障害物
    for y in range(GRID):
        for x in range(GRID):
            if grid[y][x]==1:
                ax.text(x,y,"✕",color="red",ha="center",va="center")

    # START / GOAL
    ax.text(*START,"START",color="green",ha="center",va="center")
    ax.text(*GOAL,"GOAL",color="blue",ha="center",va="center")

    # 経路（青）
    xs,ys=zip(*path)
    ax.plot(xs,ys,"b--",label="A* path")

    # Waypoints（オレンジ）
    wx,wy=zip(*waypoints)
    ax.plot(wx,wy,"o-",color="orange",label="Waypoints")

    point,=ax.plot([],[],"ro",label="Car")

    head=np.array([float(xs[0]),float(ys[0])])
    pid=PID(0.5,0.0,0.1)
    idx=[0]

    def update(_):
        nonlocal head
        if idx[0]>=len(waypoints): return point,
        target=np.array(waypoints[idx[0]])
        error=np.linalg.norm(target-head)
        if error<0.1:
            idx[0]+=1
        else:
            direction=(target-head)
            if np.linalg.norm(direction)>0:
                direction/=np.linalg.norm(direction)
            adjust=pid.control(error)
            head+=direction*0.2+adjust*0.01
        point.set_data(head[0],head[1])
        return point,

    ani=animation.FuncAnimation(fig,update,interval=100)
    plt.legend()
    plt.show()

# ==== Main ====
if __name__=="__main__":
    while True:
        grid=[[0]*GRID for _ in range(GRID)]
        for _ in range(40):
            x,y=random.randint(0,GRID-1),random.randint(0,GRID-1)
            if (x,y) not in (START,GOAL):
                grid[y][x]=1
        path=a_star(grid,START,GOAL)
        if path: break

    waypoints=extract_waypoints(path)
    print("Waypoints:",waypoints)
    simulate(grid,path,waypoints)