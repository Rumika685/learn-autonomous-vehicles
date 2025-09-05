import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import heapq
import random

GRID = 20
START = (19, 0)
GOAL = (0, 19)

# ========= A* Pathfinding =========
def a_star(grid, start, goal):
    h = lambda a, b: abs(a[0] - b[0]) + abs(a[1] - b[1])
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

        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < GRID and 0 <= ny < GRID and grid[ny][nx] == 0:
                heapq.heappush(open_set, (cost+1+h((nx,ny),goal), cost+1, (nx,ny), path))
    return []

# ========= PID Controller =========
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_err = 0
        self.integral = 0
    def control(self, err, dt=1.0):
        self.integral += err*dt
        diff = (err-self.prev_err)/dt
        self.prev_err = err
        return self.kp*err + self.ki*self.integral + self.kd*diff

# ========= Simulation =========
grid = [[0]*GRID for _ in range(GRID)]
for _ in range(60):
    x,y = random.randint(0,GRID-1), random.randint(0,GRID-1)
    if (x,y) not in (START,GOAL): grid[y][x]=1

path = a_star(grid, START, GOAL)
if not path:
    print("No path found")
    exit()

waypoints = [np.array([x,y],dtype=float) for x,y in path]

# 車の状態
pos = np.array(START,dtype=float)
heading = np.array([0,1],dtype=float)
speed = 1.0
pid = PID(2.0,0.0,0.3)

fig,ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-0.5,GRID-0.5)
ax.set_ylim(-0.5,GRID-0.5)
ax.invert_yaxis()
ax.grid(True)

trail_x, trail_y = [], []

def angle_between(v1,v2):
    v1,v2 = v1/np.linalg.norm(v1), v2/np.linalg.norm(v2)
    dot = np.clip(np.dot(v1,v2),-1.0,1.0)
    return np.arccos(dot)

def update(frame):
    global pos, heading, speed

    ax.clear()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5,GRID-0.5)
    ax.set_ylim(-0.5,GRID-0.5)
    ax.invert_yaxis()
    ax.grid(True)

    # Draw obstacles
    for y in range(GRID):
        for x in range(GRID):
            if grid[y][x]==1:
                ax.text(x,y,"✕",ha="center",va="center",color="red")
    ax.text(*START,"START",color="green",ha="center",va="center")
    ax.text(*GOAL,"GOAL",color="blue",ha="center",va="center")

    if waypoints:
        target = waypoints[0]
        vec = target - pos
        dist = np.linalg.norm(vec)

        # waypoint到達判定を緩める
        if dist < 1.0:
            waypoints.pop(0)
        else:
            desired = vec/dist
            err_angle = np.arctan2(desired[1],desired[0]) - np.arctan2(heading[1],heading[0])
            err_angle = np.arctan2(np.sin(err_angle),np.cos(err_angle))

            w = pid.control(err_angle)
            c,s = np.cos(w), np.sin(w)
            rot = np.array([[c,-s],[s,c]])
            heading = rot@heading
            heading /= np.linalg.norm(heading)

            # 誤差に応じて速度調整
            speed = max(0.2, 1.0 - abs(err_angle))

            # ゴール直前でさらに減速
            if len(waypoints)<5:
                speed = 0.3

            pos += heading*speed*0.5  # 前進成分保証

    # Draw trajectory
    trail_x.append(pos[0])
    trail_y.append(pos[1])
    ax.plot(trail_x,trail_y,"b--")
    ax.plot(pos[0],pos[1],"ro")
    ax.arrow(pos[0],pos[1],heading[0]*0.5,heading[1]*0.5,
             head_width=0.3,head_length=0.3,fc="red",ec="red")

ani = animation.FuncAnimation(fig,update,interval=200)
plt.show()