# File: pid_physics_model.py
# PID + physics (velocity/acceleration) model demo

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq, random

# --------------------
# A* Pathfinding
# --------------------
GRID = 20
START = (0, 0)
GOAL = (19, 19)
NUM_OBS = 40  # 障害物数（調整済み）

def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def a_star(grid, start, goal):
    OPEN = [(heuristic(start,goal), 0, start, [start])]
    visited = set()
    while OPEN:
        _, cost, cur, path = heapq.heappop(OPEN)
        if cur in visited:
            continue
        visited.add(cur)
        if cur == goal:
            return path
        x, y = cur
        for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < GRID and 0 <= ny < GRID and grid[ny][nx]==0:
                heapq.heappush(OPEN, (cost+1+heuristic((nx,ny),goal), cost+1, (nx,ny), path+[(nx,ny)]))
    return []

def generate_grid():
    grid = [[0]*GRID for _ in range(GRID)]
    count = 0
    while count < NUM_OBS:
        x = random.randint(0, GRID-1)
        y = random.randint(0, GRID-1)
        if (x,y) in (START, GOAL): continue
        if grid[y][x]==0:
            grid[y][x]=1
            count += 1
    return grid

# --------------------
# PID Controller
# --------------------
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0
        self.prev_err = 0
    def control(self, err, dt):
        self.integral += err*dt
        deriv = (err-self.prev_err)/dt if dt>0 else 0
        self.prev_err = err
        return self.kp*err + self.ki*self.integral + self.kd*deriv

# --------------------
# Car with Physics
# --------------------
class Car:
    def __init__(self, pos=(0,0), heading=0.0):
        self.pos = np.array(pos, dtype=float)
        self.heading = heading
        self.speed = 0.0
        self.acc = 0.0
        self.steer_pid = PID(2.0, 0.0, 0.5)
        self.speed_pid = PID(1.0, 0.0, 0.2)
    def update(self, target, dt=0.1):
        # 目標方向
        vec = np.array(target) - self.pos
        dist = np.linalg.norm(vec)
        desired_heading = np.arctan2(vec[1], vec[0])
        err_heading = ((desired_heading - self.heading + np.pi) % (2*np.pi)) - np.pi
        steer = self.steer_pid.control(err_heading, dt)
        self.heading += steer*dt
        # 速度制御（ゴール付近では減速）
        desired_speed = min(2.0, dist)
        err_speed = desired_speed - self.speed
        acc_cmd = self.speed_pid.control(err_speed, dt)
        self.acc = acc_cmd
        self.speed += self.acc*dt
        self.speed = max(0.0, min(self.speed, 3.0))  # clamp
        # 移動
        self.pos += np.array([np.cos(self.heading), np.sin(self.heading)]) * self.speed * dt

# --------------------
# Simulation
# --------------------
def run():
    grid = generate_grid()
    path = a_star(grid, START, GOAL)
    if not path:
        print("No path found!")
        return
    waypoints = [np.array(p, dtype=float) for p in path]

    car = Car(pos=START, heading=0.0)
    history = [car.pos.copy()]
    idx = [0]

    fig, ax = plt.subplots(figsize=(6,6))
    ax.set_xlim(-1, GRID)
    ax.set_ylim(-1, GRID)
    ax.set_aspect("equal")
    ax.grid(True)

    # Draw obstacles
    for y in range(GRID):
        for x in range(GRID):
            if grid[y][x]==1:
                ax.text(x, y, "✕", color="red", ha="center", va="center")

    # Draw path
    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]
    ax.plot(xs, ys, "c--", label="A* path")

    trail, = ax.plot([], [], "b-", linewidth=2)
    point, = ax.plot([], [], "ro", markersize=6)
    ax.text(*START, "START", color="green", ha="center", va="center")
    ax.text(*GOAL, "GOAL", color="blue", ha="center", va="center")

    def update(frame):
        if idx[0] >= len(waypoints):
            return trail, point
        target = waypoints[idx[0]]
        car.update(target)
        if np.linalg.norm(car.pos-target) < 0.5 and idx[0] < len(waypoints)-1:
            idx[0] += 1
        history.append(car.pos.copy())
        xs = [p[0] for p in history]
        ys = [p[1] for p in history]
        trail.set_data(xs, ys)
        # --- 修正済み ---
        point.set_data([car.pos[0]], [car.pos[1]])
        ax.set_title(f"Speed={car.speed:.2f}, Acc={car.acc:.2f}")
        return trail, point

    ani = animation.FuncAnimation(fig, update, frames=500, interval=100, blit=True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    run()
