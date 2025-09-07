import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import heapq
import random

GRID_SIZE = 20
START = (19, 0)
GOAL = (0, 19)

# ---------- A* ----------
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
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = current[0]+dx, current[1]+dy
            if 0<=nx<GRID_SIZE and 0<=ny<GRID_SIZE and grid[ny][nx]==0:
                heapq.heappush(open_set, (cost+1+h((nx,ny),goal), cost+1, (nx,ny), path))
    return []

# ---------- グリッド生成 ----------
def generate_grid():
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    count = 0
    while count < 50:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x,y) not in (START,GOAL) and grid[y][x]==0:
            grid[y][x] = 1
            count += 1
    return grid

# ---------- PID ----------
class PID:
    def __init__(self,kp,ki,kd):
        self.kp,self.ki,self.kd = kp,ki,kd
        self.integral = 0.0
        self.prev_error = 0.0
    def control(self,error,dt=1.0):
        self.integral += error*dt
        derivative = (error-self.prev_error)/dt
        self.prev_error = error
        return self.kp*error + self.ki*self.integral + self.kd*derivative

# ---------- メイン ----------
def main():
    grid = generate_grid()
    path = a_star(grid, START, GOAL)
    if not path:
        print("No path found")
        return

    pos = np.array(START, dtype=float)
    heading = np.array([0.0,1.0])   # 上向き
    pid = PID(2.0,0.0,0.3)
    history = [tuple(pos)]
    lookahead = 3   # ✅ Lookahead導入

    fig, ax = plt.subplots()

    def update(_):
        nonlocal pos, heading, path

        ax.clear()
        ax.set_xlim(-0.5,GRID_SIZE-0.5)
        ax.set_ylim(-0.5,GRID_SIZE-0.5)
        ax.set_aspect("equal")
        ax.grid(True)
        ax.invert_yaxis()

        # 障害物
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                if grid[y][x]==1:
                    ax.text(x,y,"✕",color="red",ha="center",va="center")

        # START / GOAL
        ax.text(START[0],START[1],"START",color="green",ha="center",va="center")
        ax.text(GOAL[0],GOAL[1],"GOAL",color="blue",ha="center",va="center")

        # ゴール到達判定
        if np.linalg.norm(pos-np.array(GOAL))<0.5:
            ax.plot(*zip(*history),"b--")
            ax.plot(pos[0],pos[1],"ro")
            ax.set_title("Goal reached!")
            return []

        # ✅ 現在位置に最も近いpath点を探す
        dists = [np.linalg.norm(pos-np.array(p)) for p in path]
        nearest_idx = int(np.argmin(dists))
        target_idx = min(nearest_idx+lookahead, len(path)-1)
        target = path[target_idx]

        vec = np.array([target[0]-pos[0], target[1]-pos[1]])
        desired = vec / np.linalg.norm(vec)

        # 角度誤差
        err_angle = np.arctan2(desired[1],desired[0]) - np.arctan2(heading[1],heading[0])
        err_angle = np.arctan2(np.sin(err_angle), np.cos(err_angle))
        w = pid.control(err_angle)

        # heading回転
        c,s = np.cos(w), np.sin(w)
        rot = np.array([[c,-s],[s,c]])
        heading = rot @ heading
        heading /= np.linalg.norm(heading)

        # ✅ ゴール方向補正
        goal_vec = np.array([GOAL[0]-pos[0], GOAL[1]-pos[1]])
        goal_vec /= np.linalg.norm(goal_vec)
        forward_factor = max(0.5, np.dot(heading, goal_vec))

        # 速度
        speed = 1.0 * forward_factor

        # 移動
        pos[:] = pos + heading * speed
        history.append(tuple(pos))

        # 描画
        ax.plot(*zip(*path),"k--",alpha=0.3)   # 参照path
        ax.plot(*zip(*history),"b-")           # 実際の走行
        ax.plot(pos[0],pos[1],"ro")

        return []

    ani = animation.FuncAnimation(fig,update,interval=200)
    plt.show()

if __name__=="__main__":
    main()