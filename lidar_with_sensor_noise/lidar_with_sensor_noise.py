import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random, heapq

GRID_SIZE = 10
START = (9, 0)
GOAL = (0, 9)

# --- Car クラス ---
class Car:
    def __init__(self, start, heading=90, max_range=3, fov=120, num_beams=9,
                 false_negative=0.1, false_positive=0.05):
        self.pos = np.array(start)
        self.heading = heading
        self.max_range = max_range
        self.fov = fov
        self.num_beams = num_beams
        self.known_grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        self.false_negative = false_negative
        self.false_positive = false_positive

    def detect_obstacles(self, true_grid):
        detected = []
        half_fov = self.fov / 2
        angles = np.linspace(-half_fov, half_fov, self.num_beams)

        for angle in angles:
            beam_angle = np.deg2rad(self.heading + angle)
            dx, dy = np.cos(beam_angle), -np.sin(beam_angle)

            for r in range(1, self.max_range + 1):
                x = int(round(self.pos[0] + dx * r))
                y = int(round(self.pos[1] + dy * r))

                if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                    # 真の障害物にヒット
                    if true_grid[y][x] == 1:
                        if random.random() > self.false_negative:  # 見逃し確率
                            self.known_grid[y][x] = 1
                            detected.append((x, y))
                        break  # このビームはここで終了
                    else:
                        # 誤検知の可能性
                        if random.random() < self.false_positive:
                            self.known_grid[y][x] = 1
                            detected.append((x, y))
                            break
        return detected

# --- A* ---
def a_star(grid, start, goal):
    h = lambda a, b: abs(a[0] - b[0]) + abs(a[1] - b[1])
    open_set = [(h(start, goal), 0, start, [])]
    visited = set()

    while open_set:
        _, cost, current, path = heapq.heappop(open_set)
        if current in visited: continue
        visited.add(current)
        path = path + [current]
        if current == goal: return path

        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = current[0]+dx, current[1]+dy
            if 0<=nx<GRID_SIZE and 0<=ny<GRID_SIZE and grid[ny][nx]==0:
                heapq.heappush(open_set, (cost+1+h((nx,ny),goal), cost+1,(nx,ny),path))
    return []

# --- グリッド生成 ---
def generate_grid(num_obstacles=20):
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    count=0
    while count < num_obstacles:
        x,y = random.randint(0,GRID_SIZE-1),random.randint(0,GRID_SIZE-1)
        if (x,y) not in (START,GOAL) and grid[y][x]==0:
            grid[y][x]=1
            count+=1
    return grid

# --- シミュレーション ---
true_grid = generate_grid()
car = Car(START, false_negative=0.1, false_positive=0.05)
path = a_star(true_grid, tuple(car.pos), GOAL)
history = []

fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-0.5, GRID_SIZE-0.5)
ax.set_ylim(-0.5, GRID_SIZE-0.5)
ax.invert_yaxis()
ax.grid(True)

def update(_):
    global path
    ax.clear()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, GRID_SIZE-0.5)
    ax.set_ylim(-0.5, GRID_SIZE-0.5)
    ax.invert_yaxis()
    ax.grid(True)

    # 真の障害物
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if true_grid[y][x] == 1:
                ax.text(x,y,"✕",color="red",ha="center",va="center")

    # センサーで検知
    detected = car.detect_obstacles(true_grid)

    # 検知した障害物
    for (x,y) in detected:
        ax.text(x,y,"✕",color="blue",ha="center",va="center")

    # LiDARビーム可視化
    half_fov = car.fov / 2
    angles = np.linspace(-half_fov, half_fov, car.num_beams)
    for angle in angles:
        beam_angle = np.deg2rad(car.heading + angle)
        dx, dy = np.cos(beam_angle), -np.sin(beam_angle)
        bx = car.pos[0] + dx * car.max_range
        by = car.pos[1] + dy * car.max_range
        ax.plot([car.pos[0], bx], [car.pos[1], by], "gray", alpha=0.3)

    # 経路探索（センサーで得た known_grid を利用）
    path = a_star(car.known_grid, tuple(car.pos), GOAL)
    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, "g-")

    # 車の位置
    ax.plot(car.pos[0], car.pos[1], "ro")

    # START & GOAL
    ax.text(*START, "START", color="green", ha="center", va="center")
    ax.text(*GOAL, "GOAL", color="blue", ha="center", va="center")

    # ゴール判定
    if tuple(car.pos) == GOAL:
        ax.set_title("✅ Goal Reached!")
        return

    if len(path) > 1:
        car.pos = np.array(path[1])
        history.append(tuple(car.pos))

ani = animation.FuncAnimation(fig, update, interval=800)
plt.show()
