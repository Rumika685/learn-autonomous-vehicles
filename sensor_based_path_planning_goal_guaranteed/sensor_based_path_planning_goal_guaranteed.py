# sensor_based_path_planning.py
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
import random
from collections import deque

# ---------------------------
# Config
# ---------------------------
GRID_SIZE = 10
START = (9, 0)   # right-bottom in our coordinate convention (x,y)
GOAL = (0, 9)    # left-top
INITIAL_OBSTACLES = 20
DYNAMIC_OBSTACLE_INTERVAL = 4  # フレームごとではなく、移動回数ごとの間隔
SEED = 1  # 再現用（None にするとランダム）
# ---------------------------

random.seed(SEED)

# ---------- utilities ----------
def in_bounds(x, y):
    return 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE

def neighbors(cell):
    x, y = cell
    for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
        nx, ny = x+dx, y+dy
        if in_bounds(nx, ny):
            yield (nx, ny)

# ---------- connectivity check (BFS) ----------
def is_connected(grid, start, goal):
    if grid[start[1]][start[0]] == 1 or grid[goal[1]][goal[0]] == 1:
        return False
    seen = set([start])
    q = deque([start])
    while q:
        cur = q.popleft()
        if cur == goal:
            return True
        for nb in neighbors(cur):
            if nb not in seen and grid[nb[1]][nb[0]] == 0:
                seen.add(nb)
                q.append(nb)
    return False

# ---------- A* ----------
def a_star(grid, start, goal):
    # Manhattan heuristic
    h = lambda a,b: abs(a[0]-b[0]) + abs(a[1]-b[1])
    openq = [(h(start,goal), 0, start, [])]
    closed = set()
    while openq:
        _, cost, cur, path = heapq.heappop(openq)
        if cur in closed:
            continue
        closed.add(cur)
        path2 = path + [cur]
        if cur == goal:
            return path2
        for nb in neighbors(cur):
            if grid[nb[1]][nb[0]] == 0 and nb not in closed:
                heapq.heappush(openq, (cost+1 + h(nb,goal), cost+1, nb, path2))
    return []

# ---------- generate initial grid while preserving connectivity ----------
def generate_grid():
    grid = [[0]*GRID_SIZE for _ in range(GRID_SIZE)]
    placed = 0
    attempts = 0
    while placed < INITIAL_OBSTACLES and attempts < INITIAL_OBSTACLES*20:
        attempts += 1
        x = random.randrange(GRID_SIZE)
        y = random.randrange(GRID_SIZE)
        if (x,y) in (START, GOAL) or grid[y][x] == 1:
            continue
        # tentatively place and test connectivity
        grid[y][x] = 1
        if is_connected(grid, START, GOAL):
            placed += 1
        else:
            grid[y][x] = 0  # rollback
    return grid

# ---------- sensor model ----------
# Our car has three binary sensors relative to facing: (left, front, right)
# We will model facing as one of four cardinal directions (0=east,1=south,2=west,3=north)
# Utility: transform relative sensor direction to absolute grid cell
DIRS = ['east','south','west','north']
VECT = {'east':(1,0),'south':(0,1),'west':(-1,0),'north':(0,-1)}

def sensor_cells(pos, facing_idx):
    # returns list of (left_cell, front_cell, right_cell) (cells may be out of bounds)
    fx = DIRS[facing_idx]
    # left direction = facing_idx -1
    left_dir = DIRS[(facing_idx-1)%4]
    right_dir = DIRS[(facing_idx+1)%4]
    left = (pos[0] + VECT[left_dir][0], pos[1] + VECT[left_dir][1])
    front = (pos[0] + VECT[fx][0], pos[1] + VECT[fx][1])
    right = (pos[0] + VECT[right_dir][0], pos[1] + VECT[right_dir][1])
    return left, front, right

def read_sensors(grid, pos, facing_idx):
    cells = sensor_cells(pos, facing_idx)
    results = []
    for c in cells:
        x,y = c
        if not in_bounds(x,y):
            results.append(True)  # out-of-bounds treated as obstacle (wall)
        else:
            results.append(grid[y][x] == 1)
    return tuple(results)  # (left, front, right)

# ---------- dynamic obstacle adder but preserves connectivity ----------
def add_dynamic_obstacle_preserve_connectivity(grid):
    # try a few times to find a random cell that is free and doesn't disconnect Start-Goal
    tries = 0
    while tries < 50:
        tries += 1
        x = random.randrange(GRID_SIZE)
        y = random.randrange(GRID_SIZE)
        if (x,y) in (START, GOAL):
            continue
        if grid[y][x] == 1:
            continue
        # tentatively place
        grid[y][x] = 1
        if is_connected(grid, current_pos, GOAL):  # ensure connectivity from current pos too
            return (x,y)
        # rollback
        grid[y][x] = 0
    return None

# ---------- main simulation (with Matplotlib animation) ----------
grid = generate_grid()
# Keep a separate overlay of detected obstacles (so we can mark where sensors actually saw them)
detected_overlay = [[False]*GRID_SIZE for _ in range(GRID_SIZE)]

# initial state
facing_idx = 3  # north (user requested initial north)
current_pos = START
history = [current_pos]
facing_history = [facing_idx]
path = a_star(grid, current_pos, GOAL)

# animation drawing function
fig, ax = plt.subplots(figsize=(6,6))
ax.set_aspect('equal')
ax.set_xlim(-0.5, GRID_SIZE-0.5)
ax.set_ylim(-0.5, GRID_SIZE-0.5)
ax.invert_yaxis()
ax.grid(True)

step_counter = 0
dynamic_counter = 0

def draw_static(ax, g, overlay, hist, planned_path):
    ax.clear()
    ax.set_aspect('equal')
    ax.set_xlim(-0.5, GRID_SIZE-0.5)
    ax.set_ylim(-0.5, GRID_SIZE-0.5)
    ax.invert_yaxis()
    ax.grid(True)
    # obstacles
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if g[y][x] == 1:
                ax.text(x, y, "✕", ha='center', va='center', color='red', fontsize=14)
            elif overlay[y][x]:
                # sensor-detected (but not yet marked as obstacle in grid) — show smaller x
                ax.text(x, y, "×", ha='center', va='center', color='orange', fontsize=10)
    # start/goal
    ax.text(START[0], START[1], "START", ha='center', va='center', color='green', fontsize=10)
    ax.text(GOAL[0], GOAL[1], "GOAL", ha='center', va='center', color='blue', fontsize=10)
    # history
    if len(hist) >= 2:
        hx, hy = zip(*hist)
        ax.plot(hx, hy, 'b-')  # keep solid trail
    # planned path (dashed)
    if planned_path:
        px, py = zip(*planned_path)
        ax.plot(px, py, 'b--', alpha=0.6)
    # current position marker
    cur = hist[-1]
    ax.plot(cur[0], cur[1], 'ro')

# update function called by animation
def update(frame):
    global current_pos, path, facing_idx, step_counter, dynamic_counter
    step_counter += 1

    # 1) Replan if no path or path doesn't start at current pos
    if not path or path[0] != current_pos:
        path = a_star(grid, current_pos, GOAL)

    # if no path -> try backtrack
    if not path:
        # backtrack history if possible
        if len(history) > 1:
            history.pop()  # drop current
            current_pos = history[-1]
            print("Backtracking to", current_pos)
            path = a_star(grid, current_pos, GOAL)
            facing_idx = facing_history[-1] if facing_history else 3
        else:
            print("No path available and cannot backtrack. Giving up.")
            ani.event_source.stop()
            return

    # if next step exists, compute desired movement
    if len(path) >= 2:
        next_pos = path[1]
        # derive desired facing to move into next_pos
        dx = next_pos[0] - current_pos[0]
        dy = next_pos[1] - current_pos[1]
        # choose facing_idx matching dx,dy
        for i, dname in enumerate(DIRS):
            vx, vy = VECT[dname]
            if (vx, vy) == (dx, dy):
                desired_facing = i
                break
        else:
            desired_facing = facing_idx

        # rotate toward desired_facing by at most 1 step (left or right)
        if desired_facing != facing_idx:
            # choose smallest rotation direction
            diff = (desired_facing - facing_idx) % 4
            if diff == 1 or diff == 3:
                # one rotation (either way) — pick direction that results in forward possibility if possible
                if diff == 1:
                    facing_idx = (facing_idx + 1) % 4
                else:
                    facing_idx = (facing_idx - 1) % 4

        # sense obstacles around (left, front, right) relative to new facing
        left_s, front_s, right_s = read_sensors(grid, current_pos, facing_idx)
        print(f"Sensors at {current_pos} facing {DIRS[facing_idx]} -> L:{left_s} F:{front_s} R:{right_s}")

        # mark detected cells in overlay (for visualization)
        left_cell, front_cell, right_cell = sensor_cells(current_pos, facing_idx)
        for c, sensed in zip((left_cell, front_cell, right_cell), (left_s, front_s, right_s)):
            x,y = c
            if in_bounds(x,y) and sensed:
                detected_overlay[y][x] = True
                # update map (we choose to immediately mark them as obstacles to enforce replan)
                if grid[y][x] == 0:
                    # But ensure marking doesn't disconnect Start-Goal
                    grid[y][x] = 1
                    if not is_connected(grid, current_pos, GOAL):
                        grid[y][x] = 0  # rollback if disconnects
                    else:
                        print("Sensor-detected obstacle recorded at", c)

        # if front is free, move forward
        if not front_s:
            # perform move
            current_pos = (current_pos[0] + VECT[DIRS[facing_idx]][0],
                           current_pos[1] + VECT[DIRS[facing_idx]][1])
            history.append(current_pos)
            facing_history.append(facing_idx)
        else:
            # blocked ahead -> try turning preference: right, left, then backtrack
            turned = False
            # try right
            r_idx = (facing_idx + 1) % 4
            rx, ry = sensor_cells(current_pos, r_idx)[1] if False else (current_pos[0] + VECT[DIRS[r_idx]][0], current_pos[1] + VECT[DIRS[r_idx]][1])
            if in_bounds(rx,ry) and grid[ry][rx] == 0:
                facing_idx = r_idx
                turned = True
            else:
                l_idx = (facing_idx - 1) % 4
                lx, ly = current_pos[0] + VECT[DIRS[l_idx]][0], current_pos[1] + VECT[DIRS[l_idx]][1]
                if in_bounds(lx,ly) and grid[ly][lx] == 0:
                    facing_idx = l_idx
                    turned = True

            if not turned:
                # cannot turn, so backtrack one step if possible
                if len(history) > 1:
                    history.pop()
                    current_pos = history[-1]
                    facing_history.pop()
                    facing_idx = facing_history[-1] if facing_history else 3
                    print("Blocked ahead and sides; backtracking to", current_pos)
                else:
                    print("Surrounded at start! Stopping.")
                    ani.event_source.stop()
                    return

    # dynamic obstacle add occasionally (but only if it won't disconnect)
    dynamic_counter += 1
    if dynamic_counter % DYNAMIC_OBSTACLE_INTERVAL == 0:
        # try to add a dynamic obstacle that does not disconnect start-goal
        tries = 0
        added = False
        while tries < 30 and not added:
            tries += 1
            rx = random.randrange(GRID_SIZE)
            ry = random.randrange(GRID_SIZE)
            if (rx,ry) in (START, GOAL) or grid[ry][rx] == 1 or (rx,ry) == current_pos:
                continue
            grid[ry][rx] = 1
            if is_connected(grid, current_pos, GOAL):
                print("Dynamic obstacle added at", (rx,ry))
                added = True
            else:
                grid[ry][rx] = 0

    # if reached goal
    if current_pos == GOAL:
        ax.set_title("Goal reached!")
        draw_static(ax, grid, detected_overlay, history, [])
        ani.event_source.stop()
        return

    # replan for next frame (so path displayed is current)
    path = a_star(grid, current_pos, GOAL)
    draw_static(ax, grid, detected_overlay, history, path)

# run animation
ani = animation.FuncAnimation(fig, update, interval=500)
plt.show()
