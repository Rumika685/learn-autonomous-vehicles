# occupancy_param_sweep.py
import random
import numpy as np
import matplotlib.pyplot as plt
import heapq

SIZE = 20
START = (0, 0)
GOAL = (SIZE - 1, SIZE - 1)
OBSTACLE_DENSITY = 0.25
LIDAR_RADIUS = 5
P_FALSE, P_MISS = 0.05, 0.05
L_OCC, L_FREE = 2.2, 2.2
L_MIN, L_MAX = -8.0, 8.0
TRIALS = 30
MAX_STEPS = SIZE * SIZE * 2

def a_star_with_cost(cost_grid, start, goal, blocked_mask):
    size = len(cost_grid)
    h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])
    open_set = [(h(start, goal), 0, start, [])]
    best = {start: 0}
    while open_set:
        _, g, cur, path = heapq.heappop(open_set)
        if g != best.get(cur, float('inf')):
            continue
        path = path + [cur]
        if cur == goal:
            return path
        x, y = cur
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < size and 0 <= ny < size and not blocked_mask[ny][nx]:
                ng = g + cost_grid[ny][nx]
                if ng < best.get((nx,ny), float('inf')):
                    best[(nx,ny)] = ng
                    f = ng + h((nx,ny), goal)
                    heapq.heappush(open_set, (f, ng, (nx,ny), path))
    return []

def generate_true_grid():
    g = [[0]*SIZE for _ in range(SIZE)]
    for y in range(SIZE):
        for x in range(SIZE):
            if (x,y) not in (START,GOAL) and random.random() < OBSTACLE_DENSITY:
                g[y][x] = 1
    return g

def lidar_scan(true_grid, pos, radius=LIDAR_RADIUS, p_false=P_FALSE, p_miss=P_MISS):
    size = len(true_grid)
    x0, y0 = pos
    observed = []
    for y in range(max(0, y0-radius), min(size, y0+radius+1)):
        for x in range(max(0, x0-radius), min(size, x0+radius+1)):
            if np.hypot(x-x0, y-y0) <= radius:
                true = true_grid[y][x]
                if true == 0:
                    meas = 1 if random.random() < p_false else 0
                else:
                    meas = 0 if random.random() < p_miss else 1
                observed.append((x,y,meas))
    return observed

def logodds_to_prob(L):
    return 1.0 / (1.0 + np.exp(-L))

def update_logodds(logodds_grid, observations):
    for x,y,meas in observations:
        if meas == 1:
            logodds_grid[y][x] = np.clip(logodds_grid[y][x] + L_OCC, L_MIN, L_MAX)
        else:
            logodds_grid[y][x] = np.clip(logodds_grid[y][x] - L_FREE, L_MIN, L_MAX)

def build_cost_and_block(logodds_grid, W_RISK=2.0, W_UNK=1.0, P_BLOCK=0.7):
    h, w = len(logodds_grid), len(logodds_grid[0])
    p_grid = np.zeros((h, w), dtype=float)
    blocked = np.zeros((h, w), dtype=bool)
    cost = np.ones((h, w), dtype=float)
    for y in range(h):
        for x in range(w):
            p = logodds_to_prob(logodds_grid[y][x])
            p_grid[y, x] = p
            if p >= P_BLOCK:
                blocked[y, x] = True
            unknown = 1.0 - abs(p - 0.5) * 2.0
            cost[y, x] = 1.0 + W_RISK*p + W_UNK*unknown
    blocked[START[1], START[0]] = False
    blocked[GOAL[1],  GOAL[0]] = False
    return cost, blocked

def run_trial(W_UNK, P_BLOCK):
    true_grid = generate_true_grid()
    logodds = np.zeros((SIZE, SIZE), dtype=float)
    pos = START
    steps = 0
    while pos != GOAL and steps < MAX_STEPS:
        obs = lidar_scan(true_grid, pos)
        update_logodds(logodds, obs)
        cost, blocked = build_cost_and_block(logodds, W_UNK=W_UNK, P_BLOCK=P_BLOCK)
        path = a_star_with_cost(cost, pos, GOAL, blocked)
        if not path: return False, steps
        pos = path[1] if len(path) > 1 else pos
        steps += 1
    return (pos == GOAL), steps

def main():
    P_BLOCK_values = [0.5, 0.6, 0.7, 0.8]
    W_UNK_values = [0.0, 0.5, 1.0, 2.0]
    results_success = np.zeros((len(W_UNK_values), len(P_BLOCK_values)))
    results_steps = np.zeros((len(W_UNK_values), len(P_BLOCK_values)))

    for i, W_UNK in enumerate(W_UNK_values):
        for j, P_BLOCK in enumerate(P_BLOCK_values):
            succ, total_steps = 0, 0
            for _ in range(TRIALS):
                ok, steps = run_trial(W_UNK, P_BLOCK)
                if ok:
                    succ += 1
                    total_steps += steps
            results_success[i,j] = succ / TRIALS
            results_steps[i,j] = (total_steps / succ) if succ>0 else np.nan

    fig, axes = plt.subplots(1,2, figsize=(12,5))
    im0 = axes[0].imshow(results_success, cmap="viridis", origin="upper")
    axes[0].set_xticks(range(len(P_BLOCK_values)))
    axes[0].set_xticklabels(P_BLOCK_values)
    axes[0].set_yticks(range(len(W_UNK_values)))
    axes[0].set_yticklabels(W_UNK_values)
    axes[0].set_xlabel("P_BLOCK threshold")
    axes[0].set_ylabel("W_UNK penalty")
    axes[0].set_title("Success rate")
    fig.colorbar(im0, ax=axes[0])

    im1 = axes[1].imshow(results_steps, cmap="plasma", origin="upper")
    axes[1].set_xticks(range(len(P_BLOCK_values)))
    axes[1].set_xticklabels(P_BLOCK_values)
    axes[1].set_yticks(range(len(W_UNK_values)))
    axes[1].set_yticklabels(W_UNK_values)
    axes[1].set_xlabel("P_BLOCK threshold")
    axes[1].set_ylabel("W_UNK penalty")
    axes[1].set_title("Average steps (successful only)")
    fig.colorbar(im1, ax=axes[1])

    plt.suptitle("Parameter sweep: Unknown penalty & Block threshold")
    plt.show()

if __name__ == "__main__":
    main()