# occupancy_planning_logodds.py
import random
import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# ===== 基本設定 =====
SIZE = 20
START = (0, 0)
GOAL = (SIZE - 1, SIZE - 1)
OBSTACLE_DENSITY = 0.25

# LiDAR
LIDAR_RADIUS = 5
# センサーノイズ（誤検知と見逃し）
P_FALSE = 0.05   # 空き→障害物 と誤検知
P_MISS  = 0.05   # 障害物→空き を見逃し

# ログオッズ更新ゲイン（センサーの確信度）
# 観測が「障害物」なら +L_OCC、 「空き」なら -L_FREE を足し込み
L_OCC  = 2.2   # 大きいほど「障害物」観測を強く信じる
L_FREE = 2.2   # 大きいほど「空き」観測を強く信じる
L_MIN, L_MAX = -8.0, 8.0   # クリップ（数値安定）

# プランナの危険度設定
P_BLOCK = 0.70            # これ以上の占有確率は通行不可にする
W_RISK  = 2.0             # 占有確率コストの重み（高いほどリスク回避）
W_UNK   = 1.0             # 未知ペナルティの重み（高いほど未知を避ける）

MAX_STEPS = SIZE * SIZE * 2

# ===== A*（セル毎コスト対応） =====
def a_star_with_cost(cost_grid, start, goal, blocked_mask):
    """
    cost_grid[y][x]: 基本移動コスト（>=1）
    blocked_mask[y][x]: True のセルは探索不可
    4近傍で計画
    """
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

# ===== 真の環境生成 =====
def generate_true_grid(size=SIZE, density=OBSTACLE_DENSITY, start=START, goal=GOAL):
    g = [[0]*size for _ in range(size)]
    for y in range(size):
        for x in range(size):
            if (x,y) not in (start,goal) and random.random() < density:
                g[y][x] = 1
    return g

# ===== LiDAR風観測（円形半径・ノイズ込み） =====
def lidar_scan(true_grid, pos, radius=LIDAR_RADIUS, p_false=P_FALSE, p_miss=P_MISS):
    size = len(true_grid)
    x0, y0 = pos
    observed = []
    for y in range(max(0, y0-radius), min(size, y0+radius+1)):
        for x in range(max(0, x0-radius), min(size, x0+radius+1)):
            if np.hypot(x-x0, y-y0) <= radius:
                true = true_grid[y][x]
                # ノイズ適用
                if true == 0:
                    meas = 1 if random.random() < p_false else 0
                else:
                    meas = 0 if random.random() < p_miss else 1
                observed.append((x, y, meas))  # meas: 0=free観測, 1=occupied観測
    return observed

# ===== ログオッズ更新 =====
def logodds_to_prob(L):
    # p = 1 - 1/(1+e^L) と同じ（シグモイド）
    return 1.0 / (1.0 + np.exp(-L))

def update_logodds(logodds_grid, observations):
    """
    observations: (x,y,meas) meas=0:free観測, 1:occupied観測
    """
    for x, y, meas in observations:
        if meas == 1:
            logodds_grid[y][x] = np.clip(logodds_grid[y][x] + L_OCC, L_MIN, L_MAX)
        else:
            logodds_grid[y][x] = np.clip(logodds_grid[y][x] - L_FREE, L_MIN, L_MAX)

# ===== コストマップ生成 =====
def build_cost_and_block(logodds_grid):
    """
    占有確率 p を計算し、以下で計画用のコストを与える:
      - p >= P_BLOCK なら通行不可
      - それ以外は base=1 に、 W_RISK*p + W_UNK*unknown を加算
    unknown 指標は |p-0.5| の反転で近似（0.5 付近ほど未知: 1、0 or 1 近いほど確定: 0）
    """
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
            # 不確実性（未知ペナルティに使う）
            unknown = 1.0 - abs(p - 0.5) * 2.0  # p=0.5 →1, p=0/1 →0
            cost[y, x] = 1.0 + W_RISK * p + W_UNK * unknown

    # スタート／ゴールは必ず探索可能に
    blocked[START[1], START[0]] = False
    blocked[GOAL[1],  GOAL[0]]  = False
    return cost, blocked, p_grid

# ===== 可視化 =====
def draw(ax, p_grid, trail, planned_path):
    """
    p_grid: 占有確率（0..1）
    trail: 実走行軌跡
    planned_path: 現在の計画経路
    """
    size = p_grid.shape[0]
    # 可視化用：0..1 を色に（白=自由, 赤=障害物, 灰=中間/未知）
    # ここでは簡易に三値マップも重畳
    vis = np.zeros((size, size, 3), dtype=float)
    for y in range(size):
        for x in range(size):
            p = p_grid[y, x]
            # 自由(白)〜障害(赤)のグラデーション。p=0.5 付近は薄灰で不確実を示す
            if abs(p - 0.5) < 0.08:
                vis[y, x] = [0.85, 0.85, 0.85]  # 不確実：薄灰
            else:
                vis[y, x] = [1.0, 1.0 - p, 1.0 - p]  # pが高いほど赤みが増す

    ax.clear()
    ax.imshow(vis, origin="upper", extent=(-0.5, size-0.5, size-0.5, -0.5))
    ax.set_xticks(range(size)); ax.set_yticks(range(size))
    ax.grid(True, color="#aaaaaa", linewidth=0.5, alpha=0.6)

    # 経路・ラベル
    ax.text(START[0], START[1], "START", color="green", ha="center", va="center", fontsize=9, fontweight="bold")
    ax.text(GOAL[0],  GOAL[1],  "GOAL",  color="blue",  ha="center", va="center", fontsize=9, fontweight="bold")

    if len(trail) >= 2:
        xs, ys = zip(*trail)
        ax.plot(xs, ys, "b-", linewidth=2)
    if trail:
        ax.plot(trail[-1][0], trail[-1][1], "ro")

    if planned_path and len(planned_path) >= 2:
        px, py = zip(*planned_path)
        ax.plot(px, py, "g--", linewidth=2)

    ax.set_title("A* with Occupancy Probabilities (unknown penalized)")

# ===== メイン =====
def main():
    true_grid = generate_true_grid()

    # ログオッズ初期化（0 → p=0.5：完全未知）
    logodds = np.zeros((SIZE, SIZE), dtype=float)

    pos = START
    trail = [pos]
    planned = []
    steps = 0

    fig, ax = plt.subplots(figsize=(6,6))

    while pos != GOAL and steps < MAX_STEPS:
        # センサー観測 → ログオッズ更新
        obs = lidar_scan(true_grid, pos, radius=LIDAR_RADIUS, p_false=P_FALSE, p_miss=P_MISS)
        update_logodds(logodds, obs)

        # 計画用のコスト＆通行不可生成
        cost, blocked, p_grid = build_cost_and_block(logodds)

        # A*（現在位置→ゴール）
        path = a_star_with_cost(cost, pos, GOAL, blocked)
        planned = path

        # 可視化
        draw(ax, p_grid, trail, planned)
        plt.pause(0.05)

        if not path:
            ax.set_title("No path under current risk/unknown settings (final state kept)")
            break

        # 1歩前進
        pos = path[1] if len(path) > 1 else pos
        trail.append(pos)
        steps += 1

    # 終了可視化：ゴールなら計画線を消し、軌跡を残す
    _, _, p_grid = build_cost_and_block(logodds)
    if pos == GOAL:
        draw(ax, p_grid, trail, planned_path=[])
        ax.set_title("✅ Goal reached (final state is kept)")
    else:
        draw(ax, p_grid, trail, planned_path=planned)

    # 必要なら保存
    # plt.savefig("final_occupancy_plan.png", dpi=150)
    plt.show()

if __name__ == "__main__":
    main()