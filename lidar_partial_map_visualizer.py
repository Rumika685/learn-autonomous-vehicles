# lidar_partial_map_visualizer.py
import random
import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# ====== 基本設定 ======
SIZE = 20
START = (0, 0)
GOAL = (SIZE - 1, SIZE - 1)
OBSTACLE_DENSITY = 0.25      # 真の環境の障害物密度
LIDAR_RADIUS = 5             # センサー観測半径（マス）
NOISE = False                # センサーノイズ（誤検知/見逃し）を使うか
P_FALSE = 0.05               # 誤検知確率（空き→障害物）
P_MISS  = 0.05               # 見逃し確率（障害物→空き）
MAX_STEPS = SIZE * SIZE * 2  # 安全上限

# ====== A* ======
def a_star(grid, start, goal):
    """grid: 0=free, 1=blocked"""
    size = len(grid)
    h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])
    open_set = [(h(start, goal), 0, start, [])]
    visited = set()
    while open_set:
        _, cost, cur, path = heapq.heappop(open_set)
        if cur in visited: 
            continue
        visited.add(cur)
        path = path + [cur]
        if cur == goal:
            return path
        x, y = cur
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < size and 0 <= ny < size and grid[ny][nx] == 0:
                heapq.heappush(open_set, (cost+1+h((nx,ny),goal), cost+1, (nx,ny), path))
    return []  # 見つからない

# ====== 真の環境生成 ======
def generate_true_grid(size=SIZE, density=OBSTACLE_DENSITY, start=START, goal=GOAL):
    g = [[0]*size for _ in range(size)]
    for y in range(size):
        for x in range(size):
            if (x,y) not in (start,goal) and random.random() < density:
                g[y][x] = 1
    return g

# ====== LiDAR観測（円形半径） ======
def lidar_scan(true_grid, pos, radius=LIDAR_RADIUS, noise=NOISE, p_false=P_FALSE, p_miss=P_MISS):
    size = len(true_grid)
    x0, y0 = pos
    observed = []
    for y in range(max(0, y0-radius), min(size, y0+radius+1)):
        for x in range(max(0, x0-radius), min(size, x0+radius+1)):
            if np.hypot(x-x0, y-y0) <= radius:
                val = true_grid[y][x]
                if noise:
                    if val == 0 and random.random() < p_false: val = 1  # 誤検知
                    if val == 1 and random.random() < p_miss:  val = 0  # 見逃し
                observed.append((x, y, val))
    return observed

# ====== 可視化補助 ======
def draw_map(ax, partial_grid, trail, planned_path):
    """
    partial_grid: -1=unknown, 0=free, 1=obstacle
    trail: 走行履歴（実績）
    planned_path: 現在の計画経路
    """
    size = len(partial_grid)
    # カテゴリ → 表示値： unknown=0, free=1, obstacle=2
    vis = np.zeros((size, size), dtype=int)
    for y in range(size):
        for x in range(size):
            if partial_grid[y][x] == -1:
                vis[y][x] = 0
            elif partial_grid[y][x] == 0:
                vis[y][x] = 1
            else:
                vis[y][x] = 2

    # カラーマップ（unknown=light gray, free=white, obstacle=light red）
    cmap = ListedColormap(["#dddddd", "#ffffff", "#ffcccc"])

    ax.clear()
    ax.imshow(vis, cmap=cmap, origin="upper", extent=( -0.5, size-0.5, size-0.5, -0.5 ))
    ax.set_xticks(range(size))
    ax.set_yticks(range(size))
    ax.grid(True, color="#999999", linewidth=0.5, alpha=0.5)

    # Start / Goal
    ax.text(START[0], START[1], "START", color="green", ha="center", va="center", fontsize=10, fontweight="bold")
    ax.text(GOAL[0],  GOAL[1],  "GOAL",  color="blue",  ha="center", va="center", fontsize=10, fontweight="bold")

    # 実走行履歴（青）
    if len(trail) >= 2:
        xs, ys = zip(*trail)
        ax.plot(xs, ys, "b-", linewidth=2)
    if trail:
        ax.plot(trail[-1][0], trail[-1][1], "ro")  # 現在位置

    # 計画経路（緑）
    if planned_path and len(planned_path) >= 2:
        px, py = zip(*planned_path)
        ax.plot(px, py, "g--", linewidth=2)

    ax.set_title("Partial-Map Planning (keeps final state after Goal)")

# ====== メイン：部分マップで逐次再計画 ======
def main():
    true_grid = generate_true_grid()
    partial = [[-1]*SIZE for _ in range(SIZE)]  # -1=unknown, 0=free, 1=obstacle
    partial[START[1]][START[0]] = 0
    partial[GOAL[1]][GOAL[0]] = 0

    pos = START
    trail = [pos]
    planned_path = []
    steps = 0

    fig, ax = plt.subplots(figsize=(6,6))

    while pos != GOAL and steps < MAX_STEPS:
        # センサーで観測 → 部分マップ更新
        for x, y, val in lidar_scan(true_grid, pos):
            partial[y][x] = val

        # 未知セルは自由扱いで推定グリッドを作成（探索を促す）
        est_grid = [[1 if partial[y][x] == 1 else 0 for x in range(SIZE)] for y in range(SIZE)]

        # A* 再計画（現在位置 → ゴール）
        path = a_star(est_grid, pos, GOAL)
        planned_path = path

        # 可視化更新
        draw_map(ax, partial, trail, planned_path)
        plt.pause(0.1)

        if not path:
            ax.set_title("No path (with current partial map). Final state shown.")
            break

        # 1歩進む
        pos = path[1] if len(path) > 1 else pos
        trail.append(pos)
        steps += 1

    # ====== 終了処理（最終状態を保持） ======
    if pos == GOAL:
        draw_map(ax, partial, trail, planned_path=[])  # 計画線は消して軌跡のみ強調
        ax.set_title("✅ Goal reached (final state is kept)")
    else:
        # 失敗時でも最終状態を保持
        draw_map(ax, partial, trail, planned_path=planned_path)
        ax.set_title("🛑 Stopped (final state is kept)")

    # 任意：最終図を保存（必要なら）
    # plt.savefig("final_partial_map.png", dpi=150)
    plt.show()

if __name__ == "__main__":
    main()