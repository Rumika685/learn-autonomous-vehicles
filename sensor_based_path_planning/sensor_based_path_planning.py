import matplotlib.pyplot as plt
import numpy as np
import heapq
import random
import time

# ==== 設定 ====
GRID_SIZE = 10
START = (9, 0)  # 右下
GOAL = (0, 9)   # 左上
SENSOR_RANGE = 1  # 前後左右1マスを感知

# ==== A*探索 ====
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

        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and grid[ny][x:=nx] == 0:
                heapq.heappush(open_set, (cost + 1 + h((nx, ny), goal), cost + 1, (nx, ny), path))
    return []

# ==== 初期マップ生成 ====
def generate_grid():
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    count = 0
    while count < 20:
        x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
        if (x, y) not in (START, GOAL):
            grid[y][x] = 1
            count += 1
    return grid

# ==== センサーで障害物検知 ====
def sense_obstacles(pos, grid):
    px, py = pos
    detected = []
    for dx, dy in [(0,1),(0,-1),(1,0),(-1,0)]:
        nx, ny = px + dx, py + dy
        if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
            if grid[ny][nx] == 1:
                detected.append((nx, ny))
    return detected

# ==== 描画 ====
def draw_map(grid, path, history):
    plt.clf()
    ax = plt.gca()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, GRID_SIZE - 0.5)
    ax.set_ylim(-0.5, GRID_SIZE - 0.5)
    ax.invert_yaxis()
    ax.grid(True)

    # 障害物
    for y in range(GRID_SIZE):
        for x in range(GRID_SIZE):
            if grid[y][x] == 1:
                ax.text(x, y, "✕", ha="center", va="center", fontsize=14, color="red")

    # START & GOAL
    ax.text(START[0], START[1], "START", color="green", ha="center", va="center")
    ax.text(GOAL[0], GOAL[1], "GOAL", color="blue", ha="center", va="center")

    # 履歴
    if history:
        hx, hy = zip(*history)
        ax.plot(hx, hy, "b--")

    # 現在位置
    if path:
        ax.plot(path[0][0], path[0][1], "ro")

    plt.pause(0.3)

# ==== メイン ====
def main():
    grid = generate_grid()
    history = []
    current_pos = START

    while current_pos != GOAL:
        path = a_star(grid, current_pos, GOAL)

        if not path:  # 行き止まり → 後退
            if len(history) > 1:
                history.pop()  # 最後の位置を削除
                current_pos = history[-1]  # 1歩前に戻る
                print("🛑 Dead end, backtracking...")
                continue
            else:
                print("❌ No possible path to goal!")
                break

        # 次の位置へ移動
        current_pos = path[1] if len(path) > 1 else GOAL
        history.append(current_pos)

        # センサーで障害物を検知 → マップ更新
        detected = sense_obstacles(current_pos, grid)
        for (ox, oy) in detected:
            grid[oy][ox] = 1

        # 描画
        draw_map(grid, path, history)

    print("✅ Goal reached!")

if __name__ == "__main__":
    plt.figure()
    main()
    plt.show()
