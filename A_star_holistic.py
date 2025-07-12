import matplotlib.pyplot as plt
import numpy as np
import random
import heapq

GRID_SIZE = 10
NUM_OBSTACLES = 20
START = (9, 0)
GOAL = (0, 9)

def generate_obstacles():
    obstacles = set()
    while len(obstacles) < NUM_OBSTACLES:
        x = random.randint(0, GRID_SIZE - 1)
        y = random.randint(0, GRID_SIZE - 1)
        if (x, y) not in [START, GOAL]:
            obstacles.add((x, y))
    return obstacles

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan距離

def a_star(start, goal, obstacles):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < GRID_SIZE and
                0 <= neighbor[1] < GRID_SIZE and
                neighbor not in obstacles):
                
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
                    came_from[neighbor] = current
    return None

# 障害物生成
obstacles = generate_obstacles()
path = a_star(START, GOAL, obstacles)

# 描画
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-0.5, GRID_SIZE - 0.5)
ax.set_ylim(-0.5, GRID_SIZE - 0.5)
ax.set_xticks(range(GRID_SIZE))
ax.set_yticks(range(GRID_SIZE))
ax.grid(True)

# 障害物表示
for ox, oy in obstacles:
    ax.text(ox, oy, "✕", color="red", fontsize=14, ha="center", va="center")

# START・GOAL
sx, sy = START
gx, gy = GOAL
ax.plot(sx, sy, "go")
ax.plot(gx, gy, "ro")
ax.text(sx, sy, "START", color="green", ha="right", va="bottom", fontsize=8)
ax.text(gx, gy, "GOAL", color="red", ha="left", va="top", fontsize=8)

# 経路をアニメーションで表示
for i in range(len(path)):
    x, y = path[i]
    if i > 0:
        x0, y0 = path[i - 1]
        ax.arrow(x0, y0, x - x0, y - y0, head_width=0.2, head_length=0.2,
                 fc="blue", ec="blue", length_includes_head=True)
    ax.plot(x, y, "bo", markersize=4)
    plt.pause(0.3)

plt.title("🌍 A* Path Planning with Obstacles")
plt.show()
