import matplotlib.pyplot as plt
import numpy as np
import random

GRID_SIZE = 10
START = (9, 0)
GOAL = (0, 9)

# 回転行列で方向を更新
def rotate(vec, angle_deg):
    theta = np.deg2rad(angle_deg)
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    return np.round(rotation_matrix @ vec).astype(int)

# 車クラス
class Car:
    def __init__(self, ax=None):
        self.pos = np.array(START)
        self.heading = np.array([0, 1])  # 北向き（上方向）
        self.map = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
        self.sensed = np.zeros_like(self.map, dtype=bool)
        self.route = [tuple(self.pos)]
        self.ax = ax
        self.goal = np.array(GOAL)

        # 固定障害物を初期化（最大30個まで）
        self.obstacles = set()
        while len(self.obstacles) < 30:
            x, y = random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)
            if (x, y) != tuple(self.pos) and (x, y) != tuple(self.goal):
                self.obstacles.add((x, y))
                self.map[y][x] = 1

    def sense(self):
        sensor_angles = [-90, 0, 90]  # 左、前、右
        results = []
        for angle in sensor_angles:
            dir_vec = rotate(self.heading, angle)
            target = self.pos + dir_vec
            x, y = target
            if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                obstacle = self.map[y][x] == 1
                results.append(obstacle)
                if obstacle:
                    self.sensed[y][x] = True
            else:
                results.append(True)  # 枠外は障害物扱い
        return tuple(results)

    def decide_and_move(self):
        left, front, right = self.sense()
        print(f"RumiCar のセンサー状態: 前={front}, 左={left}, 右={right}")

        if not front:
            direction = "forward"
        elif not left:
            direction = "left"
        elif not right:
            direction = "right"
        else:
            print("🛑 全方向ふさがれました")
            return False

        print(f"→ {direction}に回避します")

        # 向きを更新
        if direction == "left":
            self.heading = rotate(self.heading, -90)
        elif direction == "right":
            self.heading = rotate(self.heading, 90)

        # 前進
        self.pos += self.heading
        self.route.append(tuple(self.pos))
        return True

    def at_goal(self):
        return np.array_equal(self.pos, self.goal)

    def draw(self):
        self.ax.clear()
        self.ax.set_xlim(-0.5, GRID_SIZE - 0.5)
        self.ax.set_ylim(-0.5, GRID_SIZE - 0.5)
        self.ax.set_aspect("equal")
        self.ax.invert_yaxis()
        self.ax.grid(True)

        # ゴールとスタート
        gx, gy = self.goal
        sx, sy = START
        self.ax.text(sx, sy, "START", ha="center", va="center", color="green")
        self.ax.text(gx, gy, "GOAL", ha="center", va="center", color="blue")

        # 障害物
        for (x, y) in self.obstacles:
            self.ax.text(x, y, "■", ha="center", va="center", color="red", fontsize=14)

        # 検知された障害物
        ys, xs = np.where(self.sensed)
        for x, y in zip(xs, ys):
            self.ax.text(x, y, "✕", ha="center", va="center", color="black", fontsize=12)

        # 通過ルート
        if self.route:
            xs, ys = zip(*self.route)
            self.ax.plot(xs, ys, "b--")
            self.ax.plot(xs[-1], ys[-1], "ro")

        self.ax.set_title("RumiCar with Sensor Mapping")
        plt.pause(0.5)

# 実行
fig, ax = plt.subplots()
car = Car(ax=ax)

while not car.at_goal():
    moved = car.decide_and_move()
    car.draw()
    if not moved:
        break

plt.show()
