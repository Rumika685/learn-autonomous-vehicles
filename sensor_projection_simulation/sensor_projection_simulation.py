import matplotlib.pyplot as plt
import numpy as np
import random
import time

# 定数
GRID_SIZE = 10
START = (5, 5)
GOAL_STEPS = 15
DIRECTIONS = ['east', 'south', 'west', 'north']
VEC_TABLE = {
    'east': np.array([1, 0]),
    'south': np.array([0, 1]),
    'west': np.array([-1, 0]),
    'north': np.array([0, -1])
}
ROTATION_MATRICES = {
    'east': np.array([[1, 0], [0, 1]]),
    'south': np.array([[0, 1], [-1, 0]]),
    'west': np.array([[-1, 0], [0, -1]]),
    'north': np.array([[0, -1], [1, 0]])
}
SENSOR_VECTORS = [np.array([-1, 0]), np.array([0, 1]), np.array([1, 0])]  # left, front, right

class RumiCar:
    def __init__(self, ax):
        self.ax = ax
        self.pos = np.array(START)
        self.dir_index = 3  # 'north'
        self.route = [tuple(self.pos)]
        self.sensor_log = []

    def sense(self):
        # センサー情報をランダムに生成
        sensor = tuple(random.choice([True, False]) for _ in range(3))  # left, front, right
        self.sensor_log.append(sensor)
        print(f"RumiCar のセンサー状態: 前={sensor[1]}, 左={sensor[0]}, 右={sensor[2]}")
        return sensor

    def decide_and_move(self, sensors):
        options = []
        if not sensors[1]: options.append('forward')
        if not sensors[0]: options.append('left')
        if not sensors[2]: options.append('right')
        if not options:
            print("🛑 全方向ふさがれているため停止")
            return False

        choice = random.choice(options)
        print(f"→ {choice}に回避します")

        # 進行方向を更新
        if choice == 'left':
            self.dir_index = (self.dir_index - 1) % 4
        elif choice == 'right':
            self.dir_index = (self.dir_index + 1) % 4

        if choice == 'forward':
            facing = DIRECTIONS[self.dir_index]
            move = VEC_TABLE[facing]
            self.pos += move
            self.route.append(tuple(self.pos))

        return True

    def project_sensors_on_map(self, grid, sensors):
        facing = DIRECTIONS[self.dir_index]
        rot = ROTATION_MATRICES[facing]

        for i, active in enumerate(sensors):
            if active:
                local_vec = SENSOR_VECTORS[i]
                rotated = rot @ local_vec
                projected = self.pos + rotated

                x, y = projected
                if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                    grid[int(y)][int(x)] = "✕"

    def drive(self, steps, grid):
        for _ in range(steps):
            sensors = self.sense()
            self.project_sensors_on_map(grid, sensors)
            moved = self.decide_and_move(sensors)
            self.visualize(grid)
            if not moved:
                break

    def visualize(self, grid):
        self.ax.clear()
        self.ax.set_title("RumiCar Sensor Mapping")
        self.ax.set_xlim(-0.5, GRID_SIZE - 0.5)
        self.ax.set_ylim(-0.5, GRID_SIZE - 0.5)
        self.ax.set_aspect("equal")
        self.ax.grid(True)
        self.ax.invert_yaxis()

        # 障害物（✕）を描画
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                if grid[y][x] == "✕":
                    self.ax.text(x, y, "✕", ha="center", va="center", fontsize=14, color="red")

        # ルート描画
        xs, ys = zip(*self.route)
        self.ax.plot(xs, ys, "b--")
        self.ax.plot(xs[-1], ys[-1], "ro")

        # 向きベクトル
        facing = DIRECTIONS[self.dir_index]
        dx, dy = VEC_TABLE[facing]
        self.ax.arrow(xs[-1], ys[-1], dx*0.3, dy*0.3, head_width=0.2, head_length=0.2, fc='black', ec='black')

        plt.pause(0.6)

# 実行
fig, ax = plt.subplots()
grid = [["" for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

car = RumiCar(ax)
car.drive(GOAL_STEPS, grid)

plt.show()
