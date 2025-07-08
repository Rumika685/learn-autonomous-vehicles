import numpy as np
import matplotlib.pyplot as plt
import random

class Car:
    def __init__(self, ax=None):
        self.map_size = 10
        self.map = np.zeros((self.map_size, self.map_size), dtype=int)

        # 自動的に障害物をランダム配置（数を調整可能）
        obstacle_count = 15
        for _ in range(obstacle_count):
            x = random.randint(0, self.map_size - 1)
            y = random.randint(0, self.map_size - 1)
            self.map[y, x] = 1  # 注意: yが行, xが列

        # 車の初期状態
        self.pos = np.array([5, 5])  # 中心 (x, y)
        self.dir_index = 0  # 0=north, 1=east, 2=south, 3=west
        self.directions = ['north', 'east', 'south', 'west']
        self.vec_table = {
            'north': np.array([0, -1]),
            'east':  np.array([1, 0]),
            'south': np.array([0, 1]),
            'west':  np.array([-1, 0])
        }

        self.route_positions = [tuple(self.pos)]
        self.ax = ax
        self.goal_steps = 50
        self.steps = 0

    def sense_environment_from_map(self):
        """マップからセンサー情報を取得"""
        dir_names = ['left', 'front', 'right']
        left_dir = self.directions[(self.dir_index - 1) % 4]
        front_dir = self.directions[self.dir_index]
        right_dir = self.directions[(self.dir_index + 1) % 4]
        sensor_dirs = [left_dir, front_dir, right_dir]

        results = []
        for d in sensor_dirs:
            delta = self.vec_table[d]
            check_pos = self.pos + delta
            x, y = check_pos
            if 0 <= x < self.map_size and 0 <= y < self.map_size:
                results.append(self.map[y, x] == 1)  # 1なら障害物
            else:
                results.append(True)  # 外は壁扱い
        return tuple(results)

    def decide_direction(self):
        left, front, right = self.sense_environment_from_map()

        options = []
        if not front: options.append("forward")
        if not left: options.append("left")
        if not right: options.append("right")

        if not options:
            return False

        direction = random.choice(options)

        if direction == "left":
            self.dir_index = (self.dir_index - 1) % 4
        elif direction == "right":
            self.dir_index = (self.dir_index + 1) % 4

        if direction == "forward":
            move = self.vec_table[self.directions[self.dir_index]]
            new_pos = self.pos + move
            if self.map[new_pos[1], new_pos[0]] == 0:  # 安全確認
                self.pos = new_pos
                self.route_positions.append(tuple(self.pos))
                self.steps += 1
        else:
            self.route_positions.append(tuple(self.pos))

        return True

    def drive_to_goal(self):
        while self.steps < self.goal_steps:
            moved = self.decide_direction()
            if not moved:
                print("🛑 進めませんでした")
                break
            if self.ax:
                self.draw()

        print("✅ 終了: ステップ =", self.steps)

    def draw(self):
        self.ax.clear()
        self.ax.set_title("🌍 Auto Map Navigation")
        self.ax.set_xlim(-1, self.map_size)
        self.ax.set_ylim(-1, self.map_size)
        self.ax.set_aspect('equal')
        self.ax.grid(True)

        # 障害物描画
        for y in range(self.map_size):
            for x in range(self.map_size):
                if self.map[y, x] == 1:
                    self.ax.text(x, y, "✕", color="red", ha="center", va="center")

        # ルートと車の位置
        xs, ys = zip(*self.route_positions)
        self.ax.plot(xs, ys, "b--")
        self.ax.plot(xs[-1], ys[-1], "ro")

        # 向き矢印
        dx, dy = self.vec_table[self.directions[self.dir_index]]
        self.ax.arrow(xs[-1], ys[-1], dx*0.3, dy*0.3, head_width=0.3, head_length=0.3, fc='red', ec='red')

        plt.pause(0.5)

# 実行
fig, ax = plt.subplots()
car = Car(ax=ax)
car.drive_to_goal()
plt.show()
