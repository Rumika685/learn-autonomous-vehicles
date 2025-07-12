import numpy as np
import matplotlib.pyplot as plt
import random

class Car:
    def __init__(self, grid_size=10):
        self.grid_size = grid_size
        self.map = np.zeros((grid_size, grid_size), dtype=int)

        self.pos = np.array([grid_size // 2, grid_size // 2])
        self.dir_index = 3  # 北
        self.directions = ['east', 'south', 'west', 'north']
        self.vec_table = {
            'east':  np.array([1, 0]),
            'south': np.array([0, 1]),
            'west':  np.array([-1, 0]),
            'north': np.array([0, -1]),
        }

        self.route_positions = [tuple(self.pos)]
        self.goal_steps = 50

        # ✅ 事前に障害物マップを生成
        self._generate_obstacles(prob=0.2)

    def _generate_obstacles(self, prob=0.2):
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                if (x, y) == tuple(self.pos):
                    continue
                if random.random() < prob:
                    self.map[y][x] = 1  # 1が障害物

    def sense_environment(self):
        result = []
        for offset in [-1, 0, 1]:  # 左・前・右
            dir_idx = (self.dir_index + offset) % 4
            vec = self.vec_table[self.directions[dir_idx]]
            check_pos = self.pos + vec
            if (0 <= check_pos[0] < self.grid_size) and (0 <= check_pos[1] < self.grid_size):
                x, y = check_pos
                result.append(self.map[y][x] == 1)
            else:
                result.append(True)  # 外は壁
        return tuple(result)

    def decide_direction(self, left, front, right):
        options = []
        if not front: options.append("forward")
        if not left:  options.append("left")
        if not right: options.append("right")
        if not options:
            return False

        direction = random.choice(options)
        if direction == "left":
            self.dir_index = (self.dir_index - 1) % 4
        elif direction == "right":
            self.dir_index = (self.dir_index + 1) % 4
        elif direction == "forward":
            move_vec = self.vec_table[self.directions[self.dir_index]]
            self.pos += move_vec
        self.route_positions.append(tuple(self.pos))
        return True

    def drive(self):
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.grid_size)
        ax.set_ylim(0, self.grid_size)
        ax.set_aspect('equal')
        ax.set_title("🌍 RumiCar with Static Obstacles")

        for step in range(self.goal_steps):
            left, front, right = self.sense_environment()
            moved = self.decide_direction(left, front, right)
            if not moved:
                print("🛑 停止：全方向障害物")
                break

            ax.clear()
            ax.set_xlim(0, self.grid_size)
            ax.set_ylim(0, self.grid_size)
            ax.set_aspect('equal')
            ax.grid(True)

            # ✅ 全障害物を表示
            for y in range(self.grid_size):
                for x in range(self.grid_size):
                    if self.map[y][x] == 1:
                        ax.text(x, y, "✕", color="red", fontsize=12,
                                ha="center", va="center")

            # 軌跡と現在位置
            xs, ys = zip(*self.route_positions)
            ax.plot(xs, ys, 'b--')
            ax.plot(xs[-1], ys[-1], 'ro')

            # 進行方向
            dx, dy = self.vec_table[self.directions[self.dir_index]]
            ax.arrow(xs[-1], ys[-1], dx * 0.3, dy * 0.3,
                     head_width=0.2, head_length=0.2, fc='red', ec='red')

            plt.pause(0.4)

        print("✅ 終了")
        plt.show()


# ✅ 実行
if __name__ == "__main__":
    car = Car(grid_size=10)
    car.goal_steps = 50
    car.drive()
