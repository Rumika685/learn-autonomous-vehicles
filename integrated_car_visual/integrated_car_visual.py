import numpy as np
import matplotlib.pyplot as plt
import random

class Car:
    def __init__(self, grid_size=10):
        self.grid_size = grid_size
        self.map = np.zeros((grid_size, grid_size), dtype=int)

        self.pos = np.array([grid_size // 2, grid_size // 2])
        self.dir_index = 3  # 0:east, 1:south, 2:west, 3:north ← 北向き
        self.directions = ['east', 'south', 'west', 'north']
        self.vec_table = {
            'east':  np.array([1, 0]),
            'south': np.array([0, 1]),
            'west':  np.array([-1, 0]),
            'north': np.array([0, -1]),
        }

        self.route_positions = [tuple(self.pos)]
        self.obstacle_map = []  # [(左, 前, 右), ...]
        self.goal_steps = 50

    def sense_environment(self):
        # ランダム障害物配置（実運用では map 参照 or センサーデータ使用）
        sensed = []
        for offset in [-1, 0, 1]:  # 左・前・右
            dir_idx = (self.dir_index + offset) % 4
            vec = self.vec_table[self.directions[dir_idx]]
            check_pos = self.pos + vec
            if (0 <= check_pos[0] < self.grid_size) and (0 <= check_pos[1] < self.grid_size):
                # ランダムに障害物あり／なし
                sensed.append(random.random() < 0.3)
            else:
                sensed.append(True)  # 外は壁とみなす
        self.obstacle_map.append(tuple(sensed))
        return tuple(sensed)

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
        ax.set_title("🚗 RumiCar Integrated View")

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
            ax.set_title("🚗 RumiCar Integrated View")
            ax.grid(True)

            # 描画：障害物 ×
            pos = self.route_positions[-2]  # 現在のステップ前の位置
            dir_idx = self.dir_index
            sensor_dirs = [
                self.directions[(dir_idx - 1) % 4],
                self.directions[dir_idx],
                self.directions[(dir_idx + 1) % 4]
            ]
            sensor_vals = self.obstacle_map[-1]
            for d, sensed in zip(sensor_dirs, sensor_vals):
                if sensed:
                    dx, dy = self.vec_table[d]
                    obs_x, obs_y = pos[0] + dx, pos[1] + dy
                    ax.text(obs_x, obs_y, "✕", color="red", fontsize=12, ha="center", va="center")

            # 描画：軌跡
            xs, ys = zip(*self.route_positions)
            ax.plot(xs, ys, 'b--')
            ax.plot(xs[-1], ys[-1], 'ro')  # 現在位置

            # 進行方向矢印
            dx, dy = self.vec_table[self.directions[self.dir_index]]
            ax.arrow(xs[-1], ys[-1], dx * 0.3, dy * 0.3,
                     head_width=0.2, head_length=0.2, fc='red', ec='red')

            plt.pause(0.5)

        print("✅ 終了：走行完了")
        plt.show()


# ✅ 実行
if __name__ == "__main__":
    car = Car(grid_size=10)
    car.goal_steps = 50
    car.drive()
