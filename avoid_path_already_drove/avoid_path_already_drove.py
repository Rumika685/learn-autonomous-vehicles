import numpy as np
import matplotlib.pyplot as plt
import random

class Car:
    def __init__(self, grid_size=10):
        self.grid_size = grid_size
        self.map = np.zeros((grid_size, grid_size), dtype=int)

        self.start_pos = np.array([9, 0])  # 右下
        self.goal_pos = np.array([0, 9])   # 左上

        self.pos = self.start_pos.copy()
        self.dir_index = 3  # north
        self.directions = ['east', 'south', 'west', 'north']
        self.vec_table = {
            'east':  np.array([1, 0]),
            'south': np.array([0, 1]),
            'west':  np.array([-1, 0]),
            'north': np.array([0, -1]),
        }

        self.route_positions = [tuple(self.pos)]
        self.visited = {tuple(self.pos)}  # ✅ 通過地点の記録
        self._generate_obstacles(prob=0.25)

    def _generate_obstacles(self, prob=0.25):
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                if (x, y) in [tuple(self.start_pos), tuple(self.goal_pos)]:
                    continue
                if random.random() < prob:
                    self.map[y][x] = 1

    def sense_environment(self):
        result = []
        for offset in [-1, 0, 1]:  # left, front, right
            dir_idx = (self.dir_index + offset) % 4
            vec = self.vec_table[self.directions[dir_idx]]
            check_pos = self.pos + vec
            if (0 <= check_pos[0] < self.grid_size) and (0 <= check_pos[1] < self.grid_size):
                x, y = check_pos
                result.append(self.map[y][x] == 1)
            else:
                result.append(True)
        return tuple(result)

    def decide_direction(self, left, front, right):
        options = []
        offsets = {'left': -1, 'front': 0, 'right': 1}

        for dir_name, offset, blocked in zip(['left', 'forward', 'right'], [-1, 0, 1], [left, front, right]):
            if blocked:
                continue
            dir_idx = (self.dir_index + offset) % 4
            vec = self.vec_table[self.directions[dir_idx]]
            new_pos = tuple(self.pos + vec)
            if new_pos not in self.visited:
                options.append((dir_name, dir_idx, vec))

        if not options:
            return False

        direction, new_dir_idx, move_vec = random.choice(options)

        if direction in ["left", "right"]:
            self.dir_index = new_dir_idx

        if direction == "forward":
            self.pos += move_vec
            self.visited.add(tuple(self.pos))

        elif direction in ["left", "right"]:
            # 向きを変えただけでは位置は変わらないが、記録は必要
            self.visited.add(tuple(self.pos))

        self.route_positions.append(tuple(self.pos))
        return True

    def drive(self):
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.grid_size)
        ax.set_ylim(0, self.grid_size)
        ax.set_aspect('equal')
        ax.set_title("🚗 RumiCar: Avoiding Visited Paths")

        while not np.array_equal(self.pos, self.goal_pos):
            left, front, right = self.sense_environment()
            moved = self.decide_direction(left, front, right)
            if not moved:
                print("🛑 停止：進路がありません（全て障害物 or 訪問済）")
                break

            ax.clear()
            ax.set_xlim(0, self.grid_size)
            ax.set_ylim(0, self.grid_size)
            ax.set_aspect('equal')
            ax.grid(True)

            # 障害物表示
            for y in range(self.grid_size):
                for x in range(self.grid_size):
                    if self.map[y][x] == 1:
                        ax.text(x, y, "✕", color="red", fontsize=12, ha="center", va="center")

            # スタートとゴール
            sx, sy = tuple(self.start_pos)
            gx, gy = tuple(self.goal_pos)
            ax.text(sx, sy, "START", fontsize=10, ha="center", va="center", color="green")
            ax.text(gx, gy, "GOAL", fontsize=10, ha="center", va="center", color="orange")

            # 経路と現在位置
            xs, ys = zip(*self.route_positions)
            ax.plot(xs, ys, 'b--')
            ax.plot(xs[-1], ys[-1], 'ro')

            # 進行方向矢印
            dx, dy = self.vec_table[self.directions[self.dir_index]]
            ax.arrow(xs[-1], ys[-1], dx * 0.3, dy * 0.3,
                     head_width=0.2, head_length=0.2, fc='red', ec='red')

            plt.pause(0.3)

        if np.array_equal(self.pos, self.goal_pos):
            print("🎉 ゴール到達！")
        else:
            print("❌ ゴール未到達")

        plt.show()

# 実行
if __name__ == "__main__":
    car = Car(grid_size=10)
    car.drive()
