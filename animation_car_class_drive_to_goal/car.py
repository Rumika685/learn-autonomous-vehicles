import matplotlib
matplotlib.use('TkAgg')
import csv
import random
import numpy as np
import matplotlib.pyplot as plt

class Car:
    def __init__(self, brand, manual_mode=False, ax=None):
        self.brand = brand
        self.manual_mode = manual_mode
        self.ax = ax  # matplotlib の Axes
        self.pos = np.array([0, 0])
        self.position = 0
        self.dir_index = 0
        self.directions = ['east', 'south', 'west', 'north']
        self.vec_table = {
            'east': np.array([1, 0]),
            'south': np.array([0, 1]),
            'west': np.array([-1, 0]),
            'north': np.array([0, -1])
        }

        self.route = []
        self.obstacle_map = []
        self.log = []
        self.last_direction = None
        self.rule_based_sensor_inputs = []
        self.rule_index = 0
        self.route_positions = [tuple(self.pos)]

    def sense_environment(self):
        if self.rule_based_sensor_inputs:
            sensors = self.rule_based_sensor_inputs[self.rule_index % len(self.rule_based_sensor_inputs)]
            self.rule_index += 1
        elif self.manual_mode:
            sensors = (
                input("左に障害物？ (y/n): ").lower() == "y",
                input("前に障害物？ (y/n): ").lower() == "y",
                input("右に障害物？ (y/n): ").lower() == "y"
            )
        else:
            sensors = tuple(random.choice([True, False]) for _ in range(3))

        print(f"{self.brand} のセンサー状態: 前={sensors[1]}, 左={sensors[0]}, 右={sensors[2]}")
        self.obstacle_map.append(sensors)
        return sensors

    def decide_direction(self):
        left, front, right = self.sense_environment()

        # 進行方向を決める（ランダム選択）
        options = []
        if not front: options.append("forward")
        if not left: options.append("left")
        if not right: options.append("right")

        if not options:
            return False

        direction = random.choice(options)
        print(f"→ {direction}に回避します")

        # 進行方向の更新
        if direction == "left":
            self.dir_index = (self.dir_index - 1) % 4
        elif direction == "right":
            self.dir_index = (self.dir_index + 1) % 4

        facing = self.directions[self.dir_index]

        if direction == "forward":
            move = self.vec_table[facing]
            self.pos += move
            self.position += 1
            self.route_positions.append(tuple(self.pos))
        else:
            # ✅ 前進以外のときも「その場で記録」
            self.route_positions.append(tuple(self.pos))
        return True

    def drive_to_goal(self, goal=5, max_steps=20):
        steps = 0

        if self.ax:
            # ✅ 先にスケールを決定
            self.ax.set_xlim(-5, 5)
            self.ax.set_ylim(-5, 5)

        while self.position < goal and steps < max_steps:
            moved = self.decide_direction()
            if not moved:
                print("全方向に障害物 → 停止")
                break
            steps += 1

            # ✅ matplotlib でリアルタイム描画
            if self.ax:
                self.ax.clear()
                self.ax.set_title("🚗 RumiCar Real-Time")
                self.ax.set_aspect("equal")
                self.ax.grid(True)

                # ✅ 範囲は再設定しない
                self.ax.set_xlim(-5, 5)
                self.ax.set_ylim(-5, 5)

                xs, ys = zip(*self.route_positions)
                self.ax.plot(xs, ys, "b--")
                self.ax.plot(xs[-1], ys[-1], "ro")

                # ✅ 進行方向のベクトルを計算（pause前に）
                dx, dy = self.vec_table[self.directions[self.dir_index]]

                # ✅ 赤い矢印を描画（pause前に）
                self.ax.arrow(xs[-1], ys[-1], dx*0.3, dy*0.3,
                            head_width=0.2, head_length=0.2, fc='red', ec='red') 
                plt.pause(0.5)


        print("✅ ゴール到達" if self.position >= goal else "🛑 ゴール未達")
