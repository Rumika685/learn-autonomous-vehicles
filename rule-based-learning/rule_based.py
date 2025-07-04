import csv
import random

class Car:
    def __init__(self, brand, manual_mode=False):
        self.brand = brand
        self.position = 0
        self.route = []
        self.obstacle_map = []
        self.manual_mode = manual_mode
        self.last_direction = None
        self.x, self.y = 0, 0
        self.directions = ['east', 'south', 'west', 'north']
        self.dir_index = 0  # 初期方向：east
        self.log = []
        self.learned_route = []
        self.training_data = []
        self.test_sensor_inputs = []
        self.load_learned_route()
        self.rule_based_sensor_inputs = []  # 🎲 ルールベースのセンサー入力
        self.rule_index = 0  # 今どのルールを使っているかのカウンター                

    def load_learned_route(self, filename="learned_route.txt"):
        try:
            with open(filename, "r") as f:
                self.learned_route = [line.strip() for line in f.readlines()]
            print(f"📥 学習済みルートを読み込みました: {self.learned_route}")
        except FileNotFoundError:
            self.learned_route = []
            print("📂 学習ルートファイルが見つかりませんでした。")

    def sense_environment(self):
        # 優先順位: ルールベース > テスト入力 > 手動入力 > ランダム
        if self.rule_based_sensor_inputs:
            input_set = self.rule_based_sensor_inputs[self.rule_index % len(self.rule_based_sensor_inputs)]
            self.rule_index += 1
            left, front, right = input_set
            print(f"🎲 ルール入力: 左={left}, 前={front}, 右={right}")
        elif self.test_sensor_inputs:
            left, front, right = self.test_sensor_inputs.pop(0)
            print(f"📦 テスト入力: 左={left}, 前={front}, 右={right}")
        elif self.manual_mode:
            left = input("左に障害物がありますか？ (y/n): ").strip().lower() == "y"
            front = input("前に障害物がありますか？ (y/n): ").strip().lower() == "y"
            right = input("右に障害物がありますか？ (y/n): ").strip().lower() == "y"
        else:
            left = random.choice([True, False])
            front = random.choice([True, False])
            right = random.choice([True, False])

        self.obstacle_map.append((left, front, right))
        return left, front, right

    def decide_direction(self):
        left, front, right = self.sense_environment()
        direction_str = self.directions[self.dir_index]
        dxdy = {'east': (1, 0), 'south': (0, 1), 'west': (-1, 0), 'north': (0, -1)}

        # 🧠 進行方向を決定
        if self.learned_route:
            direction = self.learned_route.pop(0)
            print(f"🧭 学習ルートを再現中: {direction}")
        else:
            options = []
            weights = []

            if not front:
                options.append("forward")
                weights.append(1.0)
            if not left:
                options.append("left")
                weights.append(1.0)
            if not right:
                options.append("right")
                weights.append(1.0)

            if not options:
                direction = "stop"
            else:
                if self.last_direction in options:
                    i = options.index(self.last_direction)
                    weights[i] *= 0.3
                norm = sum(weights)
                norm_weights = [w / norm for w in weights]
                direction = random.choices(options, weights=norm_weights)[0]
                print(f"🎲 重みづけ選択: {list(zip(options, norm_weights))} → 選ばれた: {direction}")

        # ✅ 移動処理
        if direction == "left":
            self.dir_index = (self.dir_index - 1) % 4
        elif direction == "right":
            self.dir_index = (self.dir_index + 1) % 4
        direction_str = self.directions[self.dir_index]

        if direction == "forward":
            dx, dy = dxdy[direction_str]
            self.x += dx
            self.y += dy
            self.position += 1
            if front:
                print("⚠️ 前に障害物があるのに前進しようとしています！")

        self.last_direction = direction
        self.route.append(direction)

        self.training_data.append({
            "sensor_left": left,
            "sensor_front": front,
            "sensor_right": right,
            "direction": direction
        })

        self.log.append({
            "step": len(self.log) + 1,
            "direction": direction,
            "x": self.x,
            "y": self.y,
            "facing": direction_str,
            "sensor_left": left,
            "sensor_front": front,
            "sensor_right": right
        })

        return direction != "stop"

    def drive_to_goal(self, goal=5, max_steps=20):
        steps = 0
        while self.position < goal and steps < max_steps:
            moved = self.decide_direction()
            if not moved:
                print("停止：全方向に障害物")
            steps += 1

        self.save_log_csv("run_log.csv")
        self.save_training_data_csv()

        if self.position >= goal:
            print("🎉 ゴール到達")
            self.save_learned_route()
        else:
            print("🛑 ゴール未達")

    def save_log_csv(self, filename):
        if not self.log:
            print("⚠ ログが空です。保存されませんでした。")
            return
        with open(filename, "w", newline='', encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=self.log[0].keys())
            writer.writeheader()
            writer.writerows(self.log)
        print(f"📄 ログを {filename} に保存しました！")

    def save_training_data_csv(self, filename="training_data.csv"):
        if not self.training_data:
            print("⚠ トレーニングデータが空です。保存されませんでした。")
            return
        with open(filename, "w", newline='', encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=self.training_data[0].keys())
            writer.writeheader()
            writer.writerows(self.training_data)
        print(f"🧠 トレーニングデータを {filename} に保存しました！")

    def save_learned_route(self, filename="learned_route.txt"):
        with open(filename, "w") as f:
            for d in self.route:
                f.write(d + "\n")
        print(f"🧠 学習ルートを {filename} に保存しました！")

# ルールベース入力の例（左、前、右）
car = Car("RumiCar", manual_mode=True)
car.rule_based_sensor_inputs = [
    (True, False, False),  # 左に障害物あり
    (True, False, True),
    (True, False, True),
    (False, False, False),  # 左が空いた
]
car.drive_to_goal(goal=3, max_steps=10)
