import csv

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
        self.log = []  # ✅ ログを記録するリスト
        self.learned_route = []
        self.load_learned_route()
        self.training_data = []  # 🧠 センサー＋行動のセット記録

    def load_learned_route(self, filename="learned_route.txt"):
        try:
            with open(filename, "r") as f:
                self.learned_route = [line.strip() for line in f.readlines()]
            print(f"📥 学習済みルートを読み込みました: {self.learned_route}")
        except FileNotFoundError:
            self.learned_route = []
            print("📂 学習ルートファイルが見つかりませんでした。")

    def sense_environment(self):
        if self.test_sensor_inputs:
            front, left, right = self.test_sensor_inputs.pop(0)
            print(f"📦 テスト入力: 前={front}, 左={left}, 右={right}")
        elif self.manual_mode:
            front = input("前に障害物がありますか？ (y/n): ").strip().lower() == "y"
            left = input("左に障害物がありますか？ (y/n): ").strip().lower() == "y"
            right = input("右に障害物がありますか？ (y/n): ").strip().lower() == "y"
        else:
            import random
            front = random.choice([True, False])
            left = random.choice([True, False])
            right = random.choice([True, False])
        
        self.obstacle_map.append((front, left, right))
        return front, left, right

    def decide_direction(self):
        front, left, right = self.sense_environment()
        direction_str = self.directions[self.dir_index]
        dxdy = {'east': (1, 0), 'south': (0, 1), 'west': (-1, 0), 'north': (0, -1)}        

        # 🧠 学習ルートを優先（再現モード）
        if self.learned_route:
            direction = self.learned_route.pop(0)
            print(f"🧭 学習ルートを再現中: {direction}")
            
            # 進行方向の変更
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

        if direction == "forward" and front:
            print("⚠️ 前に障害物があるのに前進しようとしています！")

        self.log.append({
            "step": len(self.log) + 1,
            "direction": direction,
            "x": self.x,
            "y": self.y,
            "facing": direction_str,
            "sensor_front": front,
            "sensor_left": left,
            "sensor_right": right
        })

        self.route.append(direction)
        self.last_direction = direction

        # 🧠 センサーと選択方向をセットで記録
        self.training_data.append({
        "sensor_front": front,
        "sensor_left": left,
        "sensor_right": right,
        "direction": direction
        })

        return direction != "stop"

    def drive_to_goal(self, goal=5, max_steps=20):
        steps = 0
        while self.position < goal and steps < max_steps:
            moved = self.decide_direction()
            if not moved:
                print("停止：全方向に障害物")
            steps += 1

        self.save_log_csv("run_log.csv")  # ✅ CSV保存

        if self.position >= goal:
            print("🎉 ゴール到達")
            self.save_learned_route()
        else:
            print("🛑 ゴール未達")
        self.save_training_data_csv()

    def save_learned_route(self, filename="learned_route.txt"):
        with open(filename, "w") as f:
            for direction in self.route:
                 f.write(direction + "\n")
        print(f"🧠 学習ルートを {filename} に保存しました！")
                
    def save_log_csv(self, filename):
        if not self.log:
            print("⚠ ログが空です。CSVは保存されませんでした。")
            return
        with open(filename, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=self.log[0].keys())
            writer.writeheader()
            writer.writerows(self.log)
        print(f"📄 ログを {filename} に保存しました！")

    def save_training_data_csv(self, filename="training_data.csv"):
        if not self.training_data:
            print("⚠ トレーニングデータが空です。保存されませんでした。")
            return
        with open(filename, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=self.training_data[0].keys())
            writer.writeheader()
            writer.writerows(self.training_data)
        print(f"🧠 トレーニングデータを {filename} に保存しました！")

# テストセンサー入力（front, left, right）
test_inputs = [
    (False, True, True),   # 前OK → 進める
    (True, False, True),   # 左OK → 左へ
    (True, True, False),   # 右OK → 右へ
    (True, True, True),    # 全方向障害物 → 停止
]

car = Car("RumiCar", manual_mode=True)
car.test_sensor_inputs = test_inputs
car.drive_to_goal(goal=3, max_steps=10)
