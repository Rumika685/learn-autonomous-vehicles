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

    def sense_environment(self):
        import random
        if self.manual_mode:
            front = input("前に障害物がありますか？ (y/n): ").strip().lower() == "y"
            left = input("左に障害物がありますか？ (y/n): ").strip().lower() == "y"
            right = input("右に障害物がありますか？ (y/n): ").strip().lower() == "y"
        else:
            front = random.choice([True, False])
            left = random.choice([True, False])
            right = random.choice([True, False])
        self.obstacle_map.append((front, left, right))
        return front, left, right

    def decide_direction(self):
        front, left, right = self.sense_environment()
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
            import random
            if self.last_direction in options:
                i = options.index(self.last_direction)
                weights[i] *= 0.3
            norm = sum(weights)
            norm_weights = [w / norm for w in weights]
            direction = random.choices(options, weights=norm_weights)[0]

        # 進行方向の変更
        if direction == "left":
            self.dir_index = (self.dir_index - 1) % 4
        elif direction == "right":
            self.dir_index = (self.dir_index + 1) % 4

        # 位置更新
        dxdy = {'east': (1, 0), 'south': (0, 1), 'west': (-1, 0), 'north': (0, -1)}
        direction_str = self.directions[self.dir_index]

        if direction == "forward":
            dx, dy = dxdy[direction_str]
            self.x += dx
            self.y += dy
            self.position += 1

        # ✅ ログに記録
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
        else:
            print("🛑 ゴール未達")

    def save_log_csv(self, filename):
        with open(filename, mode='w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=self.log[0].keys())
            writer.writeheader()
            writer.writerows(self.log)
        print(f"📄 ログを {filename} に保存しました！")

car = Car("RumiCar")
car.drive_to_goal(goal=3, max_steps=10)
