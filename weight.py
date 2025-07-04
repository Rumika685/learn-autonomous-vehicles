import random

class Car:
    def __init__(self, brand, manual_mode=False):
        self.brand = brand
        self.position = 0
        self.route = []
        self.manual_mode = manual_mode
        self.last_direction = None  # 前回の進行方向を記録

    def sense_environment(self):
        if self.manual_mode:
            front = input("前に障害物がありますか？ (y/n): ").strip().lower() == "y"
            left = input("左に障害物がありますか？ (y/n): ").strip().lower() == "y"
            right = input("右に障害物がありますか？ (y/n): ").strip().lower() == "y"
        else:
            front = random.choice([True, False])
            left = random.choice([True, False])
            right = random.choice([True, False])
        return front, left, right

    def decide_direction(self):
        front, left, right = self.sense_environment()
        print(f"{self.brand} のセンサー状態: 前={front}, 左={left}, 右={right}")

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
            self.route.append("stop")
            print("→ 停止：すべての方向に障害物")
            return

        # 重み補正：前回と同じ方向の重みを少し下げる
        if self.last_direction in options:
            i = options.index(self.last_direction)
            weights[i] *= 0.3  # 重みを下げる（例：70% 減）

        # 正規化して選択
        total = sum(weights)
        norm_weights = [w / total for w in weights]
        direction = random.choices(options, weights=norm_weights, k=1)[0]

        self.route.append(direction)
        self.last_direction = direction

        if direction == "forward":
            self.position += 1
            print("→ 前に進みました。現在の位置:", self.position)
        else:
            print(f"→ {direction} に回避しました（位置は変わらない）")

    def drive_to_goal(self, goal):
        print(f"{self.brand} がゴール（{goal}歩先）を目指して出発！")
        while self.position < goal:
            self.decide_direction()

        print("🎉 ゴールに到達しました！")
        print("🧭 進行ルート:", self.route)

# 実行（自動モード）
rumi_car = Car("RumiCar", manual_mode=False)
rumi_car.drive_to_goal(goal=5)
