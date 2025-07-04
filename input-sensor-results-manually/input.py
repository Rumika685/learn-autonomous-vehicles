class Car:
    def __init__(self, brand, manual_mode=False):
        self.brand = brand
        self.position = 0
        self.route = []
        self.manual_mode = manual_mode

    def sense_environment(self):
        if self.manual_mode:
            # 手動で入力するモード
            front = input("前に障害物がありますか？ (y/n): ").strip().lower() == "y"
            left = input("左に障害物がありますか？ (y/n): ").strip().lower() == "y"
            right = input("右に障害物がありますか？ (y/n): ").strip().lower() == "y"
        else:
            # 自動（ランダム）モード
            import random
            front = random.choice([True, False])
            left = random.choice([True, False])
            right = random.choice([True, False])
        return front, left, right

    def decide_direction(self):
        front, left, right = self.sense_environment()
        print(f"{self.brand} のセンサー状態: 前={front}, 左={left}, 右={right}")

        if not front:
            self.position += 1
            self.route.append("forward")
            print("→ 前に進みました。現在の位置:", self.position)
        elif not left:
            self.route.append("left")
            print("→ 左に回避しました（位置は変わらない）")
        elif not right:
            self.route.append("right")
            print("→ 右に回避しました（位置は変わらない）")
        else:
            self.route.append("stop")
            print("→ 停止：すべての方向に障害物")

    def drive_to_goal(self, goal):
        print(f"{self.brand} がゴール（{goal}歩先）を目指して出発！")
        while self.position < goal:
            self.decide_direction()

        print("🎉 ゴールに到達しました！")
        print("🧭 進行ルート:", self.route)

# 実行（manual_mode=True で手動入力）
rumi_car = Car("RumiCar", manual_mode=True)
rumi_car.drive_to_goal(goal=3)
