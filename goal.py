import random

class Car:
    def __init__(self, brand):
        self.brand = brand
        self.position = 0  # 前に進んだ歩数

    def sense_environment(self):
        # 仮想センサー情報：Trueは障害物あり、Falseは進める
        front = random.choice([True, False])
        left = random.choice([True, False])
        right = random.choice([True, False])
        return front, left, right

    def decide_direction(self):
        front, left, right = self.sense_environment()

        print(f"{self.brand} のセンサー状態: 前={front}, 左={left}, 右={right}")

        if not front:
            self.position += 1
            print("→ 前に進みました。現在の位置:", self.position)
        elif not left:
            print("→ 左に回避しました（位置は変わらない）")
        elif not right:
            print("→ 右に回避しました（位置は変わらない）")
        else:
            print("→ 停止：すべての方向に障害物")

    def drive_to_goal(self, goal):
        print(f"{self.brand} がゴール（{goal}歩先）を目指して出発！")
        while self.position < goal:
            self.decide_direction()
        print("🎉 ゴールに到達しました！")

# 実行
rumi_car = Car("RumiCar")
rumi_car.drive_to_goal(goal=5)
