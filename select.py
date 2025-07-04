import random

class Car:
    def __init__(self, brand):
        self.brand = brand

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
            print("→ 前方に進みます")
        elif not left:
            print("→ 左に回避します")
        elif not right:
            print("→ 右に回避します")
        else:
            print("→ 停止（すべての方向に障害物）")

# 実行
rumi_car = Car("RumiCar")
rumi_car.decide_direction()
