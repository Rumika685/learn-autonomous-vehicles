import random
import time
import winsound

class Car:
    def __init__(self, brand):
        self.brand = brand
        self.speed = 0

    def drive(self):
        print(f"{self.brand} を発進します")

        for _ in range(5):
            self.speed = random.randint(20, 40)
            print(f"{self.brand} は {self.speed} km/h で走行中")
            time.sleep(1)

            # 擬似センサー：ランダムで障害物を検出
            obstacle = random.choice([True, False, False])
            if obstacle:
                print("⚠️ 障害物を検知しました！")
                winsound.PlaySound("warning.wav", winsound.SND_FILENAME)
                self.stop()
                self.reverse()
                break

    def stop(self):
        self.speed = 0
        print(f"{self.brand} は停止中")

    def reverse(self):
        print(f"{self.brand} は後退しています")
        for speed in range(10, 0, -5):
            print(f"{self.brand} は -{speed} km/h で後退中")
            time.sleep(1)
        print(f"{self.brand} は完全に停止しました")

rumi_car = Car("RumiCar")
rumi_car.drive()
