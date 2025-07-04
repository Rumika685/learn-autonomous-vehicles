import random

class Car:
    def __init__(self, brand, manual_mode=False):
        self.brand = brand
        self.position = 0
        self.route = []
        self.manual_mode = manual_mode
        self.last_direction = None

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
            return False  # 進めなかった

        if self.last_direction in options:
            i = options.index(self.last_direction)
            weights[i] *= 0.3

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
        return True  # 進めた

    def drive_to_goal(self, goal, max_steps=30):
        print(f"{self.brand} がゴール（{goal}歩先）を目指して出発！")
        steps = 0

        while self.position < goal:
            moved = self.decide_direction()
            steps += 1

            if steps >= max_steps:
                print("⚠ 最大試行回数に達しました。進行を終了します。")
                break

        if self.position >= goal:
            print("🎉 ゴールに到達しました！")
        else:
            print("🛑 ゴールに到達できませんでした。")

        print("🧭 進行ルート:", self.route)

# 実行例（自動モード）
rumi_car = Car("RumiCar", manual_mode=False)
rumi_car.drive_to_goal(goal=5, max_steps=20)
# これまで通りのルート記録
route = ['forward', 'left', 'forward', 'right', 'forward']

# 可視化パート ↓↓↓
directions = ['east', 'south', 'west', 'north']
current_dir_index = 0  # start facing east
arrow_map = {'east': '→', 'south': '↓', 'west': '←', 'north': '↑'}

output = ""
for move in route:
    if move == 'left':
        current_dir_index = (current_dir_index - 1) % 4
    elif move == 'right':
        current_dir_index = (current_dir_index + 1) % 4
    elif move == 'forward':
        dir_now = directions[current_dir_index]
        output += arrow_map[dir_now]

print(output)
