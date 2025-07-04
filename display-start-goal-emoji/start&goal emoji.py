import random

class Car:
    def __init__(self, brand, manual_mode=False):
        self.brand = brand
        self.position = 0
        self.route = []
        self.obstacle_map = []  # ✨ 障害物を記録するためのリスト
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
            self.route.append("stop")
            print("→ 停止：すべての方向に障害物")
            return False

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
        return True

    def drive_to_goal(self, goal, max_steps=20):
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


# ⭐ ここで Car を実行してから
rumi_car = Car("RumiCar")
rumi_car.drive_to_goal(goal=5, max_steps=20)

# ⭐ ここで可視化を行う！
directions = ['east', 'south', 'west', 'north']
arrow_map = {'east': '→', 'south': '↓', 'west': '←', 'north': '↑'}
current_dir_index = 0
output = ""

for move, (front, left, right) in zip(rumi_car.route, rumi_car.obstacle_map):
    if move == 'left':
        current_dir_index = (current_dir_index - 1) % 4
    elif move == 'right':
        current_dir_index = (current_dir_index + 1) % 4
    elif move == 'forward':
        dir_now = directions[current_dir_index]
        output += arrow_map[dir_now]

    obstacles = []
    if front:
        obstacles.append("前✕")
    if left:
        obstacles.append("左✕")
    if right:
        obstacles.append("右✕")
    if obstacles:
        output += "（" + ",".join(obstacles) + "）"

# 🚗 を先頭に、🏁 を末尾に付ける
if output:
    output = "🚗" + output[:-1] + output[-1] + "🏁" if len(output) >= 1 else "🚗🏁"

print("\n📍 マップ表示:")
print(output)
