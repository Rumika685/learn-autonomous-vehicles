import numpy as np
import matplotlib.pyplot as plt
import random

GRID_SIZE = 10

class Car:
    def __init__(self, pos=(5,5), heading=np.array([0,-1])):
        self.pos = np.array(pos)
        self.heading = heading / np.linalg.norm(heading)  # 正規化
        self.detected = []

    def sense_obstacles(self, obstacles):
        """障害物を左右・前後に分類"""
        self.detected = []
        for obs in obstacles:
            vec = np.array(obs) - self.pos
            dot = np.dot(self.heading, vec)
            cross = np.cross(self.heading, vec)

            if dot > 0:
                direction = "Front"
            else:
                direction = "Back"

            if cross > 0:
                side = "Left"
            elif cross < 0:
                side = "Right"
            else:
                side = "Center"

            self.detected.append((obs, direction, side))
        return self.detected

# -------------------------
# テスト実行
# -------------------------
def main():
    car = Car(pos=(5,5), heading=np.array([0,-1]))  # 北向き

    # ランダム障害物を配置
    obstacles = [(random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1)) for _ in range(6)]

    results = car.sense_obstacles(obstacles)

    # 結果を表示
    for obs, direction, side in results:
        print(f"Obstacle at {obs} → {direction}-{side}")

    # 可視化
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(0, GRID_SIZE)
    ax.set_ylim(0, GRID_SIZE)
    ax.grid(True)

    # 車
    ax.plot(car.pos[0], car.pos[1], "ro")
    ax.arrow(car.pos[0], car.pos[1], car.heading[0], car.heading[1],
             head_width=0.2, color="red", length_includes_head=True)

    # 障害物
    for obs, d, s in results:
        ax.plot(obs[0], obs[1], "bx")
        ax.text(obs[0]+0.1, obs[1]+0.1, f"{d}-{s}", fontsize=8)

    plt.title("Car Sensor Classification (Dot & Cross)")
    plt.show()

if __name__ == "__main__":
    main()