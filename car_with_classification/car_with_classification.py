import numpy as np

class Car:
    def __init__(self, start=(5, 5), heading=(1, 0)):
        self.pos = np.array(start, dtype=float)
        self.heading = np.array(heading, dtype=float)  # 進行方向ベクトル
        self.heading /= np.linalg.norm(self.heading)  # 正規化

    def classify_obstacle(self, obstacle):
        """
        障害物の位置を self.heading に基づいて Front/Back/Left/Right に分類
        """
        vec = np.array(obstacle, dtype=float) - self.pos
        dot = np.dot(self.heading, vec)
        cross = np.cross(self.heading, vec)

        eps = 1e-6  # 真横判定用の閾値

        if abs(dot) < eps:
            # 真横 → Left / Right のみ
            if cross > 0:
                return "Left"
            elif cross < 0:
                return "Right"
            else:
                return "Center"  # 原点に障害物がある特殊ケース
        else:
            # 前後方向の判定
            if dot > 0:
                direction = "Front"
            else:
                direction = "Back"

            if cross > 0:
                return f"{direction}-Left"
            elif cross < 0:
                return f"{direction}-Right"
            else:
                return direction

# --- 動作確認 ---
if __name__ == "__main__":
    car = Car(start=(5, 5), heading=(1, 0))  # 右向き

    obstacles = [(7, 5), (7, 3), (10, 5), (5, 5), (3, 7)]
    for obs in obstacles:
        label = car.classify_obstacle(obs)
        print(f"Obstacle at {obs}: {label}")
