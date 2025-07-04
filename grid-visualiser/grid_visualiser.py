import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import matplotlib

matplotlib.rcParams['font.family'] = 'Segoe UI Emoji'

# マップサイズ
width, height = 5, 5

# サンプルデータ
route = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (3, 2), (4, 2)]
start = route[0]
goal = route[-1]
obstacles = [(1, 1), (3, 1)]

# 図のセットアップ
fig, ax = plt.subplots()
ax.set_xlim(-0.5, width - 0.5)
ax.set_ylim(-0.5, height - 0.5)
ax.set_xticks(range(width))
ax.set_yticks(range(height))
ax.invert_yaxis()
ax.set_aspect('equal')
ax.grid(True)

# 画像の読み込み
car_img = mpimg.imread('car_right.png')
pylon_img = mpimg.imread('pylon.png')

# 画像を描画可能な形式に変換
car_icon = OffsetImage(car_img, zoom=0.08)
pylon_icon = OffsetImage(pylon_img, zoom=0.08)

# 🎯 スタート地点に車画像を表示
car_ab = AnnotationBbox(car_icon, start, frameon=False)
ax.add_artist(car_ab)

# 🪧 障害物をすべて配置
for ox, oy in obstacles:
    ab = AnnotationBbox(pylon_icon, (ox, oy), frameon=False)
    ax.add_artist(ab)

# 🏁 ゴール
ax.text(goal[0], goal[1], '🏁', ha='center', va='center', fontsize=18)

# • ルート表示（通過点）
for (x, y) in route:
    if (x, y) != start and (x, y) != goal:  # スタート・ゴールには上書きしない
        ax.text(x, y, '•', ha='center', va='center', fontsize=14, color='blue')

plt.title("RumiCar Route Visualization")
plt.show()
