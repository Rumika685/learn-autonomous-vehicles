import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)

point, = ax.plot([], [], 'ro')  # 初期プロット（赤い点）

def update(frame):
    x = frame
    y = frame
    point.set_data([x], [y])  # 🔧 [x], [y] でリストにして渡す
    return point,

ani = FuncAnimation(fig, update, frames=range(10), interval=500)
plt.show()
