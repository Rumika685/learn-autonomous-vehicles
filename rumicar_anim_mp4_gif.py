import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
import matplotlib as mpl
mpl.use("Agg")  # GUI描画なしでファイル出力のみ

positions = []
with open("run_log.csv", newline='', encoding="utf-8") as f:
    reader = csv.DictReader(f)
    for row in reader:
        positions.append((int(row["x"]), int(row["y"])))

fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(min(x for x, _ in positions) - 1, max(x for x, _ in positions) + 1)
ax.set_ylim(min(y for _, y in positions) - 1, max(y for _, y in positions) + 1)
ax.grid(True)

trail_line, = ax.plot([], [], 'b--')
current_point, = ax.plot([], [], 'ro')

def update(frame):
    x_vals = [x for x, y in positions[:frame+1]]
    y_vals = [y for x, y in positions[:frame+1]]
    trail_line.set_data(x_vals, y_vals)
    current_point.set_data([x_vals[-1]], [y_vals[-1]])
    return trail_line, current_point

ani = FuncAnimation(fig, update, frames=len(positions), interval=1000, blit=False)

# MP4保存
mp4_writer = FFMpegWriter(fps=1, metadata={"artist": "RumiCar"})
ani.save("rumicar_anim.mp4", writer=mp4_writer)

# GIF保存（代替）
gif_writer = PillowWriter(fps=1)
ani.save("rumicar_anim.gif", writer=gif_writer)

print("🎥 動画保存が完了しました（MP4 & GIF）")