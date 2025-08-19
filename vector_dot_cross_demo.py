import numpy as np
import matplotlib.pyplot as plt

# ベクトル定義
a = np.array([2, 1])
b = np.array([1, 3])

# 内積と外積
dot = np.dot(a, b)
cross = np.cross(a, b)  # 2Dではスカラー
angle = np.degrees(np.arccos(dot / (np.linalg.norm(a)*np.linalg.norm(b))))

print(f"a·b (dot) = {dot}")
print(f"a×b (cross) = {cross}")
print(f"angle between a and b = {angle:.2f} degrees")

# プロット
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(-1, 4)
ax.set_ylim(-1, 4)
ax.grid(True)

# ベクトル描画
ax.arrow(0, 0, a[0], a[1], head_width=0.1, color="blue", length_includes_head=True)
ax.arrow(0, 0, b[0], b[1], head_width=0.1, color="green", length_includes_head=True)

ax.text(a[0], a[1], "a", color="blue")
ax.text(b[0], b[1], "b", color="green")

ax.set_title(f"Dot={dot}, Cross={cross}, angle={angle:.1f}°")
plt.show()