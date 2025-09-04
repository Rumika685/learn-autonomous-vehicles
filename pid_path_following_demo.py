import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# =========================
# Config (feel free to tune)
# =========================
DT = 0.05                 # [s] sim time step
V  = 1.2                  # [m/s] constant forward speed
LOOKAHEAD_DIST = 0.6      # [m] waypoint切替のしきい値
GOAL_TOL = 0.5            # [m] ゴール到達判定
MAX_STEER_RATE = 2.5      # [rad/s] 角速度飽和（実機っぽさ）
Kp, Ki, Kd = 2.2, 0.05, 0.25  # PID ゲイン（見やすい初期値）

# =========================
# Utilities
# =========================
def wrap_angle(a):
    """[-pi, pi] に正規化"""
    a = (a + math.pi) % (2*math.pi) - math.pi
    return a

# =========================
# Simple PID
# =========================
class PID:
    def __init__(self, kp, ki, kd, integral_limit=2.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_err = 0.0
        self.int_lim = integral_limit

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

    def __call__(self, err, dt):
        self.integral += err * dt
        # anti-windup
        self.integral = max(-self.int_lim, min(self.integral, self.int_lim))
        deriv = (err - self.prev_err) / dt if dt > 0 else 0.0
        self.prev_err = err
        return self.kp*err + self.ki*self.integral + self.kd*deriv

# =========================
# Vehicle (unicycle model)
# =========================
class Vehicle:
    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.omega = 0.0  # current angular rate

    def step(self, v, omega_cmd, dt):
        # saturate angular rate
        omega_cmd = max(-MAX_STEER_RATE, min(MAX_STEER_RATE, omega_cmd))
        self.omega = omega_cmd
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw = wrap_angle(self.yaw + self.omega * dt)

# =========================
# Path (polyline waypoints)
# =========================
def build_demo_path():
    """
    A*などから来た離散経路を連続空間のwaypointsとみなす想定。
    わかりやすさ優先でS字の例を用意。
    """
    xs = np.linspace(0.5, 9.0, 20)
    ys = 5.0 + 2.0 * np.sin(xs * 0.7)
    wps = np.vstack([xs, ys]).T
    # スタートを右上、ゴールを左下寄りにしたい時は以下のような別パスでもOK
    # wps = np.array([(9.0, 8.5), (8.0, 7.5), (7.0, 7.0), (6.0, 6.0), (5.0, 5.0),
    #                 (4.0, 4.0), (3.0, 3.2), (2.0, 2.2), (1.0, 1.0), (0.6, 0.6)])
    return wps

class PathFollower:
    def __init__(self, waypoints):
        self.wps = waypoints
        self.i = 0  # current target index

    def current_target(self, pos):
        """pos=(x,y) に最も近い or 次の waypoint を返す簡易ロジック"""
        x, y = pos
        # 近づいたら次へ
        while self.i < len(self.wps)-1:
            dx = self.wps[self.i][0] - x
            dy = self.wps[self.i][1] - y
            if math.hypot(dx, dy) < LOOKAHEAD_DIST:
                self.i += 1
            else:
                break
        return self.wps[self.i]

    def reached_goal(self, pos):
        gx, gy = self.wps[-1]
        return math.hypot(pos[0]-gx, pos[1]-gy) < GOAL_TOL

# =========================
# Main (animation)
# =========================
def main():
    wps = build_demo_path()
    follower = PathFollower(wps)
    pid = PID(Kp, Ki, Kd)

    # 初期姿勢：スタート付近を向く
    x0, y0 = wps[0]
    x1, y1 = wps[1]
    yaw0 = math.atan2(y1 - y0, x1 - x0)
    car = Vehicle(x0, y0, yaw0)

    # 可視化準備
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(-0.5, 10.5)
    ax.set_ylim(-0.5, 10.5)
    ax.grid(True)

    # ルートと目印
    ax.plot(wps[:,0], wps[:,1], 'k--', linewidth=1.0, label="reference path")
    ax.text(wps[0,0], wps[0,1], "START", color="green", ha="left", va="bottom")
    ax.text(wps[-1,0], wps[-1,1], "GOAL",  color="blue",  ha="right", va="top")

    # 車の描画物
    (traj_line,) = ax.plot([], [], 'b-', lw=2, label="trajectory")
    (car_point,) = ax.plot([], [], 'ro', ms=6)
    head = ax.arrow(car.x, car.y, 0.6*math.cos(car.yaw), 0.6*math.sin(car.yaw),
                    head_width=0.18, length_includes_head=True, ec='r', fc='r')

    text_box = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", family="monospace")

    xs_hist, ys_hist = [car.x], [car.y]
    done = {"flag": False}  # to freeze the last frame

    def update(_):
        nonlocal head
        if done["flag"]:
            return traj_line, car_point, head, text_box

        # 目標waypointとヘディング誤差
        tx, ty = follower.current_target((car.x, car.y))
        desired_yaw = math.atan2(ty - car.y, tx - car.x)
        err = wrap_angle(desired_yaw - car.yaw)

        # PID で角速度コマンド算出
        omega_cmd = pid(err, DT)

        # 1ステップ前進
        car.step(V, omega_cmd, DT)

        # ログ
        xs_hist.append(car.x)
        ys_hist.append(car.y)

        # 描画更新（矢印は描き直し）
        traj_line.set_data(xs_hist, ys_hist)
        car_point.set_data([car.x], [car.y])

        # 既存の矢印を消して描き直し
        head.remove()
        head = ax.arrow(car.x, car.y, 0.6*math.cos(car.yaw), 0.6*math.sin(car.yaw),
                        head_width=0.18, length_includes_head=True, ec='r', fc='r')

        text_box.set_text(
            f"target=({tx:4.1f},{ty:4.1f})\n"
            f"yaw_err={err:+.2f} rad\n"
            f"omega_cmd={omega_cmd:+.2f} rad/s"
        )

        # ゴール判定
        if follower.reached_goal((car.x, car.y)):
            ax.set_title("Goal reached ✓")
            done["flag"] = True  # フリーズ（表示は保持）
        return traj_line, car_point, head, text_box

    ani = animation.FuncAnimation(fig, update, interval=int(DT*1000), blit=False)
    ax.legend(loc="upper right")
    plt.show()

if __name__ == "__main__":
    main()