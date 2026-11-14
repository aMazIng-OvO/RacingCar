import math

H = 120
CENTER = 94
DEVICE_H = 18
C4 = 2000
C5 = 400
SPEED_LIMIT = 400

OLD_BORDER_RATIO = 70
BORDER_IN_SPEED_RATIO = 50
BEND_STEER_PCT = 115
BEND_SLOW_START = 4
BEND_SLOW_GAIN = 3
BEND_SLOW_MIN = 40
BEND_SPEED_CAP = 200
BEND_RELEASE = 2
BEND_FEED_DIV = 2


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def tani(d):
    return int(round(math.tan(math.radians(abs(d))) * 1000))


def curve_speed(d):
    t = tani(d)
    r = 0 if t == 0 else 20000 // t
    if r == 0:
        return C5
    return min(int(math.sqrt(r * C4)), C5)


def angle_from_dev(dy, dx):
    return int(math.degrees(math.atan2(2 * DEVICE_H * dy, dx * dx + dy * dy)))


def bend_brake(dy_aim, dy_far, d, hold=100):
    """返回 (生效比例, 弯道强度, 新的保持值)。hold 记住本次弯道最严的比例。"""
    bend = abs(dy_far - dy_aim) + abs(d)
    if bend <= BEND_RELEASE:
        return 100, bend, 100
    if bend > BEND_SLOW_START:
        ratio = clamp(100 - (bend - BEND_SLOW_START) * BEND_SLOW_GAIN,
                      BEND_SLOW_MIN, 100)
        if ratio < hold:
            hold = ratio
    return hold, bend, hold


def old_target(dy_aim, border_in):
    d = angle_from_dev(dy_aim, 60)
    target = curve_speed(d)
    if border_in:
        target = target * OLD_BORDER_RATIO // 100
    return min(target, SPEED_LIMIT)


def new_target(dy_aim, dy_far, border_in, cap=BEND_SPEED_CAP, hold=100):
    dy = dy_aim + (dy_far - dy_aim) // BEND_FEED_DIV
    d = angle_from_dev(dy, 60)
    ratio, bend, hold = bend_brake(dy_aim, dy_far, d, hold)
    slow = min(ratio, BORDER_IN_SPEED_RATIO) if border_in else ratio
    # 弯道状态放大转角，放大后的角度同时用于曲率限速的 tan 表
    d_servo = clamp(d * BEND_STEER_PCT // 100, -30, 30) if slow < 100 else d
    target = curve_speed(d_servo)
    if slow < 100:
        target = min(target * slow // 100, cap)
    return min(target, SPEED_LIMIT), ratio, bend, d_servo, hold


def prev_target(dy_aim, dy_far, border_in):
    """上一版参数：FEED_DIV=3、START=6、GAIN=2、MIN=55、CAP=260、BORDER=60、无转向增益。"""
    dy = dy_aim + (dy_far - dy_aim) // 3
    d = angle_from_dev(dy, 60)
    bend = abs(dy_far - dy_aim) + abs(d)
    ratio = 100
    if bend > 6:
        ratio = clamp(100 - (bend - 6) * 2, 55, 100)
    slow = min(ratio, 60) if border_in else ratio
    target = curve_speed(d)
    if slow < 100:
        target = min(target * slow // 100, 260)
    return min(target, SPEED_LIMIT)


def report(name, dy_aim, dy_far, border_in):
    old = old_target(dy_aim, border_in)
    prev = prev_target(dy_aim, dy_far, border_in)
    new, ratio, bend, d, _ = new_target(dy_aim, dy_far, border_in)
    print("%-28s bend=%2d 比例=%3d%% 角度=%3d  原=%3d 上一版=%3d 当前=%3d  比上一版再降=%3d%%"
          % (name, bend, ratio, d, old, prev, new,
             (prev - new) * 100 // prev if prev else 0))


print("=== 1. 各阶段的目标速度（c4=2000 c5=400，单位 脉冲/5 ms）===")
print("场景                         bend 比例 角度  原  上一版  当前  比上一版再降")
report("直道", 0, 0, False)
report("远端有弯，前瞻点仍在直道", 0, -16, False)
report("更急的入弯（远端偏 28）", 0, -28, False)
report("弯中，前瞻点已偏 12", -12, -28, False)
report("车头已进弯 borderIn=1", -12, -28, True)
report("急弯，前瞻点偏 30", -30, -46, True)
report("弯心，两端都偏", -36, -42, True)

print()
print("=== 2. 减速触发与保持（逐帧推进：直道 → 入弯 → 弯心 → 出弯）===")
print("帧  前瞻点偏差  远端偏差  bend  比例  保持  原实现  当前")
frames = [(0, 0), (0, -4), (0, -10), (0, -16), (-4, -22), (-10, -28),
          (-18, -34), (-26, -40), (-32, -42), (-36, -42),
          (-20, -24), (-4, -6), (0, 0), (0, 0)]
hold = 100
for n, (aim, far) in enumerate(frames):
    old = old_target(aim, False)
    new, ratio, bend, d, hold = new_target(aim, far, False, hold=hold)
    print(" %2d    %+4d       %+4d     %3d   %3d   %3d   %3d    %3d"
          % (n + 1, aim, far, bend, ratio, hold, old, new))

print()
print("=== 3. BEND_SPEED_CAP 的作用（入弯初期比例仍高时）===")
aim, far = 0, -16
ratio, bend, _ = bend_brake(aim, far, angle_from_dev((far - aim) // BEND_FEED_DIV, 60))
base = curve_speed(angle_from_dev((far - aim) // BEND_FEED_DIV, 60))
scaled = base * ratio // 100
print("  弯道前基础目标 %d，按比例 %d%% 得 %d，cap %d 生效后 %d"
      % (base, ratio, scaled, BEND_SPEED_CAP, min(scaled, BEND_SPEED_CAP)))
print("  没有 cap 时入弯初期仍会跑 %d，与直道 %d 接近" % (scaled, C5))

print()
print("=== 4. 两个减速条件取更严格值，不连乘 ===")
aim, far = -12, -28
ratio, bend, _ = bend_brake(aim, far, angle_from_dev(aim + (far - aim) // BEND_FEED_DIV, 60))
slow = min(ratio, BORDER_IN_SPEED_RATIO)
print("  弯道比例 %d%%  borderIn 比例 %d%%  取 %d%%" % (ratio, BORDER_IN_SPEED_RATIO, slow))
print("  连乘会得到 %d%%，比设定值慢得多" % (ratio * BORDER_IN_SPEED_RATIO // 100))

print()
print("=== 5. 参数扫描：BEND_SPEED_CAP 改变入弯初期的限幅 ===")
print("  cap   入弯初期目标   急弯目标(borderIn=1)")
for cap in (160, 180, 200, 240, 300):
    a, _, _, _, _ = new_target(0, -16, False, cap=cap)
    b, _, _, _, _ = new_target(-30, -46, True, cap=cap)
    print("  %3d      %3d            %3d" % (cap, a, b))

print()
print("=== 6. 出弯后减速保持是否解除 ===")
hold = 100
seq = [(0, -16), (-30, -46), (-36, -42), (-4, -6), (0, -1), (0, 0)]
print("  帧  前瞻偏差  远端偏差  bend  生效比例  保持值  目标速度")
for n, (aim, far) in enumerate(seq):
    v, ratio, bend, _, hold = new_target(aim, far, False, hold=hold)
    print("  %2d    %+4d      %+4d     %3d     %3d      %3d      %3d"
          % (n + 1, aim, far, bend, ratio, hold, v))
print("  出弯后比例回到:", hold, "（应为 100）")

print()
print("=== 7. 弯道转向增强：角度、舵机 duty 与曲率限速 ===")
print("场景                    增强前角度  增强后角度  duty 前  duty 后  基础限速前  后")
for name, aim, far in (("入弯初期", 0, -16), ("弯中", -12, -28),
                       ("急弯", -30, -46), ("弯心", -36, -42)):
    dy = aim + (far - aim) // BEND_FEED_DIV
    d0 = angle_from_dev(dy, 60)
    d1 = clamp(d0 * BEND_STEER_PCT // 100, -30, 30)
    duty0 = clamp(d0 * 260 // 20, -395, 395)
    duty1 = clamp(d1 * 260 // 20, -395, 395)
    print("  %-20s  %+5d       %+5d      %+5d    %+5d      %3d      %3d"
          % (name, d0, d1, duty0, duty1, curve_speed(d0), curve_speed(d1)))
print("  直道角度为 0，增益不生效，duty 与限速均不变")
