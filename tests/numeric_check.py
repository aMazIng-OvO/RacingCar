import math

# 与 zf_common_function.c 完全一致的整数实现
TAN_TABLE = [
    0, 1143, 2288, 3434, 4582, 5732,
    6887, 8046, 9212, 10383, 11562, 12749,
    13945, 15151, 16368, 17597, 18840, 20097,
    21369, 22657, 23962, 25286, 26628, 27989,
    29371, 30775, 32203, 33654, 35132, 36637,
    37837,
]


def atan2_int(y, x):
    sign = 1 if y >= 0 else -1
    ay = y if y >= 0 else -y
    d = 29
    if ay == 0 or x <= 0:
        return 0
    lhs = ay * 65536
    rhs = x * TAN_TABLE[30]
    if lhs >= rhs:
        return sign * 30
    while d >= 0:
        if lhs >= x * TAN_TABLE[d]:
            break
        d -= 1
    return sign * d


def sqrt_int(x):
    res = 0
    bi = 1 << 30
    if x <= 0:
        return 0
    while bi > x:
        bi >>= 2
    while bi:
        if x >= res + bi:
            x -= res + bi
            res = (res >> 1) + bi
        else:
            res >>= 1
        bi >>= 2
    return res


# 与 Motor.c 完全一致的 tan 表（tan(度) × 1000，四舍五入）
TANI = [
    0, 17, 35, 52, 70, 87, 105, 123, 141, 158, 176,
    194, 213, 231, 249, 268, 287, 306, 325, 344, 364,
    384, 404, 424, 445, 466, 488, 510, 532, 554, 577,
    600, 628,
]

DEVICE_H = 18
STEER_DUTY_DIV = 20
STEER_DUTY_LIMIT = 395
SPEED_LIMIT = 400


def along_line_angle(x, y):
    dx = 120 - x + DEVICE_H
    dy = 94 - y
    ax = dx * dx + dy * dy
    ay = 2 * DEVICE_H * dy
    if ax <= 0:
        return 0
    return atan2_int(ay, ax)


def curve_speed(d, c4, c5):
    t = TANI[min(abs(d), 32)]
    r = 0 if t == 0 else 20000 // t
    v = sqrt_int(r * c4) if c4 > 0 else 0
    target = c5 if r == 0 else min(v, c5)
    return target, r, v


def steer_duty(d, c1):
    d = max(-30, min(30, d))
    duty = d * c1 // STEER_DUTY_DIV
    return max(-STEER_DUTY_LIMIT, min(STEER_DUTY_LIMIT, duty))


def merge_center_line(road_l, road_r):
    if not road_l or not road_r:
        return []
    out = []
    j = 0
    rc = len(road_r)
    for lx, ly in road_l:
        while j + 1 < rc and road_r[j][0] > lx:
            j += 1
        rx, ry = road_r[j]
        if ly >= ry:
            continue
        if rx > lx + 15 or rx + 15 < lx:
            continue
        width = ry - ly
        if width < 12 or width > 120:
            continue
        out.append((lx, (ly + ry) >> 1))
    return out


print("=== 1. atan2_int 返回值范围（alongLine 的实际输入域）===")
seen = set()
for x in range(0, 120, 2):
    for y in range(0, 188, 2):
        d = along_line_angle(x, y)
        seen.add(d)
print("d 取值集合:", sorted(seen))
print("d 绝对值最大:", max(abs(v) for v in seen))

print()
print("=== 2. tani 表与 math.tan 的偏差（应当小于 2）===")
worst = 0
for i, v in enumerate(TANI):
    ref = round(math.tan(math.radians(i)) * 1000)
    worst = max(worst, abs(ref - v))
print("最大偏差:", worst, "（1 度 ~ 32 度）")

print()
print("=== 3. 曲率限速曲线：c4 = 2000, c5 = 400 ===")
for d in (0, 5, 10, 15, 20, 25, 30):
    target, r, v = curve_speed(d, 2000, 400)
    print("d=%2d  r=%4d  sqrt(R*c4)=%4d  target=%4d" % (d, r, v, min(target, SPEED_LIMIT)))

print()
print("=== 4. 曲率限速曲线：c4 = 2400, c5 = 400 ===")
for d in (0, 5, 10, 15, 20, 25, 30):
    target, r, v = curve_speed(d, 2400, 400)
    print("d=%2d  r=%4d  sqrt(R*c4)=%4d  target=%4d" % (d, r, v, min(target, SPEED_LIMIT)))

print()
print("=== 5. 低速档 c5 = 300 时的实际目标 ===")
for d in (0, 10, 20, 30):
    target, r, v = curve_speed(d, 2000, 300)
    print("d=%2d  target=%4d" % (d, min(target, SPEED_LIMIT)))

print()
print("=== 6. 舵机 duty 与等效转角（c1 = 260, 每 duty 0.09 度）===")
for d in (0, 5, 10, 15, 20, 25, 30):
    duty = steer_duty(d, 260)
    print("d=%2d  duty=%4d  舵机约 %5.1f 度" % (d, duty, duty * 0.09))

print()
print("=== 7. 舵机增益 c1 = 200 / 260 / 300 的饱和点 ===")
for c1 in (200, 260, 300):
    sat = None
    for d in range(0, 31):
        if abs(steer_duty(d, c1)) >= STEER_DUTY_LIMIT:
            sat = d
            break
    print("c1=%3d  在 d=%s 处达到机械限位" % (c1, sat))

print()
print("=== 8. mergeCenterLine 在典型双边界下的输出（赛道宽 36 像素）===")
left = [(110 - 2 * i, 60) for i in range(35)]
right = [(110 - 2 * i, 96) for i in range(35)]
merged = merge_center_line(left, right)
print("左右各 35 点，融合得到", len(merged), "点")
if merged:
    print("首点:", merged[0], " 末点:", merged[-1])
    print("中线列号集合:", sorted(set(p[1] for p in merged)))

print()
print("=== 9. 弯道：左右边界列号同步平移，宽度不变 ===")
left = [(110 - 2 * i, 60 + i // 2) for i in range(35)]
right = [(110 - 2 * i, 96 + i // 2) for i in range(35)]
merged = merge_center_line(left, right)
print("融合得到", len(merged), "点")
if merged:
    print("首点:", merged[0], " 末点:", merged[-1], "（列号应随弯道同步平移）")

print()
print("=== 10. 右边界只剩近处 1 点时的融合结果 ===")
merged = merge_center_line(left, [(110, 96)])
print("融合得到", len(merged), "点；左边界", len(left), "点")
print("判定「centerLineCount < 10 或 2*centerLineCount < LRoadCount」:",
      len(merged) < 10 or len(merged) * 2 < len(left))

print()
print("=== 10. sqrt_int 与参考值 ===")
for x in (1, 100, 10000, 1000000, 5292000, 80000000):
    got = sqrt_int(x)
    print("x=%9d  sqrt_int=%6d  参考=%8.2f" % (x, got, math.sqrt(x)))
