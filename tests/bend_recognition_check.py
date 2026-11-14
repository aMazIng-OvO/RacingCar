import math

H = 120
W = 188
CENTER = 94
EXP_ROAD_COUNT = 100
MERGE_WIDTH_MIN = 12
MERGE_WIDTH_MAX = 130
MERGE_ROW_TOL = 15
CENTER_LINE_DISTANCE = 18
BEND_FAR_AHEAD = 12
BEND_FEED_DIV = 2
BEND_STEER_PCT = 115
DEVICE_H = 18


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def steer_angle(line, pos):
    """纯跟踪：返回图像角度（度），用于比较前馈前后的转向时机。"""
    dx = H - line[pos][0] + DEVICE_H
    dy = CENTER - line[pos][1]
    return math.degrees(math.atan2(2 * DEVICE_H * dy, dx * dx + dy * dy))


def steer_angle_feed(line, pos, boost=True):
    """当前实现：远端趋势前馈叠加进横向偏差，弯道状态再按百分比放大转角。
    boost=False 用于对比只看前馈、不含弯道增益的结果。"""
    pos = clamp(pos, 2, len(line) - 1)
    if len(line) > BEND_FAR_AHEAD + 3:
        pos = clamp(pos, 2, len(line) - 1 - BEND_FAR_AHEAD)
    dy_aim = CENTER - line[pos][1]
    far = clamp(pos + BEND_FAR_AHEAD, pos, len(line) - 1)
    dy_far = CENTER - line[far][1]
    dy = dy_aim + (dy_far - dy_aim) // BEND_FEED_DIV
    dx = H - line[pos][0] + DEVICE_H
    deg = math.degrees(math.atan2(2 * DEVICE_H * dy, dx * dx + dy * dy))
    if boost:
        deg = clamp(deg * BEND_STEER_PCT / 100, -30, 30)
    return deg


def make_line(n, bend_at=999, direction=0, slope=2):
    """中线点列：行号从 119 递减（间距 2），列号在 bend_at 之后按 direction 平移。"""
    out = []
    for i in range(n):
        x = max(0, 119 - 2 * i)
        y = CENTER
        if i >= bend_at:
            y = min(W - 1, CENTER + direction * slope * (i - bend_at))
        out.append((x, y))
    return out


LAST_HALF_W = 0


def build_center_line(ml, mr):
    """复现 main.c 的 buildCenterLine，含跨帧的实测半宽记忆。"""
    global LAST_HALF_W
    left_is_skel = len(ml) >= len(mr)
    skel = ml if left_is_skel else mr
    other = mr if left_is_skel else ml
    out = []
    j = 0
    half_w = LAST_HALF_W if LAST_HALF_W else CENTER_LINE_DISTANCE
    for sx, sy in skel:
        best, best_d = -1, 0x7FFF
        while j + 1 < len(other) and other[j][0] > sx:
            j += 1
        for t in range(j - 2, j + 3):
            if t < 0 or t >= len(other):
                continue
            d = abs(other[t][0] - sx)
            if d < best_d:
                best_d, best = d, t
        paired = False
        if best >= 0 and len(other) > 0 and best_d <= MERGE_ROW_TOL:
            yl = sy if left_is_skel else other[best][1]
            yr = other[best][1] if left_is_skel else sy
            width = yr - yl
            if MERGE_WIDTH_MIN <= width <= MERGE_WIDTH_MAX:
                half_w = clamp((width + 1) >> 1, 8, 60)
                LAST_HALF_W = ((LAST_HALF_W * 3 + half_w + 2) >> 2) if LAST_HALF_W else half_w
                out.append((sx, (yl + yr) >> 1))
                paired = True
        if not paired:
            dy = sy + (half_w if left_is_skel else -half_w)
            out.append((sx, clamp(dy, 0, W - 1)))
    return out


print("=== 1. 弯道前馈：车尚未进弯时是否已有转向角 ===")
straight = make_line(45)
right_bend = make_line(45, bend_at=18, direction=1, slope=2)
left_bend = make_line(45, bend_at=18, direction=-1, slope=2)

print("场景                  前瞻点  原实现   仅前馈   前馈+弯道增益  结论")
for name, line in (("直道", straight), ("右弯（前瞻点仍在直道段）", right_bend),
                   ("左弯（前瞻点仍在直道段）", left_bend)):
    pos = 14
    old = steer_angle(line, pos)
    feed = steer_angle_feed(line, pos, boost=False)
    new = steer_angle_feed(line, pos)
    note = "弯道增益再放大转向" if abs(new) > abs(feed) + 0.05 else "直道不受影响"
    print("%-24s %3d   %+7.2f   %+7.2f      %+7.2f      %s"
          % (name, pos, old, feed, new, note))

print()
print("=== 2. 弯道内各前瞻点的转向角（右弯，弯起点序号 18）===")
print("前瞻点  该点列号  远端列号  原实现   仅前馈   前馈+弯道增益")
for pos in (6, 12, 18, 24, 30, 36):
    far = clamp(pos + BEND_FAR_AHEAD, pos, len(right_bend) - 1)
    print("  %3d      %3d        %3d      %+7.2f   %+7.2f      %+7.2f"
          % (pos, right_bend[pos][1], right_bend[far][1],
             steer_angle(right_bend, pos),
             steer_angle_feed(right_bend, pos, boost=False),
             steer_angle_feed(right_bend, pos)))

print()
print("=== 3. 中线融合：弯道外侧边界断裂时点数是否保持 ===")
left = [(119 - 2 * i, 60 + i // 2) for i in range(40)]
right = [(119 - 2 * i, 120 + i // 2) for i in range(40)]

LAST_HALF_W = 0
cases = [
    ("两侧各 40 点", left, right),
    ("左侧 40 点、右侧 8 点（外侧断裂）", left, right[:8]),
    ("左侧 8 点、右侧 40 点（内侧断裂）", left[:8], right),
    ("左侧 0 点、右侧 40 点", [], right),
    ("左侧 40 点、右侧 0 点", left, []),
]
for name, ml, mr in cases:
    out = build_center_line(ml, mr)
    mid_y = out[len(out) // 2][1] if out else None
    print("  %-36s 输出 %2d 点  中点列号 %s" % (name, len(out), mid_y))

print()
print("=== 4. 拐点截断后中线可用的前瞻范围 ===")
full_l = [(119 - 2 * i, 60) for i in range(40)]
full_r = [(119 - 2 * i, 120) for i in range(40)]
merged = build_center_line(full_l, full_r)
corner = 6
print("  参考实现：拐点序号 %d 会把全局点数裁到 %d，前瞻点最大只能取 %d"
      % (corner, corner, corner - 2))
print("  当前实现：全局点数仍裁到 %d 供元素判据，中线保持 %d 点，前瞻点可取到 %d"
      % (corner, len(merged), len(merged) - 1))
print("  前瞻点序号 24 对应中线列号:", merged[24][1] if len(merged) > 24 else "不可用")

print()
print("=== 5. 跨帧半宽记忆：先双侧配对，下一帧只剩单侧 ===")
LAST_HALF_W = 0
build_center_line(left, right)
print("  第 1 帧双侧配对，实测半宽:", (right[0][1] - left[0][1] + 1) >> 1, " 记忆值:", LAST_HALF_W)
only_left = build_center_line(left, [])
only_right = build_center_line([], right)
print("  第 2 帧只有左边界（列 60），中线首点:", only_left[0][1], "（应约为 60 + 30 = 90）")
print("  第 2 帧只有右边界（列 120），中线首点:", only_right[0][1], "（应约为 120 - 30 = 90）")

print()
print("=== 6. 上电第一帧没有历史时的保守值 ===")
LAST_HALF_W = 0
cold = build_center_line(left, [])
print("  无历史且只有左边界，中线首点:", cold[0][1], "（用 centerLineDistance = 18，得 78）")
