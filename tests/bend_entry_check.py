"""弯道入口过渡区识别：状态机、出界宽容与转角保持的数值核对。"""

STRAIGHT, APPROACH, COMMITTED, RECOVER = 0, 1, 2, 3
NAMES = {STRAIGHT: "STRAIGHT", APPROACH: "APPROACH", COMMITTED: "COMMITTED", RECOVER: "RECOVER"}

BE_BEND_MIN = 10
BE_BEND_EXIT = 4
BE_CONFIRM_FRAMES = 2
BE_BLANK_MS = 250
BE_MAX_LOST_MS = 900
BE_RECOVER_POINTS = 15
BE_MIN_POINTS = 8
BE_TURN_DECAY = 1
BE_TURN_MIN = 2


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class BendEntry:
    def __init__(self):
        self.state = STRAIGHT
        self.same = 0
        self.last = 0
        self.hold = 0
        self.state_ms = 0
        self.lost_ms = 0

    def update(self, ms, bend, d, lc, rc):
        one_lost = lc <= 0 or rc <= 0
        both_lost = lc <= 0 and rc <= 0

        if self.state == STRAIGHT:
            sign = 1 if d > 0 else (-1 if d < 0 else 0)
            if bend >= BE_BEND_MIN and abs(d) >= BE_TURN_MIN:
                if self.same and sign and sign != (1 if self.last > 0 else -1):
                    self.same = 0
                self.same += 1
                if self.same >= BE_CONFIRM_FRAMES:
                    self.state = APPROACH
                    self.state_ms = ms
                    self.same = 0
            else:
                self.same = 0

        elif self.state == APPROACH:
            if one_lost:
                self.state = COMMITTED
                self.state_ms = ms
                self.lost_ms = 0
                if abs(d) >= BE_TURN_MIN:
                    self.hold = d
            elif bend < BE_BEND_EXIT:
                self.state = STRAIGHT

        elif self.state == COMMITTED:
            self.lost_ms = ms - self.state_ms
            if not one_lost and lc > BE_RECOVER_POINTS and rc > BE_RECOVER_POINTS:
                self.state = RECOVER
                self.state_ms = ms
            elif self.lost_ms > BE_MAX_LOST_MS:
                self.state = RECOVER
                self.state_ms = ms

        elif self.state == RECOVER:
            if bend >= BE_BEND_MIN:
                # 连续弯道：还没回到直道就又看到弯，直接重新确认
                self.state = APPROACH
                self.state_ms = ms
            elif bend < BE_BEND_EXIT and not one_lost:
                self.state = STRAIGHT
                self.hold = 0
            elif ms - self.state_ms > BE_MAX_LOST_MS:
                self.state = STRAIGHT
                self.hold = 0

        if abs(d) >= BE_TURN_MIN:
            self.last = d

    def guard(self, ms):
        if self.state == APPROACH:
            return 1
        if self.state == COMMITTED:
            return 2 if (ms - self.state_ms) < BE_BLANK_MS else 1
        return 0

    def turn(self, cc):
        if self.state != COMMITTED:
            return None
        if cc >= BE_MIN_POINTS:
            return None
        if not self.hold:
            return None
        out = self.hold
        if self.hold > 0:
            self.hold = clamp(self.hold - BE_TURN_DECAY, 0, 30)
        else:
            self.hold = clamp(self.hold + BE_TURN_DECAY, -30, 0)
        return out


def out_of_track(guard, dark, lc, rc):
    """复现 main.c 的出界判定。"""
    if guard >= 2:
        return False
    if guard == 1:
        return dark >= 5
    return (dark >= 5) or (lc == 0 and rc == 0 and dark >= 3)


print("=== 1. 完整入弯序列：直道 → 确认 → 连接处 → 恢复 ===")
be = BendEntry()
ms = 0
frames = [
    # (bend, 转角, L, R, 暗点)
    (0, 0, 60, 58, 0),
    (0, 0, 60, 58, 0),
    (15, -8, 60, 58, 1),
    (16, -8, 58, 56, 1),
    (17, -9, 55, 52, 2),
    (18, -9, 40, 0, 2),
    (18, -9, 0, 0, 3),
    (16, -8, 0, 0, 3),
    (14, -7, 0, 12, 3),
    (12, -6, 22, 20, 2),
    (2, -2, 40, 38, 1),
    (0, 0, 58, 56, 0),
]
print(" 帧  毫秒  bend  转角    L   R 暗点  状态      宽容  出界")
for n, (bend, d, lc, rc, dark) in enumerate(frames):
    ms = n * 20
    be.update(ms, bend, d, lc, rc)
    g = be.guard(ms)
    print("  %2d  %4d  %3d  %+3d   %3d %3d  %2d  %-9s  %d    %s"
          % (n + 1, ms, bend, d, lc, rc, dark, NAMES[be.state], g,
             "停车" if out_of_track(g, dark, lc, rc) else "继续"))

print()
print("=== 2. 直道上误丢边：不能因为模块存在就放过 ===")
be = BendEntry()
print(" 帧  毫秒   L   R 暗点  状态      宽容  出界")
for n, (lc, rc, dark) in enumerate([(60, 58, 0), (0, 0, 3), (0, 0, 3), (0, 0, 5)]):
    ms = n * 20
    be.update(ms, 0, 0, lc, rc)
    g = be.guard(ms)
    print("  %2d  %4d  %3d %3d  %2d  %-9s  %d    %s"
          % (n + 1, ms, lc, rc, dark, NAMES[be.state], g,
             "停车" if out_of_track(g, dark, lc, rc) else "继续"))

print()
print("=== 3. 连接处超时：600 ms 后交还正常判据 ===")
be = BendEntry()
be.state = COMMITTED
be.state_ms = 0
be.hold = -9
print("  毫秒  已丢失  宽容  出界(暗点=3)")
for ms in (0, 100, 249, 250, 400, 599, 601, 800):
    be.lost_ms = ms
    if ms > BE_MAX_LOST_MS:
        be.state = RECOVER
        be.state_ms = ms
    g = be.guard(ms)
    print("  %4d   %4d    %d    %s" % (ms, ms, g, "停车" if out_of_track(g, 3, 0, 0) else "继续"))

print()
print("=== 4. 转角保持与衰减（中线点数不足时）===")
be = BendEntry()
be.state = COMMITTED
be.hold = -12
print("  中线点数  保持输出")
for cc in (4, 4, 4, 4, 4, 4, 4, 4, 20):
    v = be.turn(cc)
    print("  %4d      %s" % (cc, "无（走纯跟踪）" if v is None else "%+d" % v))

print()
print("=== 5. 连续弯道：S 弯之间直道很短，状态不回到 STRAIGHT ===")
be = BendEntry()
seq = [
    (0, 0, 60, 58),
    (16, -9, 58, 56),
    (18, -9, 55, 52),
    (20, -10, 0, 0),
    (18, -9, 0, 0),
    (14, -7, 0, 14),
    (12, -6, 20, 22),
    (14, 8, 22, 24),
    (16, 9, 20, 0),
    (16, 9, 0, 0),
    (2, 0, 40, 38),
]
print(" 帧  毫秒  bend  转角    L   R  状态      宽容")
for n, (bend, d, lc, rc) in enumerate(seq):
    ms = n * 20
    be.update(ms, bend, d, lc, rc)
    print("  %2d  %4d  %3d  %+3d   %3d %3d  %-9s  %d"
          % (n + 1, ms, bend, d, lc, rc, NAMES[be.state], be.guard(ms)))
print("  第 8 帧从 RECOVER 直接回 APPROACH，不经过 STRAIGHT")

print()
print("=== 6. 大转弯幅度：连接处容忍时间延长到 900 ms ===")
be = BendEntry()
be.state = COMMITTED
be.state_ms = 0
print("  已丢失   宽容  说明")
for ms in (200, 250, 500, 700, 899, 901):
    be.lost_ms = ms
    if ms > BE_MAX_LOST_MS:
        be.state = RECOVER
        be.state_ms = ms
    g = be.guard(ms)
    note = "免判" if g == 2 else ("只有 5 点全暗才判" if g == 1 else "恢复正常判据")
    print("  %4d ms   %d   %s" % (ms, g, note))
be.state = COMMITTED
be.state_ms = 0

print()
print("=== 7. 关键对照：同样两侧无边界，两种状态的结论 ===")
rows = [
    ("直道（未确认入弯）", STRAIGHT),
    ("已确认入弯，连接处 100 ms", COMMITTED),
    ("已确认入弯，连接处 400 ms", COMMITTED),
]
print(" 场景                        宽容  暗点=2  暗点=3  暗点=5")
for name, st in rows:
    be = BendEntry()
    be.state = st
    be.state_ms = 0
    ms = 100 if "100" in name else 400
    if st == COMMITTED:
        be.lost_ms = ms
    g = be.guard(ms)
    print("  %-26s  %d    %-6s  %-6s  %-6s"
          % (name, g,
             "停车" if out_of_track(g, 2, 0, 0) else "继续",
             "停车" if out_of_track(g, 3, 0, 0) else "继续",
             "停车" if out_of_track(g, 5, 0, 0) else "继续"))
