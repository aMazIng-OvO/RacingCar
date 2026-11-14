import math

H = 120
W = 188
DEVICE_H = 18
C1 = 260
STEER_DUTY_DIV = 20
STEER_DUTY_LIMIT = 395
CENTER_COL = 94
AIM_ROW = 60           # 前瞻点所在行，取中线中段的典型值
MERGE_WIDTH_MIN = 12
MERGE_WIDTH_MAX = 130
FIXED_HALF = 18


def angle_of(dy, row=AIM_ROW):
    dx = H - row + DEVICE_H
    return math.degrees(math.atan2(2 * DEVICE_H * dy, dx * dx + dy * dy))


def duty_of(dy):
    return int(angle_of(dy) * C1 / STEER_DUTY_DIV)


def duty_to_deg(duty):
    """图像角度：duty 反算纯跟踪角，用于横向像素换算。"""
    return duty * STEER_DUTY_DIV / C1


def duty_to_servo_deg(duty):
    """舵机实际转角：按 395 duty 对应约 24° 的行程估算，1° 约 16.5 duty。"""
    return duty * 24.0 / STEER_DUTY_LIMIT


print("=== 1. 单侧外推的半宽固定 18 时，中线偏离真实中心多少像素 ===")
print("赛道宽度  真实半宽  中线偏移  纯跟踪角度  舵机 duty")
for width in (24, 36, 48, 60, 72, 96):
    half = width // 2
    off = half - FIXED_HALF          # 中线偏向可见侧的距离
    print("  %3d       %3d       %3d      %+6.2f°     %+5d"
          % (width, half, off, angle_of(off), duty_of(off)))

print()
print("=== 2. 半宽自适应后的偏移（两侧都能看到时用实测宽度）===")
print("赛道宽度  自适应半宽  中线偏移  纯跟踪角度  舵机 duty")
for width in (24, 36, 48, 60, 72, 96):
    half = width // 2
    adapted = max(8, min(60, half))  # 与 main.c 的 func_limit_ab(hw, 8, 60) 一致
    off = half - adapted
    print("  %3d        %3d        %3d      %+6.2f°     %+5d"
          % (width, adapted, off, angle_of(off), duty_of(off)))

print()
print("=== 3. TRACK_BIAS_PX 的换算（摄像头零偏补偿）===")
print("偏置(像素)  横向偏差  纯跟踪角度  舵机 duty  相当于 STEER_TRIM")
for b in (0, 3, 5, 8, 12, 16):
    print("  %3d        %+3d       %+6.2f°     %+5d        %3d"
          % (b, b, angle_of(b), duty_of(b), duty_of(b)))

print()
print("=== 4. STEER_TRIM 的换算（舵机中值补偿）===")
print("TRIM(duty)  舵机转角  图像角度  等效横向偏差(像素)")
for t in (-33, -20, -10, 0, 10, 15, 20, 33):
    servo = duty_to_servo_deg(t)
    deg = duty_to_deg(t)
    # 反解横向像素：angle_of(dy) = |deg|
    lo, hi = 0.0, 80.0
    for _ in range(40):
        mid = (lo + hi) / 2
        if angle_of(mid) < abs(deg):
            lo = mid
        else:
            hi = mid
    px = lo if t >= 0 else -lo
    print("  %+3d        %+6.2f°   %+6.2f°     %+6.1f" % (t, servo, deg, px))

print()
print("=== 5. MOTOR_DIFF_TRIM 的换算（左右轮速差）===")
print("TRIM  右轮目标变化  左轮目标变化  相对差速(以 setSpeed=300 计)")
for t in (0, 4, 8, 12):
    print("  %2d      %+5d        %+5d        %+5.1f%%"
          % (t, -t, t, (2 * t) * 100.0 / 300))

print()
print("=== 6. 当前默认值的合计修正量 ===")
trim, bias, mdiff = -58, 0, 0
print("  STEER_TRIM = %+d duty  舵机转角 %+.2f°  图像角度 %+.2f°"
      % (trim, duty_to_servo_deg(trim), duty_to_deg(trim)))
print("  TRACK_BIAS_PX = %d 像素  等效 %.2f°  %.0f duty"
      % (bias, angle_of(bias), duty_of(bias)))
print("  MOTOR_DIFF_TRIM = %d  等效相对差速 %.1f%%（setSpeed=300 时）"
      % (mdiff, (2 * mdiff) * 100.0 / 300))
print("  当前修正量对应的机械偏差：%.2f°（1° 约 %.1f duty）"
      % (abs(duty_to_servo_deg(trim)), 24.0 / STEER_DUTY_LIMIT and STEER_DUTY_LIMIT / 24.0))
print("  当前设置把舵机拉向左侧 %.2f°，用于抵消机械右偏"
      % abs(duty_to_servo_deg(trim)))

print()
print("=== 7. 偏左程度的估计：串口 DUTY 读数 ===")
print("  车稳定跑在某侧时，DUTY 的平均值就是纯跟踪为了抵消偏置付出的稳态转角。")
print("  DUTY 平均值  对应横向偏差  建议 STEER_TRIM")
for d in (0, 10, 20, 30, 40):
    lo, hi = 0.0, 60.0
    for _ in range(40):
        mid = (lo + hi) / 2
        if angle_of(mid) < duty_to_deg(d):
            lo = mid
        else:
            hi = mid
    print("    %+4d        %+6.1f 像素     %3d" % (d, lo, d))

print()
print("=== 8. 上电舵机自检的摆动范围 ===")
STEER_MID = 2975
TRIM = -58
SWING = 495
center = STEER_MID + TRIM
print("  中心 = steerMid + STEER_TRIM = %d + (%d) = %d duty（%.1f us）"
      % (STEER_MID, TRIM, center, center * 0.5))
print("  范围 = %d ± %d = %d ~ %d duty（%.1f ~ %.1f us）"
      % (center, SWING, center - SWING, center + SWING,
         (center - SWING) * 0.5, (center + SWING) * 0.5))
print("  摆幅 %d duty 约 %.1f°（按 395 duty ≈ 24° 换算）" % (SWING, SWING * 24.0 / 395))
print("  结束位置 = 中心 %d duty，即机械 90°" % center)
print("  时长 = 200 ms 初始化 + 3 段 × %d 步 × 20 ms ≈ %.1f s"
      % (SWING // 25 + 1, (200 + 3 * (SWING // 25 + 1) * 20) / 1000.0))
