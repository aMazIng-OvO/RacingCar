#!/usr/bin/env python3
"""环岛第二路几何判据的离线核对。

对应 Round.c:
  rdInnerTurn()   内侧局部极值（先变小再变大）
  rdWidthAvg()    双侧同行平均宽度
  rdWidthExpand() 实际宽度相对基准的扩张
  rdGeomCand()    宽度扩张 + 内侧拐点/断线

不依赖摄像头，只验证整数判据的形状。
"""

TURN_PROM = 5
WIDTH_EXPAND_PX = 8


def inner_turn(ys):
    n = len(ys)
    if n < 6:
        return False
    for i in range(2, n - 2):
        y = ys[i]
        if y >= ys[i - 1] and y >= ys[i + 1]:
            if y - min(ys[i - 2], ys[i + 2]) >= TURN_PROM:
                return True
        elif y <= ys[i - 1] and y <= ys[i + 1]:
            if max(ys[i - 2], ys[i + 2]) - y >= TURN_PROM:
                return True
    return False


def width_expand(avg_width, base_width):
    return avg_width > base_width + WIDTH_EXPAND_PX


def geom_cand(outer_raw, far_row, inner_pts, outer_curve,
              width_expanded, inner_raw, has_turn, has_corner,
              steer_deg, side):
    if outer_raw <= 16 or far_row > 46:
        return False
    if inner_pts >= 10 or outer_curve > 6:
        return False
    if not width_expanded:
        return False
    if inner_raw and not (has_turn or has_corner):
        return False
    if side == 0 and steer_deg < -14:
        return False
    if side == 1 and steer_deg > 14:
        return False
    return True


def main():
    # 普通弯内侧边线单调，不触发拐点
    assert not inner_turn([50, 52, 54, 56, 58, 60])
    # 环岛圆弧“先增大再减小”，存在局部极值
    assert inner_turn([50, 55, 60, 58, 52, 50])

    # 普通弯宽度稳定：不扩张
    assert not width_expand(44, 40)
    # 环岛开口：宽度比基准多 12 像素
    assert width_expand(52, 40)

    # 普通弯：宽度不扩张，几何路径否决
    assert not geom_cand(26, 40, 8, 4, False, 8, True, False, 0, 0)
    # 环岛：宽度扩张 + 内侧拐点，即使外侧略弯（5）也成立
    assert geom_cand(26, 40, 8, 5, True, 8, True, False, 0, 0)
    # 环岛：内侧整条消失，靠宽度扩张成立
    assert geom_cand(26, 44, 0, 5, True, 0, False, False, 0, 0)
    # 内侧还剩点但没有拐点/断线：不成立
    assert not geom_cand(26, 40, 8, 5, True, 8, False, False, 0, 0)
    # 反向大舵：几何路径也否决
    assert not geom_cand(26, 40, 8, 5, True, 8, True, False, -20, 0)

    print("roundabout_geom_check: PASS")


if __name__ == "__main__":
    main()
