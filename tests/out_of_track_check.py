"""出界判据核对：两侧边界点数与 5 点采样的组合，以及发车宽限期。"""

GRACE_MS = 800


def out_of_track(lc, rc, dark, ms_since_run=GRACE_MS):
    """复现 main.c 的判据：5 点全暗直接出界；两侧无边界时要求至少 3 点暗。"""
    flag = (dark >= 5) or (lc == 0 and rc == 0 and dark >= 3)
    if ms_since_run < GRACE_MS:
        flag = False
    return flag


def old_out_of_track(lc, rc, dark):
    """上一版：两侧无边界单独就成立，或 5 点全暗。"""
    return (lc == 0 and rc == 0) or (dark >= 5)


print("=== 1. 判据真值表（发车 800 ms 之后）===")
print("场景                                  L  R  暗点  上一版  当前  期望")
cases = [
    ("直道上两侧都正常", 60, 58, 0, False),
    ("直道，车头前方偏暗", 60, 58, 2, False),
    ("直道，画面整体偏暗", 60, 58, 4, False),
    ("弯道里边界跑出搜索范围", 0, 0, 0, False),
    ("弯道里边界失败，画面有亮区", 0, 0, 2, False),
    ("弯道里边界失败，画面多数偏暗", 0, 0, 3, True),
    ("单侧可见，画面多数偏暗", 40, 0, 3, False),
    ("单侧可见，画面全暗", 40, 0, 5, True),
    ("两侧都无边界，画面全暗", 0, 0, 5, True),
    ("真正冲出赛道（贴边）", 0, 6, 5, True),
]
ok_all = True
for name, lc, rc, dark, expect in cases:
    old = old_out_of_track(lc, rc, dark)
    cur = out_of_track(lc, rc, dark)
    ok = cur == expect
    ok_all = ok_all and ok
    print("  %-34s %2d %2d  %d    %-5s  %-5s  %s"
          % (name, lc, rc, dark, old, cur, "通过" if ok else "不符预期"))

print()
print("=== 2. 发车宽限期 ===")
print("  发车后经过  当前判据  说明")
for t in (0, 200, 799, 800, 1200):
    v = out_of_track(0, 0, 5, t)
    note = "宽限期内不判出界" if v is False else "已开始判出界"
    print("  %4d ms      %-5s    %s" % (t, v, note))

print()
print("=== 3. 弯道附近的典型帧序列（发车 900 ms 后）===")
seq = [(58, 56, 1), (30, 28, 2), (0, 12, 2), (0, 0, 2), (0, 0, 3), (0, 0, 5)]
print("  帧   L   R  暗点  上一版  当前   结果")
for n, (lc, rc, dark) in enumerate(seq):
    old = old_out_of_track(lc, rc, dark)
    cur = out_of_track(lc, rc, dark)
    note = ""
    if old and not cur:
        note = "上一版会误停，当前继续跑"
    elif cur:
        note = "判定出界"
    print("  %2d  %2d  %2d   %d    %-5s  %-5s  %s" % (n + 1, lc, rc, dark, old, cur, note))

print()
print("总判定: " + ("全部符合预期" if ok_all else "存在不符预期的用例"))
