H = 120
W = 188
LTHRESHOLD = 130
EDGE_DIFF = 12
EDGE_RATIO = 7
HALF_W = 94
HIDE_COL_L = 48
HIDE_ROW_REF = 90
BODY_ROW = 80


def make_image(track_left, track_right, body_left=None, body_right=None,
               body_top=80, body_bottom=80):
    img = [[60] * W for _ in range(H)]
    for row in range(H):
        for col in range(track_left, track_right + 1):
            img[row][col] = 200
    if body_left is not None:
        for row in range(body_top, body_bottom + 1):
            for col in range(body_left, body_right + 1):
                img[row][col] = 40
    return img


def index_start(img, i, row_limit, side):
    """与 getLIndexStart / getRIndexStart 相同的二分，row_limit 决定搜宽列还是窄列。"""
    if side == "L":
        low, high = 3, (HALF_W if i < row_limit else HIDE_COL_L)
        while low < high:
            m = (low + high) >> 1
            if (img[i][m] < LTHRESHOLD and
                    img[i][max(0, m - 8)] < LTHRESHOLD and
                    img[i][max(0, m - 25)] < LTHRESHOLD):
                low = m + 1
            else:
                high = m
        return low
    low, high = (HALF_W if i < row_limit else 140), 186 - 2
    while low < high:
        m = (low + high + 1) >> 1
        if (img[i][m] < LTHRESHOLD and
                img[i][min(187, m + 8)] < LTHRESHOLD and
                img[i][min(187, m + 25)] < LTHRESHOLD):
            high = m - 1
        else:
            low = m
    return high


def check(img, i, y):
    d = abs(img[i][y - 2] - img[i][y + 2])
    a = img[i][y - 2] + img[i][y + 2]
    return d > EDGE_DIFF and a // d <= EDGE_RATIO


def find_left_start(img, row_limit, rows=range(116, 69, -3)):
    for i in rows:
        y = index_start(img, i, row_limit, "L")
        if img[i][min(y + 2, 187)] > LTHRESHOLD and check(img, i, y):
            return i, y + 1
    return 0, 0


print("=== 起点搜索列范围：HIDE_ROW=90（参考）与 BODY_ROW=80（当前）===")
print("场景                                  参考起点   当前起点   结论")

cases = [
    ("无车身，赛道左边界在列 30", make_image(30, 158), False),
    ("车身只在中央 55~125，不遮边界", make_image(30, 158, 55, 125, 70, 116), False),
    ("车身遮住左边界 20~60，行 80~116", make_image(30, 158, 20, 60, 80, 116), True),
    ("车身延伸到列 10~100，行 80~116", make_image(30, 158, 10, 100, 80, 116), True),
]

for name, img, expect_skip in cases:
    ref = find_left_start(img, HIDE_ROW_REF)
    cur = find_left_start(img, BODY_ROW)
    if expect_skip:
        if cur[1] == 31 and ref[1] != 31:
            note = "当前找到真实边界，参考把车身边缘当成边界"
        elif ref == cur:
            note = "两者一致，车身未影响该行判定"
        else:
            note = "需复查"
    else:
        note = "两者一致" if ref == cur else "结论不同"
    print("%-38s %-10s %-10s %s" % (name, ref, cur, note))

print()
print("=== 车身行逐行分析（车身 20~60，真实边界 30）===")
img = make_image(30, 158, 20, 60, 80, 116)
for i in (116, 100, 92, 89, 80, 77, 74, 71):
    yr = index_start(img, i, HIDE_ROW_REF, "L")
    yc = index_start(img, i, BODY_ROW, "L")
    okr = img[i][min(yr + 2, 187)] > LTHRESHOLD and check(img, i, yr)
    okc = img[i][min(yc + 2, 187)] > LTHRESHOLD and check(img, i, yc)
    print("  行 %3d  参考: 列%3d 起点有效=%-5s  当前: 列%3d 起点有效=%-5s" % (i, yr, okr, yc, okc))

print()
print("=== 窄列搜索在直道上的可用性（车身不遮边界）===")
img = make_image(30, 158)
ref = find_left_start(img, HIDE_ROW_REF)
cur = find_left_start(img, BODY_ROW)
print("  参考起点", ref, " 当前起点", cur, " 一致:", ref == cur)
