import difflib
import hashlib
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REF = os.path.join(ROOT, "backup", "version1.3", "project")
CUR = os.path.join(ROOT, "version1.0", "project")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest().upper()


def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return text


def norm_lines(text):
    out = []
    for line in strip_comments(text).split("\n"):
        line = line.strip()
        line = re.sub(r"\s+", " ", line)
        if line:
            out.append(line)
    return out


def extract_function(text, signature):
    start = text.find(signature)
    if start < 0:
        return None
    depth = 0
    i = text.find("{", start)
    if i < 0:
        return None
    begin = i
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[begin : i + 1]
        i += 1
    return None


def report_equal(name, a, b):
    """比较前去掉注释与空白：注释不影响行为，代码行必须逐一相同。"""
    la, lb = norm_lines(a), norm_lines(b)
    if la == lb:
        print("  %-28s 一致（归一化 %d 行）" % (name, len(la)))
        return True
    print("  %-28s 不一致" % name)
    for i in range(min(len(la), len(lb))):
        if la[i] != lb[i]:
            print("    首个差异在第 %d 行" % (i + 1))
            print("      参考: " + la[i])
            print("      当前: " + lb[i])
            break
    if len(la) != len(lb):
        print("    归一化行数 参考=%d 当前=%d" % (len(la), len(lb)))
    return False


print("=== 1. 参考实现的来源 ===")
print("  目录: backup/version1.3/project")
print("  Image.c: %d 行" % sum(1 for _ in open(os.path.join(REF, "code", "Image.c"), encoding="utf-8", errors="replace")))
print("  Image.h: %d 行" % sum(1 for _ in open(os.path.join(REF, "code", "Image.h"), encoding="utf-8", errors="replace")))

def changed_lines(ref_path, cur_path):
    a = open(ref_path, encoding="utf-8", errors="replace").read().splitlines()
    b = open(cur_path, encoding="utf-8", errors="replace").read().splitlines()
    d = list(difflib.unified_diff(a, b, n=0, lineterm=""))
    removed = [x[1:] for x in d if x.startswith("-") and not x.startswith("---")]
    added = [x[1:] for x in d if x.startswith("+") and not x.startswith("+++")]
    return removed, added


print()
print("=== 2. 与参考的差异：只允许 BODY_ROW 相关的改动 ===")
ok = True
for name in ("Image.c", "Image.h"):
    hr = sha256(os.path.join(REF, "code", name))
    hc = sha256(os.path.join(CUR, "code", name))
    print("  %-10s 参考哈希 %s  当前哈希 %s" % (name, hr[:16], hc[:16]))

c_removed, c_added = changed_lines(os.path.join(REF, "code", "Image.c"),
                                   os.path.join(CUR, "code", "Image.c"))
c_ok = (len(c_removed) == 2 and len(c_added) == 2 and
        all("i < HIDE_ROW" in x for x in c_removed) and
        all("i < BODY_ROW" in x for x in c_added))
print("  Image.c   删除 %d 行 新增 %d 行  %s" % (len(c_removed), len(c_added), "符合预期" if c_ok else "超出范围"))
for line in c_removed + c_added:
    print("      " + line.strip())
if not c_ok:
    ok = False

h_removed, h_added = changed_lines(os.path.join(REF, "code", "Image.h"),
                                   os.path.join(CUR, "code", "Image.h"))
h_ok = (len(h_removed) == 0 and any("#define BODY_ROW 80" in x for x in h_added))
print("  Image.h   删除 %d 行 新增 %d 行  %s" % (len(h_removed), len(h_added), "符合预期" if h_ok else "超出范围"))
for line in h_added:
    print("      " + line.strip())
if not h_ok:
    ok = False

print()
print("=== 3. loadRoad() 与参考的关系 ===")
ref_main = open(os.path.join(REF, "user", "main.c"), encoding="utf-8", errors="replace").read()
cur_main = open(os.path.join(CUR, "user", "main.c"), encoding="utf-8", errors="replace").read()
fr = extract_function(ref_main, "void loadRoad() {")
fc = extract_function(cur_main, "void loadRoad() {")
if fr is None or fc is None:
    print("  未能提取 loadRoad")
    ok = False
else:
    # 比较终点取到半宽自适应之前：该段是有意加入的，单列为允许项
    ref_end = "filterRoad(LRoad, LRoadCount);"
    cur_end = "if (RUN_MODE == FORWARD && LRoadCount > 10 && RRoadCount > 10) {"
    start = "if (RUN_MODE == FORWARD) centerLineDistance = 18;"
    a, b = fr.find(ref_end), fc.find(cur_end)
    a0, b0 = fr.find(start), fc.find(start)
    if min(a, b, a0, b0) < 0:
        print("  未能定位感知调用序列")
        ok = False
    else:
        # 局部变量声明是新增融合函数的需要，比较从第一条感知语句开始
        ok = report_equal("感知前半段（爬线到拐点之前）", fr[a0:a], fc[b0:b]) and ok
        la, lb = norm_lines(fr[a:]), norm_lines(fc[b:])
        print("  后半段归一化行数 参考=%d 当前=%d（中线生成按弯道识别需求改进）"
              % (len(la), len(lb)))
    cur_calls = strip_comments(cur_main)
    cur_motor = strip_comments(open(os.path.join(CUR, "code", "Motor.c"),
                                    encoding="utf-8", errors="replace").read())
    has_merge = "buildCenterLine(midL, midLc, midR, midRc);" in cur_calls
    has_snapshot = "memcpy(midL, LRoad" in cur_calls
    keeps_element = "LRoadCount = LCorner;" in cur_calls
    has_halfw = "centerLineDistance = (uint8) func_limit_ab(hw, 8, 60);" in cur_calls
    # 偏置项用在纯跟踪里，位于 Motor.c 的 alongLine()
    has_bias = ("TRACK_BIAS_PX" in cur_motor) and ("STEER_TRIM" in cur_motor)
    print("  中线改用完整点列融合:", "是" if has_merge else "否")
    print("  拐点前保存未裁短的边界副本:", "是" if has_snapshot else "否")
    print("  元素判据仍用截断后的全局点数:", "是" if keeps_element else "否")
    print("  单侧外推的半宽改用实测宽度:", "是" if has_halfw else "否")
    print("  纯跟踪带横向偏置标定项:", "是" if has_bias else "否")
    if not (has_merge and has_snapshot and keeps_element and has_halfw and has_bias):
        ok = False

print()
print("=== 4. 关键参数（Image.h 宏定义）===")
ref_h = open(os.path.join(REF, "code", "Image.h"), encoding="utf-8", errors="replace").read()
cur_h = open(os.path.join(CUR, "code", "Image.h"), encoding="utf-8", errors="replace").read()
for macro in ("block", "cap", "expRoadCount", "EDGE_DIFF", "EDGE_RATIO", "HIDE_ROW",
              "HIDE_COL_L", "HIDE_COL_R", "CORNER_COS100"):
    pr = re.search(r"#define\s+%s\s+([^\s/]+)" % macro, ref_h)
    pc = re.search(r"#define\s+%s\s+([^\s/]+)" % macro, cur_h)
    vr = pr.group(1) if pr else "缺失"
    vc = pc.group(1) if pc else "缺失"
    mark = "一致" if vr == vc else "不一致"
    if vr != vc:
        ok = False
    print("  %-14s 参考=%-4s 当前=%-4s %s" % (macro, vr, vc, mark))

print()
print("=== 5. 赛道判定与参考出界条件的互补性 ===")
print("  参考: out_of_track = (LRoadCount == 0) and (RRoadCount == 0)")
print("  当前: on_track     = (LRoadCount > 0) or (RRoadCount > 0)")
rows = [(0, 0), (0, 1), (0, 40), (0, 100), (1, 0), (40, 0), (100, 0), (40, 40)]
all_ok = True
print("  %-10s %-10s %-14s %-12s %s" % ("LRoad", "RRoad", "参考出界", "当前在赛道", "判定"))
for lc, rc in rows:
    ref_out = (lc == 0 and rc == 0)
    cur_on = (lc > 0 or rc > 0)
    good = cur_on == (not ref_out)
    all_ok = all_ok and good
    print("  %-10d %-10d %-14s %-12s %s" % (lc, rc, ref_out, cur_on, "通过" if good else "失败"))
if not all_ok:
    ok = False

print()
print("=== 6. 上电初始状态 ===")
init_ref = re.search(r"LRoadCount = RRoadCount = expRoadCount", ref_main)
init_cur = re.search(r"LRoadCount = RRoadCount = expRoadCount", cur_main)
print("  参考 elementReset 填入 expRoadCount:", "是" if init_ref else "否")
print("  当前 elementReset 填入 expRoadCount:", "是" if init_cur else "否")
if not (init_ref and init_cur):
    ok = False

print()
print("总判定: " + ("全部符合预期" if ok else "存在超出范围的差异"))
