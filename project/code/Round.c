//
// 第二部分：环岛
//
// ============================ 输入 ============================
//   LRawCount / RRawCount       拐点裁短前的边界点数（loadRoad 输出）
//   LRoadCount / RRoadCount     裁短后的点数：有拐点时等于拐点序号（≥3），
//                               没拐点时等于原始点数（0~2）——这一条决定了
//                               下面"内侧 < 4"同时覆盖"被拐点裁短"和"整条消失"
//   LRoad[] / RRoad[]           边界点列，[0] 最近，[n-1] 最远
//   LCorner / RCorner           拐点序号，0 = 没有拐点
//   centerLine / centerLineCount 中线点列（供 alongLine 循迹）
//
// ========================== 识别条件 ==========================
//   右环岛（内侧 = 右边界）：             左环岛（内侧 = 左边界）：
//     左边界原始点数 > RD_OUTER_MIN          右边界原始点数 > RD_OUTER_MIN
//     左边界末点行 ≤ RD_FAR_ROW               右边界末点行 ≤ RD_FAR_ROW
//     右边界点数  < RD_INNER_MAX              左边界点数  < RD_INNER_MAX
//     左边界平均弯曲残差 ≤ RD_OUTER_AVG_MAX    右边界平均弯曲残差 ≤ RD_OUTER_AVG_MAX
//
//   前三条同时成立还不够：普通急弯也会"外侧长、内侧被拐点裁短"。
//   第四条要求外侧边界足够直（环岛是一侧直道、一侧圆环，急弯两侧都弯），
//   专门否决普通急弯。候选常常只维持 1~2 帧（内侧边界在画面底部时隐时现），
//   所以用积分确认：成立 +1、不成立 -1、到 0 为止，攒到 RD_SCORE_NEED
//   才真正进入。断续出现能过，偶发一帧不会误判。
//
// ========================== 触发后动作 ========================
//   五阶段，每阶段一个固定的转向偏置（度，正 = 往右；左环岛取负）：
//
//     阶段    进入条件                        偏置            说明
//     READY   积分确认通过                    ±RD_BIAS_READY  入口 0.3 s 就过，先带一点
//     START   内侧缺口出现(<RD_INNER_MAX)     ±RD_BIAS_START  进环，打角最大
//             且内侧重新长回 >RD_INNER_BACK
//     IN      对侧缺口出现后重新长回 >RD_OUT_BACK  ±RD_BIAS_IN 环内视觉在线，只补一点
//     END     出口侧边界恢复且末点伸回外侧      0              出环要顺，交给视觉
//     EXIT    入口侧边界恢复                  0              收尾
//
//   全程 elemSpeedRatio = RD_SPEED_RATIO（50%），与 borderIn 的作用重复但更明确。
//   "缺口锁存"用 <RD_INNER_MAX 而不是 ==0：入口判据本来就是"<4"，
//   用 ==0 会在内侧只剩 1~3 点时永远锁不上（这是"想进进不去"的老毛病）。
//
// ========================== 退出条件 ==========================
//   EXIT 完成 / 单阶段超时 RD_STAGE_MS / 入口误判超时 RD_READY_MS。
//   两侧边界与中线同时为 0 持续 RD_LOST_MS：强制退出，避免满舵打转。
//   完整出环后 RD_REARM_MS 内不再重新判定，避免出口残影二次入环。
//   总时长由仲裁器兜底（ELEM_TOTAL_MS）。
//

#include "Round.h"

/* ---- 入口判据（与 main.c 的视野窗口同尺度） ---- */
/* 外侧边界原始点数下限。原 16 太松：日志里普通左弯只要右外线到 19 点，
 * 加上内侧消失/远端/直度三条就拼成 0xF0，直接误判成左环岛。
 * 20 表示必须 >20 点；真实环岛入口在 A/B 处外侧通常 22~38 点，留有余量。 */
#define RD_OUTER_MIN     20
#define RD_FAR_ROW       (VIEW_FAR_ROW + 8)   // 外侧末点必须伸到该行以内（34+8=42）
/* 内侧点数上限。原值 4 是按"内侧整条消失（0~2 点）"设的，但实车环岛入口的
 * 内侧线通常不是整条消失，而是**弯着退出画面侧边**，还剩 5~8 点就一直不满足，
 * 于是"看得见要进环岛、舵机也有打角倾向，但状态机永远不切"。
 * 7 是实测口径下的折中：普通弯道被拐点裁短的内侧线一般 3~5 点，环岛入口 5~8 点，
 * 单靠点数分不开，还要靠下面两条（外侧够长 + 外侧末点伸到远端）兜住特异性。
 * 若普通急弯被误判成环岛（表现为入弯突然降到 50% 速度），把它退回 5。 */
#define RD_INNER_MAX      7   // 内侧点数上限：< 7 即视为缺口
/* 积分确认门槛。原 3 帧约 30 ms 太短：日志里普通右弯只在 1120 ms 这一帧
 * 拼成 0x0F，下一帧 bit3 就掉了，3 帧门槛仍会触发。6 帧约 60~100 ms，
 * 真实环岛入口会持续满足，单帧拼出来的假候选过不了。 */
#define RD_SCORE_NEED     6
/* 外侧边界直度：环岛是"一侧直道、一侧圆环"，普通弯两侧都弯。
 * 用外侧首末点连线做基准，统计中间点的平均垂直偏差（像素）。
 * 直道/环岛外侧约 1~3 像素，R≈40 像素的急弯约 6 像素以上。
 * 这是入口候选的额外否决项，不参与五阶段推进，也不影响其他元素。 */
#define RD_OUTER_AVG_MAX  4
/* 入口方向一致性：如果车已经朝候选方向的反侧大幅打角，说明它正在走一个
 * 普通弯/S 弯，不是要进环岛。日志里出现过 DUTY=-376（左满舵）却判右环岛，
 * 下一拍直接把舵从 -376 拉到 +156，车立刻甩出去。
 * 10° 约等于 130~160 duty，超过就否决。 */
#define RD_ENTRY_OPP_MAX 10

/* ---- 文档式几何候选（第二路） ----
 * 严格路径要求"外侧 >20、内侧 <7、外侧几乎笔直、方向不冲突"，精度高但会漏：
 * 真环岛内侧可能还剩 7~9 点、外侧略弯、入环前带一点反向修正。
 * 第二路放宽点数/直度，但必须同时满足"标准宽度扩张 + 内侧拐点/断线"，
 * 普通弯宽度基本不变，因此不会被放宽条件带进来。 */
#define RD_GEOM_OUTER_MIN    16   // 外侧宽松下限
#define RD_GEOM_FAR_ROW      (VIEW_FAR_ROW + 12)  // 34+12=46，比严格路径多 4 行
#define RD_GEOM_INNER_MAX    10   // 内侧可放宽到 9 点
#define RD_GEOM_OUTER_CURVE   6   // 外侧平均残差放宽到 6 像素
#define RD_GEOM_OPP_MAX      14   // 几何路径允许的反向修正角
#define RD_TURN_PROM          5   // 内侧局部极值的最小突出量（像素）
#define RD_WIDTH_EXPAND_PX    8   // 实际宽度超过基准宽度的最小量（像素）

/* ---- 阶段推进门槛 ---- */
#define RD_INNER_BACK    10   // 内侧边界"长回来"的点数（确认已绕进环内）
#define RD_OUT_BACK      20   // 对侧边界缺口后长回来的点数
#define RD_END_PTS       25   // 出口侧边界恢复的点数门槛
#define RD_END_COL_R    133   // 右环岛出口：右边界末点列号 < 该值（94+39）
#define RD_END_COL_L     55   // 左环岛出口：左边界末点列号 > 该值（94-39）

/* ---- 各阶段转向偏置（度，右环岛为正；左环岛自动取负） ---- */
#define RD_BIAS_READY    12
#define RD_BIAS_START    18
#define RD_BIAS_IN        8

/* ---- 时间与速度 ---- */
#define RD_SPEED_RATIO   50   // 环岛内目标速度比例（%）
#define RD_READY_MS    1800   // READY 误判兜底（原 1200 太短：车还没开到环内就超时退回）
#define RD_STAGE_MS    8000   // 单阶段上限
#define RD_LOST_MS     1000   // 两侧边界与中线同时为 0 持续这么久，强制退出（1 s）
#define RD_REARM_MS     800   // 完整出环后 800 ms 内不再重新判环岛

#define RD_ST_READY       1
#define RD_ST_START       2
#define RD_ST_IN          3
#define RD_ST_END         4
#define RD_ST_EXIT        5

extern int16 centerLineCount, LCorner, RCorner;
extern int16 LRawCount, RRawCount;
extern volatile int16 LRoadCount, RRoadCount;
extern Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount];
extern int8 lastSteerDeg;    // 上一拍实际打角，用于入口方向一致性否决
extern uint8 centerLineDistance;  // 双侧实测半宽，用于标准宽度基准

static uint8 rdStageNo;      // 1~5，0 = 空闲
static uint8 rdSideNo;       // 0 = 右环岛，1 = 左环岛
static uint8 rdScoreR, rdScoreL;
static uint8 rdGap1, rdGap2, rdGap3, rdGap4;   // 四个缺口的锁存
static uint16 rdMs;          // 本阶段进入时刻
static uint16 rdLostMs;      // 全丢线起始时刻
static uint8 rdLostOn;       // 1 = 正在统计全丢线时间
static uint16 rdExitMs;      // 上次完整出环时刻（0 = 从未出过环）
/* 标准宽度基准：只在宽度稳定时缓慢跟随；检测到扩张时冻结，
 * 否则中心线半宽会被环岛开口"带大"，扩张判据永远不成立。 */
static int16 rdBaseWidth;    // 0 = 尚未建立
static int16 rdLastWidth;    // 最近一次双边界实测平均宽度
static uint8 rdWidthSeen;    // 1 = rdLastWidth 有效

/* 外侧边界相对"首末点连线"的平均垂直偏差（像素），只用于入口否决。 */
static int16 rdOuterCurve(uint8 side) {
    Point *p;
    int16 n, i, dx, dy, len, cnt;
    int32 cross, sumAbs, denom;

    p = (side == RD_SIDE_R) ? LRoad : RRoad;
    n = (side == RD_SIDE_R) ? LRawCount : RRawCount;
    if (n < 6 || n > expRoadCount) return 0;

    dx = (int16) ((int16) p[n - 1].x - (int16) p[0].x);
    dy = (int16) ((int16) p[n - 1].y - (int16) p[0].y);
    len = (int16) sqrt_int((int32) dx * dx + (int32) dy * dy);
    if (len < 8) return 0;

    /* 先累加，最后只除一次：逐点做 32 位除法在 C251 上是可观的额外开销。 */
    sumAbs = 0;
    for (i = 1; i < n - 1; i++) {
        cross = (int32) dx * ((int16) p[i].y - (int16) p[0].y) -
                (int32) dy * ((int16) p[i].x - (int16) p[0].x);
        sumAbs += func_abs(cross);
    }
    cnt = (int16) (n - 2);
    denom = (int32) len * cnt;
    if (!denom) return 0;
    return (int16) (sumAbs / denom);
}

/* 双侧边界在相同行上的平均宽度。环岛入口内侧开口时，这个平均值会比直道大。
 * 行号按近→远递减，游标只向前推进；找不到配对时沿用上一帧的宽度。 */
static int16 rdWidthAvg(uint8 side) {
    Point *po, *pi;
    int16 no, ni, i, j, k, d, best, bestD, w, sum, cnt;

    po = (side == RD_SIDE_R) ? LRoad : RRoad;
    no = (side == RD_SIDE_R) ? LRawCount : RRawCount;
    pi = (side == RD_SIDE_R) ? RRoad : LRoad;
    ni = (side == RD_SIDE_R) ? RRawCount : LRawCount;
    if (no < 3 || ni < 3 || no > expRoadCount || ni > expRoadCount)
        return rdWidthSeen ? rdLastWidth : 0;

    j = 0;
    sum = 0;
    cnt = 0;
    for (i = 0; i < no; i++) {
        while (j + 1 < ni && (int16) pi[j].x > (int16) po[i].x) j++;
        best = -1;
        bestD = 0x7FFF;
        for (k = j - 1; k <= j + 1; k++) {
            if (k < 0 || k >= ni) continue;
            d = (int16) func_abs((int16) pi[k].x - (int16) po[i].x);
            if (d < bestD) {
                bestD = d;
                best = k;
            }
        }
        if (best >= 0 && bestD <= 3) {
            w = (int16) func_abs((int16) pi[best].y - (int16) po[i].y);
            if (w > 0 && w < 150) {
                sum += w;
                cnt++;
            }
        }
    }
    if (cnt < 3) return rdWidthSeen ? rdLastWidth : 0;

    rdLastWidth = (int16) (sum / cnt);
    rdWidthSeen = 1;
    return rdLastWidth;
}

/* 实际宽度相对标准宽度是否明显扩张。基准只在宽度稳定时缓慢跟随，
 * 扩张时冻结，避免 centerLineDistance 被环岛开口一起带大。 */
static uint8 rdWidthExpand(uint8 side) {
    int16 w, base;

    w = rdWidthAvg(side);
    if (!rdWidthSeen || w <= 0) return 0;

    if (rdBaseWidth <= 0) {
        /* 第一次建立基准时优先用实测宽度；centerLineDistance 初值是 18，
         * 在宽赛道上会得到 36 的假基准，导致普通直道也被判成"扩张"。 */
        base = w;
        if (base < 20) base = (int16) centerLineDistance * 2;
        rdBaseWidth = base;
    }
    base = rdBaseWidth;
    if (base < 20) return 0;

    if (w > (int16) (base + RD_WIDTH_EXPAND_PX)) return 1;

    /* 正常宽度缓慢跟随（3/4 旧值 + 1/4 新值），避免赛道宽窄变化被误判。 */
    if (func_abs((int16) (w - base)) <= 4)
        rdBaseWidth = (int16) (((int32) base * 3 + w) / 4);
    return 0;
}

/* 内侧边界是否存在局部极值：环岛圆弧会“先变小再变大”，
 * 普通弯的内侧边线通常保持单调。 */
static uint8 rdInnerTurn(uint8 side) {
    Point *p;
    int16 n, i, y, prom;

    p = (side == RD_SIDE_R) ? RRoad : LRoad;
    n = (side == RD_SIDE_R) ? RRawCount : LRawCount;
    if (n < 6 || n > expRoadCount) return 0;

    for (i = 2; i < n - 2; i++) {
        y = (int16) p[i].y;
        if (y >= (int16) p[i - 1].y && y >= (int16) p[i + 1].y) {
            prom = (int16) (y - func_min((int16) p[i - 2].y, (int16) p[i + 2].y));
            if (prom >= RD_TURN_PROM) return 1;
        } else if (y <= (int16) p[i - 1].y && y <= (int16) p[i + 1].y) {
            prom = (int16) (func_max((int16) p[i - 2].y, (int16) p[i + 2].y) - y);
            if (prom >= RD_TURN_PROM) return 1;
        }
    }
    return 0;
}

/* 第二路：放宽点数/直度，但必须同时有“宽度扩张 + 内侧拐点/断线”。
 * 普通弯宽度基本不变，不会命中这一路。 */
static uint8 rdGeomCand(uint8 side) {
    int16 outerRaw, innerPts, innerRaw, outerFarRow;

    if (side == RD_SIDE_R) {
        outerRaw = LRawCount;
        innerPts = RRoadCount;
        innerRaw = RRawCount;
        if (LRawCount <= 0 || LRawCount > expRoadCount) return 0;
        outerFarRow = (int16) LRoad[LRawCount - 1].x;
    } else {
        outerRaw = RRawCount;
        innerPts = LRoadCount;
        innerRaw = LRawCount;
        if (RRawCount <= 0 || RRawCount > expRoadCount) return 0;
        outerFarRow = (int16) RRoad[RRawCount - 1].x;
    }

    if (outerRaw <= RD_GEOM_OUTER_MIN) return 0;
    if (outerFarRow > RD_GEOM_FAR_ROW) return 0;
    if (innerPts >= RD_GEOM_INNER_MAX) return 0;
    if (rdOuterCurve(side) > RD_GEOM_OUTER_CURVE) return 0;
    if (!rdWidthExpand(side)) return 0;

    /* 内侧整条消失时只能靠宽度；还剩 5~9 点时要求有局部极值或拐点。
     * 注意这里不能用 rdInnerCorner()，它依赖当前 rdSideNo，而本函数会左右两侧都试。 */
    if (innerRaw && !rdInnerTurn(side) &&
        !(side == RD_SIDE_R ? RCorner : LCorner)) return 0;

    if (side == RD_SIDE_R && lastSteerDeg < -RD_GEOM_OPP_MAX) return 0;
    if (side == RD_SIDE_L && lastSteerDeg > RD_GEOM_OPP_MAX) return 0;
    return 1;
}

/* ------------------------- 入口候选 ------------------------- */
/* side：0 = 右环岛（内侧是右边界），1 = 左环岛（内侧是左边界） */
static uint8 rdCand(uint8 side) {
    int16 outerRaw, innerPts;
    int16 outerFarRow;
    uint8 strict = 1;

    if (side == RD_SIDE_R) {
        outerRaw = LRawCount;
        innerPts = RRoadCount;
        if (LRawCount <= 0 || LRawCount > expRoadCount) return 0;
        outerFarRow = (int16) LRoad[LRawCount - 1].x;
    } else {
        outerRaw = RRawCount;
        innerPts = LRoadCount;
        if (RRawCount <= 0 || RRawCount > expRoadCount) return 0;
        outerFarRow = (int16) RRoad[RRawCount - 1].x;
    }

    /* 第一路：原有严格判据，保持普通弯不误判。 */
    if (outerRaw <= RD_OUTER_MIN) strict = 0;
    if (outerFarRow > RD_FAR_ROW) strict = 0;
    if (innerPts >= RD_INNER_MAX) strict = 0;
    if (rdOuterCurve(side) > RD_OUTER_AVG_MAX) strict = 0;
    if (side == RD_SIDE_R && lastSteerDeg < -RD_ENTRY_OPP_MAX) strict = 0;
    if (side == RD_SIDE_L && lastSteerDeg > RD_ENTRY_OPP_MAX) strict = 0;
    if (strict) return 1;

    /* 第二路：文档式几何判据（宽度扩张 + 内侧拐点/断线）。 */
    return rdGeomCand(side);
}

static void rdScoreStep(void) {
    if (rdCand(RD_SIDE_R)) {
        if (rdScoreR < RD_SCORE_NEED) rdScoreR++;
    } else if (rdScoreR) rdScoreR--;

    if (rdCand(RD_SIDE_L)) {
        if (rdScoreL < RD_SCORE_NEED) rdScoreL++;
    } else if (rdScoreL) rdScoreL--;
}

/* ------------------------- 对外接口 ------------------------- */
uint8 rdDetect(void) {
    if (rdStageNo) return 1;      // 已经在环岛里，不再重新判定
    /* 完整出环后短暂锁定：出口附近边界还会残留环岛形状，立刻重判会二次入环 */
    if (rdExitMs && (uint16) (msTick - rdExitMs) < RD_REARM_MS) {
        rdScoreR = rdScoreL = 0;
        return 0;
    }

    rdScoreStep();
    if (rdScoreR >= RD_SCORE_NEED) { rdSideNo = RD_SIDE_R; return 1; }
    if (rdScoreL >= RD_SCORE_NEED) { rdSideNo = RD_SIDE_L; return 1; }
    return 0;
}

void rdEnter(void) {
    rdStageNo = RD_ST_READY;
    rdMs = msTick;
    rdGap1 = rdGap2 = rdGap3 = rdGap4 = 0;
    rdScoreR = rdScoreL = 0;
}

/* 本帧四个量的取值，按方向取"内侧 / 外侧"两个轴 */
static int16 rdInnerPts(void) {
    return (rdSideNo == RD_SIDE_R) ? RRoadCount : LRoadCount;
}
static int16 rdInnerRaw(void) {
    return (rdSideNo == RD_SIDE_R) ? RRawCount : LRawCount;
}
static int16 rdOuterRaw(void) {
    return (rdSideNo == RD_SIDE_R) ? LRawCount : RRawCount;
}
static int16 rdOuterCorner(void) {
    return (rdSideNo == RD_SIDE_R) ? LCorner : RCorner;
}
static int16 rdInnerCorner(void) {
    return (rdSideNo == RD_SIDE_R) ? RCorner : LCorner;
}

static int8 rdBiasNow(void) {
    int8 b;

    switch (rdStageNo) {
    case RD_ST_READY: b = RD_BIAS_READY; break;
    case RD_ST_START: b = RD_BIAS_START; break;
    case RD_ST_IN:    b = RD_BIAS_IN;    break;
    default:          b = 0;             break;
    }
    /* 右环岛往右（正），左环岛镜像取负 */
    return (int8) (rdSideNo == RD_SIDE_R ? b : -b);
}

void rdRun(void) {
    int16 innerRaw, outerRaw, outerFarCol;

    /* 全丢线兜底：误判成环岛后，如果两侧边界与中线同时归零，
     * 状态机会一直按阶段偏置打满舵，车原地打转、再也回不到赛道。
     * 持续 RD_LOST_MS（1 s）就强制结束本次环岛，交回仲裁器重新识别。 */
    if (!LRawCount && !RRawCount && !centerLineCount) {
        if (!rdLostOn) {
            rdLostOn = 1;
            rdLostMs = msTick;
        } else if ((uint16) (msTick - rdLostMs) >= RD_LOST_MS) {
            rdStageNo = 0;
            elemSteerBiasDeg = 0;
            elemSpeedRatio = 100;
            return;
        }
    } else {
        rdLostOn = 0;
    }

    elemSpeedRatio = RD_SPEED_RATIO;
    elemSteerBiasDeg = rdBiasNow();
    /* 环岛入口内侧边界消失时，中线可能塌缩到 0~2 点。
     * alongLine() 在 count<3 时直接 return，既不调用 setServoDuty()，
     * 也就不会消费 elemSteerBiasDeg，表现为入口打角断档、随后按直线回正。
     * 这里只在环岛路径用纯偏置兜底，不改变 alongLine() 的通用行为。 */
    if (centerLineCount >= 3)
        alongLine(centerLine, centerLineCount, getAimPos());
    else if (elemSteerBiasDeg)
        setServoDuty(0);   // 有环岛偏置时才兜底；END/EXIT 偏置为 0，保持原行为

    innerRaw = rdInnerRaw();
    outerRaw = rdOuterRaw();
    outerFarCol = 0;
    if (rdSideNo == RD_SIDE_R) {
        if (LRawCount > 0 && LRawCount <= expRoadCount) outerFarCol = (int16) LRoad[LRawCount - 1].y;
    } else {
        if (RRawCount > 0 && RRawCount <= expRoadCount) outerFarCol = (int16) RRoad[RRawCount - 1].y;
    }

    switch (rdStageNo) {
    case RD_ST_READY:
        if (rdInnerPts() < RD_INNER_MAX) rdGap1 = 1;          // 缺口出现过
        if (rdGap1 && rdInnerRaw() > RD_INNER_BACK && !rdInnerCorner()) {
            rdStageNo = RD_ST_START;
            rdMs = msTick;
        }
        break;

    case RD_ST_START:
        if (outerRaw < RD_INNER_MAX) rdGap2 = 1;              // 对侧缺口
        if (rdGap2 && outerRaw > RD_OUT_BACK && !rdOuterCorner()) {
            rdStageNo = RD_ST_IN;
            rdMs = msTick;
        }
        break;

    case RD_ST_IN:
        if (outerRaw < RD_INNER_MAX) rdGap3 = 1;              // 出口缺口
        if (rdGap3 && outerRaw > RD_OUT_BACK) {
            rdStageNo = RD_ST_END;
            rdMs = msTick;
        }
        break;

    case RD_ST_END:
        /* 出口侧边界恢复且末点重新伸回外侧（说明已经绕过出口的圆弧） */
        if (!rdOuterCorner() && outerRaw > RD_END_PTS &&
            ((rdSideNo == RD_SIDE_R) ? (outerFarCol > RD_END_COL_L)
                                     : (outerFarCol < RD_END_COL_R))) {
            rdStageNo = RD_ST_EXIT;
            rdMs = msTick;
        }
        break;

    case RD_ST_EXIT:
        if (innerRaw < RD_INNER_MAX) rdGap4 = 1;              // 入口侧再出现缺口
        if (rdGap4 && innerRaw > RD_INNER_BACK) rdStageNo = 0; // 完成
        break;

    default:
        rdStageNo = 0;
        break;
    }
}

uint8 rdBusy(void) {
    uint16 limit;

    if (!rdStageNo) return 0;
    limit = (rdStageNo == RD_ST_READY) ? RD_READY_MS : RD_STAGE_MS;
    if ((uint16) (msTick - rdMs) > limit) return 0;
    return 1;
}

void rdExit(void) {
    rdStageNo = 0;
    rdMs = 0;
    rdGap1 = rdGap2 = rdGap3 = rdGap4 = 0;
    /* 积分一起清零：否则刚出环岛那一帧分数还在门槛上，会立刻被自己重新触发 */
    rdScoreR = rdScoreL = 0;
    /* msTick 恰为 0 时用 1 代替，0 表示"从未出过环" */
    rdExitMs = msTick ? msTick : 1;
    rdLostOn = 0;
    elemSteerBiasDeg = 0;
    elemSpeedRatio = 100;
}

void rdReset(void) {
    rdStageNo = 0;
    rdSideNo = RD_SIDE_R;
    rdScoreR = rdScoreL = 0;
    rdGap1 = rdGap2 = rdGap3 = rdGap4 = 0;
    rdMs = 0;
    rdLostOn = 0;
    rdLostMs = 0;
    rdExitMs = 0;
    rdBaseWidth = 0;
    rdLastWidth = 0;
    rdWidthSeen = 0;
    elemSteerBiasDeg = 0;
    elemSpeedRatio = 100;
}

uint8 rdStage(void) {
    return rdStageNo;
}

uint8 rdSide(void) {
    return rdSideNo;
}

/* 只读诊断：把入口判据暴露出来，只看当前帧，不改状态。
 *   右环岛（外侧 = 左边界，内侧 = 右边界）
 *     bit0  外侧原始点数 > RD_OUTER_MIN
 *     bit1  外侧末点行   <= RD_FAR_ROW
 *     bit2  内侧点数     <  RD_INNER_MAX
 *     bit3  外侧边界足够直（不是普通急弯）
 *     → 0x0F 是严格路径完整候选
 *   左环岛是镜像，用 bit4~bit7（0xF0）
 *
 * 第二路几何候选（宽度扩张 + 内侧拐点）成立时，对应侧 4 位直接置满，
 * 所以 CAND=0x0F/0xF0 表示“最终判定为候选”，不再区分是哪一路。
 * 若需要知道几何路径为什么成立，看 STG/MODE 与现场 L/R/C/DUTY。 */
uint8 rdCandBits(void) {
    uint8 bits = 0;
    int16 farRow;

    if (LRawCount > 0 && LRawCount <= expRoadCount) {
        farRow = (int16) LRoad[LRawCount - 1].x;
        if (LRawCount > RD_OUTER_MIN) bits |= 0x01;
        if (farRow <= RD_FAR_ROW) bits |= 0x02;
        if (rdOuterCurve(RD_SIDE_R) <= RD_OUTER_AVG_MAX) bits |= 0x08;
    }
    if (RRoadCount < RD_INNER_MAX) bits |= 0x04;

    if (RRawCount > 0 && RRawCount <= expRoadCount) {
        farRow = (int16) RRoad[RRawCount - 1].x;
        if (RRawCount > RD_OUTER_MIN) bits |= 0x10;
        if (farRow <= RD_FAR_ROW) bits |= 0x20;
        if (rdOuterCurve(RD_SIDE_L) <= RD_OUTER_AVG_MAX) bits |= 0x80;
    }
    if (LRoadCount < RD_INNER_MAX) bits |= 0x40;

    /* 第二路几何候选：宽度扩张 + 内侧拐点/断线。 */
    if (rdGeomCand(RD_SIDE_R)) bits |= 0x0F;
    if (rdGeomCand(RD_SIDE_L)) bits |= 0xF0;

    return bits;
}

/* 只读诊断：积分确认的进度（取两侧较大值），到 RD_SCORE_NEED 就会触发。 */
uint8 rdScore(void) {
    return (uint8) (rdScoreR > rdScoreL ? rdScoreR : rdScoreL);
}
