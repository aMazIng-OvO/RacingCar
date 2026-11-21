//
// 第三部分：十字形环岛（X 形路口）
//
// ============================ 输入 ============================
//   LRawCount / RRawCount      两侧边界的原始点数（拐点裁短前）
//   centerLineCount            中线点数（看到路口全白时会塌缩）
//   lastSteerDeg               上一帧实际送给舵机的角度（Motor.c 维护）
//
// ========================== 识别条件 ==========================
//   强条件（两条同时成立）：两侧原始点数都 < XC_MAX_PTS
//       —— 两条线都被路口缺口同时打断，这是路口的唯一稳定特征；
//   弱条件（兜底）：两侧都 < XC_WEAK_PTS 且 中线点数 < XC_MAX_CL
//       —— 线还在但中线被宽白区拉塌了，也是路口；
//   连续 XC_CONFIRM 帧成立才认（路口和"整车压到某条线上"要区分开）。
//
//   环岛不会命中：环岛入口有一侧是"长边界伸到远端"（>16 点），
//   而这里要求两侧都 <10，天然互斥。
//
// ========================== 触发后动作 ========================
//   1. 进入瞬间记下当前打角（限制在 ±XC_HOLD_MAX 度内）：
//      路口里没有可信的中线，继续算只会画龙；保持进路口那一刻的朝向前进，
//      是这类"宽白区"最稳的处理方式；
//   2. 路口内每帧输出这个固定角度，目标速度压到 XC_SPEED_RATIO（60%）；
//   3. 出路口：两侧边界同时恢复（> XC_BACK_PTS）即交回视觉；
//      或超过 XC_MAX_MS（600 ms，约 0.7 m）强制交回，防止卡死。
//
// ========================== 与别处的边界 ======================
//   本模块优先级最高：路口一旦判定，环岛与 S 弯本帧都不执行。
//   退出后由仲裁器按正常顺序重新检测。
//

#include "XCross.h"

/* ---- 识别参数 ---- */
#define XC_MAX_PTS       6    // 强条件：两侧原始点数都低于该值
#define XC_WEAK_PTS     10    // 弱条件：两侧都低于该值
#define XC_MAX_CL       10    // 弱条件：中线点数低于该值
#define XC_CONFIRM       3    // 连续成立帧数；2 帧太短，弯道入口也常短暂满足
#define XC_BACK_PTS     10    // 两侧都恢复到该值以上才算出了路口
#define XC_RECOVER_CL   20    // 中线恢复到该值以上说明已离开路口宽白区

/* ---- 动作参数 ---- */
#define XC_SPEED_RATIO  60    // 路口内目标速度比例（%）
#define XC_HOLD_MAX     10    // 保持角度的绝对值上限（度），防止带着大打角冲进路口
/* 保持朝向的时间窗。只有路口中心那一小段（宽白区、中线不可信）才需要开环，
 * 超过这段时间必须交回视觉——否则一旦车在路口里被这个固定角带偏，
 * 边线永远回不来，状态就永远退不出（实车"进得去出不来"就是这么来的）。 */
#define XC_FREEZE_MS   250
#define XC_MAX_MS      800    // 状态最长占用（兜底）
#define XC_REARM_MS    500    // 退出后的再触发锁定：这段时间内不许重新判定
#define XC_LOST_MS     150    // 两侧边界与中线同时为 0 持续这么久，视为丢线而非路口

extern int16 centerLineCount;
extern Point centerLine[expRoadCount];
extern int16 LRawCount, RRawCount;
extern int8 lastSteerDeg;

static uint8 xcOn;
static uint8 xcHit;          // 连续成立帧数
static int8 xcHoldDeg;       // 进路口时记住的角度
static uint16 xcMs;
static uint16 xcExitMs;      // 上次退出时刻（0 = 从未退出过）
static uint16 xcLostMs;      // 全丢线起始时刻
static uint8 xcLostOn;       // 1 = 正在统计全丢线时间

/* 强条件 + 弱条件，任意一个成立即可 */
static uint8 xcRaw(void) {
    uint8 bothShort = (uint8) ((LRawCount < XC_MAX_PTS) && (RRawCount < XC_MAX_PTS));
    uint8 bothWeak = (uint8) ((LRawCount < XC_WEAK_PTS) && (RRawCount < XC_WEAK_PTS) &&
                              (centerLineCount < XC_MAX_CL));

    return (uint8) (bothShort || bothWeak);
}

uint8 xcDetect(void) {
    if (xcOn) return 1;                 // 已经在路口里
    /* 全丢线不是路口：两侧边界与中线同时为 0 时没有任何"路口"证据，
     * 若仍按 XC_MAX_PTS 命中，车出界后会一直保持固定角，无法恢复。 */
    if (!LRawCount && !RRawCount && !centerLineCount) {
        xcHit = 0;
        return 0;
    }
    /* 中线还很长说明车在弯道里，两侧边界短只是弯道出口/内侧跑出画面，
     * 不是路口中心的宽白区。此时不能抢走控制权。 */
    if (centerLineCount > XC_RECOVER_CL) {
        xcHit = 0;
        return 0;
    }
    /* 刚退出的一小段时间内不再判定：路口里若还有一段残影，会立刻二次触发，
     * 表现就是"一直保持着一个固定角出不来" */
    if (xcExitMs && (uint16) (msTick - xcExitMs) < XC_REARM_MS) {
        xcHit = 0;
        return 0;
    }
    if (xcRaw()) {
        if (xcHit < 255) xcHit++;
    } else {
        xcHit = 0;
    }
    return (uint8) (xcHit >= XC_CONFIRM);
}

void xcEnter(void) {
    xcOn = 1;
    xcMs = msTick;
    xcHoldDeg = lastSteerDeg;
    if (xcHoldDeg > XC_HOLD_MAX) xcHoldDeg = XC_HOLD_MAX;
    if (xcHoldDeg < -XC_HOLD_MAX) xcHoldDeg = -XC_HOLD_MAX;
}

void xcRun(void) {
    elemSpeedRatio = XC_SPEED_RATIO;
    if ((uint16) (msTick - xcMs) <= XC_FREEZE_MS) {
        /* 路口中心：宽白区里没有可信中线，保持进路口那一刻的朝向直穿 */
        setServoDuty(xcHoldDeg);
    } else {
        /* 超过保持窗口：交回视觉。车已经过了白区，边线重新可用，
         * 继续开环只会把一个偏掉的角度一直保持下去 */
        alongLine(centerLine, centerLineCount, getAimPos());
    }
}

uint8 xcBusy(void) {
    if (!xcOn) return 0;
    /* 两侧同时恢复 = 已经穿过去 */
    if (LRawCount > XC_BACK_PTS && RRawCount > XC_BACK_PTS) return 0;
    /* 中线恢复 = 已经离开路口宽白区。弯道入口常出现"两侧短、中线还在"，
     * 如果继续开环保持固定角，车会被带出弯道。 */
    if (centerLineCount > XC_RECOVER_CL) return 0;
    /* 全丢线持续 150 ms：不是路口中心，是车已经看不到赛道，交回主循环处理 */
    if (!LRawCount && !RRawCount && !centerLineCount) {
        if (!xcLostOn) {
            xcLostOn = 1;
            xcLostMs = msTick;
        } else if ((uint16) (msTick - xcLostMs) >= XC_LOST_MS) {
            return 0;
        }
    } else {
        xcLostOn = 0;
    }
    if ((uint16) (msTick - xcMs) > XC_MAX_MS) return 0;
    return 1;
}

void xcExit(void) {
    xcOn = 0;
    xcHit = 0;
    xcHoldDeg = 0;
    xcMs = 0;
    xcLostOn = 0;
    /* 记下退出时刻用于再触发锁定；msTick 恰为 0 时用 1 代替（0 表示"从未退出"） */
    xcExitMs = msTick ? msTick : 1;
    elemSpeedRatio = 100;
}

void xcReset(void) {
    xcOn = 0;
    xcHit = 0;
    xcHoldDeg = 0;
    xcMs = 0;
    xcLostOn = 0;
    xcLostMs = 0;
    xcExitMs = 0;
    elemSpeedRatio = 100;
}

uint8 xcStage(void) {
    return xcOn;
}
