//
// 弯道入口过渡区识别（独立模块）
//
// 状态机：STRAIGHT → APPROACH → COMMITTED → RECOVER → STRAIGHT
//
//   STRAIGHT   直道。累计同向的弯道强度，够帧数才进 APPROACH。
//   APPROACH   已经看到弯、开始打角，但边界还算完整。出界判据放宽一档。
//   COMMITTED  入弯连接处。单侧或双侧边界归零属于正常，前 250 ms 完全不判出界，
//              之后只有 5 点全暗才判；中线点数不足时用冻结的转角维持转向。
//   RECOVER    两侧都重新看到赛道，或连接处超时。强度回落且无丢边才回 STRAIGHT。
//
// 本模块只读 Image.c / main.c 产出的边界与中线，不修改任何现有算法。
//

#include "BendEntry.h"
#include "Image.h"
#include "Motor.h"

/* 与 Motor.c 的 alongLine() 保持同一套几何参数，
 * 但各自独立定义：本模块的改动不应影响主控制链路，反之亦然。 */
#define BE_FAR_AHEAD    12    // 远端趋势相对前瞻点的偏移（中线点序号）
#define BE_FEED_DIV     3     // 远端趋势的权重分母
#define BE_DEVICE_H     18    // 纯跟踪等效轴距（像素）
#define BE_TURN_MIN     2     // 参与同向确认的最小转角

/* 边界点数与中线在 main.c 里定义 */
extern int16 LRoadCount, RRoadCount, centerLineCount;

#define BE_STRAIGHT     0
#define BE_APPROACH     1
#define BE_COMMITTED    2
#define BE_RECOVER      3

static uint8 beState;        // 当前状态
static uint8 beSameDir;      // 弯道强度连续同向的帧数
static int8 beLastTurn;      // 最近一次有效转角的符号来源
static int8 beHoldTurn;      // 进入连接处时冻结的转角
static uint16 beStateMs;     // 进入当前状态的时刻（msTick）
static uint16 beLostMs;      // 连接处已持续的时间

void bendEntryInit(void) {
    beState = BE_STRAIGHT;
    beSameDir = 0;
    beLastTurn = 0;
    beHoldTurn = 0;
    beStateMs = 0;
    beLostMs = 0;
}

/* 复现 alongLine() 的几何：先按远端趋势修正横向偏差，再解算转角。
 * 同时给出弯道强度，量纲与 Motor.c 的 bend 一致。 */
static void bendEntryCalc(Point cl[], int16 cc, int16 aimPos, int8 *turn, int16 *bend) {
    int16 pos, farPos, dx, dy, dyAim, dyFar;
    int32 ax, ay;
    int8 d = 0;

    *bend = 0;
    *turn = 0;
    if (cc < 3) return;

    pos = func_limit_ab(aimPos, 2, cc - 1);
    /* 给远端趋势留出采样余量，前瞻点贴到末端时前馈才不会退化成 0 */
    if (cc > BE_FAR_AHEAD + 3)
        pos = func_limit_ab(pos, 2, cc - 1 - BE_FAR_AHEAD);
    dyAim = (int16) (94 - (int16) cl[pos].y);
    farPos = func_limit_ab(pos + BE_FAR_AHEAD, pos, cc - 1);
    dyFar = (int16) (94 - (int16) cl[farPos].y);
    dy = (int16) (dyAim + (dyFar - dyAim) / BE_FEED_DIV);
    dx = MT9V03X_H - cl[pos].x + BE_DEVICE_H;
    ay = (int32) 2 * BE_DEVICE_H * dy;
    ax = (int32) dx * dx + (int32) dy * dy;
    if (ax > 0) d = (int8) atan2_int(ay, ax);

    *turn = d;
    *bend = (int16) (func_abs((int16) (dyFar - dyAim)) + func_abs((int16) d));
}

void bendEntryUpdate(Point centerLine[], int16 centerLineCount, int16 aimPos) {
    int16 bend;
    int8 d, sign;
    uint8 oneLost, bothLost;

    bendEntryCalc(centerLine, centerLineCount, aimPos, &d, &bend);

    oneLost = (uint8) ((LRoadCount <= 0) || (RRoadCount <= 0));
    bothLost = (uint8) ((LRoadCount <= 0) && (RRoadCount <= 0));

    switch (beState) {
    case BE_STRAIGHT:
        /* 方向翻转时重新计数，避免左右来回摆也攒够帧数 */
        sign = (int8) (d > 0 ? 1 : (d < 0 ? -1 : 0));
        if (bend >= BE_BEND_MIN && func_abs((int16) d) >= BE_TURN_MIN) {
            if (beSameDir && sign && sign != (int8) (beLastTurn > 0 ? 1 : -1))
                beSameDir = 0;
            beSameDir++;
            if (beSameDir >= BE_CONFIRM_FRAMES) {
                beState = BE_APPROACH;
                beStateMs = msTick;
                beSameDir = 0;
            }
        } else {
            beSameDir = 0;
        }
        break;

    case BE_APPROACH:
        /* 一出现丢边就认定进入连接处，并把此刻的转角冻结下来 */
        if (oneLost) {
            beState = BE_COMMITTED;
            beStateMs = msTick;
            beLostMs = 0;
            if (func_abs((int16) d) >= BE_TURN_MIN) beHoldTurn = d;
        } else if (bend < BE_BEND_EXIT) {
            beState = BE_STRAIGHT;
        }
        break;

    case BE_COMMITTED:
        beLostMs = (uint16) (msTick - beStateMs);
        /* 两侧同时恢复才算真正重新看到赛道，单侧恢复在弯道里是常态 */
        if (!oneLost && LRoadCount > BE_RECOVER_POINTS && RRoadCount > BE_RECOVER_POINTS) {
            beState = BE_RECOVER;
            beStateMs = msTick;
        } else if (beLostMs > BE_MAX_LOST_MS) {
            /* 超时仍没看到双侧：可能是真的冲出去了，也有可能弯道很长。
             * 回到 RECOVER 让出界判据恢复，由主循环做最终判断。 */
            beState = BE_RECOVER;
            beStateMs = msTick;
        }
        break;

    case BE_RECOVER:
        /* 连续弯道：还没回到直道就又看到足够强的弯，直接回 APPROACH 重新确认。
         * 原来必须先满足 bend < BE_BEND_EXIT 才能离开 RECOVER，
         * S 弯之间那段直道太短时，状态机会卡在 RECOVER 里错过下一段。 */
        if (bend >= BE_BEND_MIN) {
            beState = BE_APPROACH;
            beStateMs = msTick;
        } else if (bend < BE_BEND_EXIT && !bothLost && !oneLost) {
            beState = BE_STRAIGHT;
            beHoldTurn = 0;
        } else if ((uint16) (msTick - beStateMs) > BE_MAX_LOST_MS) {
            beState = BE_STRAIGHT;
            beHoldTurn = 0;
        }
        break;

    default:
        beState = BE_STRAIGHT;
        break;
    }

    if (func_abs((int16) d) >= BE_TURN_MIN) beLastTurn = d;
}

uint8 bendEntryGuard(void) {
    if (beState == BE_APPROACH) return 1;
    if (beState == BE_COMMITTED) {
        /* 刚进入连接处的这段时间画面最不可信：弯道外侧已经出画、
         * 内侧可能还没爬出来，任何基于边界的判据都会误报 */
        if ((uint16) (msTick - beStateMs) < BE_BLANK_MS) return 2;
        return 1;
    }
    return 0;
}

uint8 bendEntryTurn(int8 *turn) {
    int8 next;

    if (beState != BE_COMMITTED) return 0;
    if (centerLineCount >= BE_MIN_POINTS) return 0;   // 中线够用，走纯跟踪
    if (!beHoldTurn) return 0;

    *turn = beHoldTurn;
    /* 每调用一次衰减一点，长时间保持同一个角度会让车画圆弧 */
    if (beHoldTurn > 0)
        next = (int8) func_limit_ab((int32) beHoldTurn - BE_TURN_DECAY, 0, 30);
    else
        next = (int8) func_limit_ab((int32) beHoldTurn + BE_TURN_DECAY, -30, 0);
    beHoldTurn = next;
    return 1;
}

uint8 bendEntryState(void) {
    return beState;
}
