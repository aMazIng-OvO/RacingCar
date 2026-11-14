//
// 赛道状态判定 + 蓝灯指示
//
// 判定依据与参考实现一致：参考工程的出界保护用
//     !LRoadCount && !RRoadCount
// 表示"两侧边界全丢"，本模块取它的补集作为"在赛道上"的判据。
// 输入是 loadRoad() 产出的边界点数，不额外读像素，所以相同输入必得相同结论。

#include "Track.h"

/* 边界点数在 main.c 里定义（loadRoad 每帧更新） */
extern volatile int16 LRoadCount, RRoadCount;

#define TRACK_CONFIRM_MS   50     // 判据消抖：连续 50 ms 一致才切换状态

#define LED_FAST_MS        100    // 正向快闪的翻转周期

static uint8 onTrack;        // 0 = 不在赛道，1 = 在赛道
static uint8 rawOn;          // 未经消抖的原始判据
static uint16 confirmMs;     // 判据开始与当前状态不一致的时刻

static uint16 ledTimer;      // 常规指示的翻转计时
static uint8 ledPhase;       // 常规指示的亮灭相位
static uint16 ledHoldUntil;  // 按键反馈的保持截止时刻（msTick）
static uint8 ledHoldActive;  // 1 = 正在保持按键反馈

/* 按键确认反馈：按住蓝灯一段时间，期间常规指示让出驱动权。 */
void trackLedHold(uint16 ms) {
    ledHoldUntil = (uint16) (msTick + (ms ? ms : 1));
    ledHoldActive = 1;
    gpio_set_level(STATUS_LED_PIN, STATUS_LED_ON);
}

void trackInit(void) {
    onTrack = 0;
    rawOn = 0;
    confirmMs = 0;
    ledTimer = 0;
    ledPhase = 0;
}

uint8 trackIsOn(void) {
    return onTrack;
}

/* 赛道判定的原始结果。判据与参考实现的出界条件互斥：
 * 参考用「两侧都为 0」表示边界全丢，这里用「任一侧非 0」表示仍在赛道上。 */
uint8 trackEvaluate(void) {
    rawOn = (uint8) ((LRoadCount > 0) || (RRoadCount > 0));
    return rawOn;
}

/* 常规指示：故障常亮 / 编码器正向快闪 / 反向常亮 / 静止熄灭。
 * 位掩码与参考实现的 updateStatusLed() 一致。 */
static void trackNormalLed(void) {
    ledTimer++;
    if (faultFlags & (FAULT_ENC_L_DOWN | FAULT_ENC_R_DOWN | FAULT_PID_SAT | FAULT_PID_REV)) {
        gpio_set_level(STATUS_LED_PIN, STATUS_LED_ON);
        ledPhase = 1;
        ledTimer = 0;
    } else if (currentSpeed >= 3) {
        if (ledTimer >= LED_FAST_MS) {
            ledTimer = 0;
            ledPhase = (uint8) (!ledPhase);
        }
        gpio_set_level(STATUS_LED_PIN, ledPhase ? STATUS_LED_ON : STATUS_LED_OFF);
    } else if (currentSpeed <= -3) {
        gpio_set_level(STATUS_LED_PIN, STATUS_LED_ON);
        ledPhase = 1;
        ledTimer = 0;
    } else {
        gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
        ledPhase = 0;
        ledTimer = 0;
    }
}

void trackMonitor(void) {
    /* 按键反馈保持期内不驱动 LED，否则刚点亮的反馈会被这一拍立刻覆盖。
     * 用「截止时刻减当前时刻」判断是否还没到，msTick 回绕时依然成立。 */
    if (ledHoldActive) {
        if ((int16) (ledHoldUntil - msTick) > 0) return;
        ledHoldActive = 0;
    }

    /* 消抖按毫秒计时：主循环一轮的耗时取决于图像处理，不能拿轮数当时间 */
    if (rawOn == onTrack) {
        confirmMs = msTick;
    } else if ((uint16) (msTick - confirmMs) >= TRACK_CONFIRM_MS) {
        onTrack = rawOn;
        confirmMs = msTick;
    }

    trackNormalLed();
}
