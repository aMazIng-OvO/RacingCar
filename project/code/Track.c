//
// 赛道识别 + 蓝灯指示
//

#include "Track.h"

/* 边界点数在 main.c 里定义（loadRoad 每帧更新） */
extern volatile int16 LRoadCount, RRoadCount;

#define TRACK_ON_POINTS    10     // 任一侧边界点数超过该值即视为"在赛道上"
#define TRACK_CONFIRM_MS   50     // 判据消抖：连续 50 ms 一致才切换状态

#define SLOW_BLINK_MS      500    // 慢闪半周期：亮 500 ms / 灭 500 ms（1 Hz）
#define LED_FAST_MS        100    // 常规指示：正向快闪的翻转周期

static uint8 onTrack;        // 0 = 不在赛道，1 = 在赛道
static uint8 confirmCnt;     // 判据消抖计数

static uint8 alarmOn;        // 1 = 正在做"不在赛道"慢闪
static uint16 slowMs;        // 慢闪计时
static uint8 slowPhase;      // 慢闪相位

static uint16 ledTimer;      // 常规指示的翻转计时
static uint8 ledPhase;       // 常规指示的亮灭相位

void trackInit(void) {
    onTrack = 0;
    confirmCnt = 0;
    alarmOn = 0;
    slowMs = 0;
    slowPhase = 1;
    ledTimer = 0;
    ledPhase = 0;
}

uint8 trackIsOn(void) {
    return onTrack;
}

void trackSetNoTrackAlarm(uint8 on) {
    alarmOn = on;
    slowMs = 0;
    slowPhase = 1; // 从头开始：先亮
    if (on) gpio_set_level(STATUS_LED_PIN, STATUS_LED_ON);
}

/* 常规指示：故障常亮 / 编码器正向快闪 / 反向常亮 / 静止熄灭 */
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
    uint8 raw;

    /* threshold == 128 是"画面没有明暗差"的兜底值，此时即使有点数也不可信 */
    raw = (uint8) ((threshold != 128) &&
                   ((LRoadCount > TRACK_ON_POINTS) || (RRoadCount > TRACK_ON_POINTS)));

    if (raw == onTrack) {
        confirmCnt = 0;
    } else if (++confirmCnt >= TRACK_CONFIRM_MS) {
        confirmCnt = 0;
        onTrack = raw;
        if (onTrack) alarmOn = 0; // 回到赛道：自动停掉慢闪
    }

    if (alarmOn) { // 发车被拒（不在赛道）：慢闪 1 Hz
        if (++slowMs >= SLOW_BLINK_MS) {
            slowMs = 0;
            slowPhase = (uint8) (!slowPhase);
        }
        gpio_set_level(STATUS_LED_PIN, slowPhase ? STATUS_LED_ON : STATUS_LED_OFF);
        return;
    }

    if (!onTrack) { // 不在赛道但没在报警：保持熄灭
        gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
        return;
    }

    trackNormalLed();
}
