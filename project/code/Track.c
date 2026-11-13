//
// 赛道识别 + 蓝灯指示
//

#include "Track.h"

/* 边界点数在 main.c 里定义（loadRoad 每帧更新） */
extern volatile int16 LRoadCount, RRoadCount;

#define TRACK_ON_POINTS    10     // 任一侧边界点数超过该值即视为"在赛道上"
#define TRACK_CONFIRM_MS   50     // 判据消抖：连续 50 ms 一致才切换状态

#define BLINK_ON_MS        80     // 报警快闪：亮 80 ms
#define BLINK_OFF_MS       80     //           灭 80 ms
#define BLINK_TIMES        3      // 一组闪 3 下
#define ALARM_PAUSE_MS     1000   // 一组闪完停 1 秒

#define LED_FAST_MS        100    // 常规指示：正向快闪的翻转周期

static uint8 onTrack;        // 0 = 不在赛道，1 = 在赛道
static uint8 confirmCnt;     // 判据消抖计数

static uint16 alarmTimer;    // 报警状态机的剩余毫秒
static uint8 alarmPhase;     // 0=亮 1=灭 2=判断是否继续 3=长停
static uint8 blinkCnt;       // 本组已闪次数

static uint16 ledTimer;      // 常规指示的翻转计时
static uint8 ledPhase;       // 常规指示的亮灭相位

void trackInit(void) {
    onTrack = 0;
    confirmCnt = 0;
    alarmTimer = 0;
    alarmPhase = 0;
    blinkCnt = 0;
    ledTimer = 0;
    ledPhase = 0;
}

uint8 trackIsOn(void) {
    return onTrack;
}

/* 报警：快闪 3 下 → 停 1 秒 → 循环 */
static void trackAlarm(void) {
    if (alarmTimer > 0) {
        alarmTimer--;
        return;
    }
    switch (alarmPhase) {
        case 0: // 亮
            gpio_set_level(STATUS_LED_PIN, STATUS_LED_ON);
            alarmTimer = BLINK_ON_MS;
            alarmPhase = 1;
            break;
        case 1: // 灭
            gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
            alarmTimer = BLINK_OFF_MS;
            alarmPhase = 2;
            break;
        case 2: // 本次闪完，判断一组是否结束
            if (++blinkCnt >= BLINK_TIMES) {
                alarmPhase = 3;
                alarmTimer = ALARM_PAUSE_MS;
            } else {
                alarmPhase = 0;
                alarmTimer = 0;
            }
            break;
        default: // 长停结束，开始下一组
            blinkCnt = 0;
            alarmPhase = 0;
            alarmTimer = 0;
            break;
    }
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
        if (!onTrack) { // 刚离开赛道：报警从头开始
            alarmTimer = 0;
            alarmPhase = 0;
            blinkCnt = 0;
        }
    }

    if (!onTrack) {
        trackAlarm();
        return;
    }

    /* 回到赛道：清掉报警状态，恢复正常指示 */
    alarmTimer = 0;
    alarmPhase = 0;
    blinkCnt = 0;
    trackNormalLed();
}
