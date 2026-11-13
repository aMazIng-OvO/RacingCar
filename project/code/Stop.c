//
// 发车 / 里程统计 / 定点停车
//

#include "Stop.h"

volatile int32 odometer = 0;
int32 lapPulse = LAP_PULSE_DEFAULT, brakePulse = BRAKE_PULSE_DEFAULT;
volatile uint8 raceState = RACE_IDLE;
volatile uint8 lapCalibrated = 0; // 是否已按 BOTTOM 标定过一圈（0 = 未标定，禁止发车）

void stopInit(void) {
    elementReset();
    interrupt_global_disable();
    odometer = 0;
    interrupt_global_enable();
    lapPulse = LAP_PULSE_DEFAULT;
    brakePulse = BRAKE_PULSE_DEFAULT;
    raceState = RACE_IDLE;
    lapCalibrated = 0; // 每次上电都要重新标定（脉冲/米是整车常数，但以实推为准）
}

void raceStart(void) {
    /* A1：未标定一圈就发车，里程会在几百毫秒内越过 lapPulse-brakePulse，
     * 表现为"起步跑一下就自己停"，而屏幕上看不出任何原因。这里直接拒绝发车。 */
    if (!lapCalibrated) {
        ips200_clear(RGB565_WHITE);
        ips200_show_string(0, 130, "CALIB LAP FIRST");
        ips200_show_string(0, 160, "push 1 lap, press B3");
        ips200_show_string(0, 190, "then B2 to start");
        return;
    }
    // 先复位全部元素/控制状态，最后才解除 stop，
    // 避免 PIT 在 stop=0 与 raceState=RUNNING 之间的窗口用旧目标驱动
    elementReset();
    interrupt_global_disable();
    odometer = 0;
    interrupt_global_enable();
    handle = 0;
    enableDifSpeed = 1;
    gpio_set_level(IO_P90, 0);
    raceState = RACE_RUNNING;
    stop = 0;
}

void raceSetLap(void) {
    int32 snap;
    // 推车从起跑线走完一圈、再前进到需要的停车点（约 1m 处）后按 BOTTOM 键
    // 32 位里程由 PIT 读改写，必须一次性取快照后再清零
    interrupt_global_disable();
    snap = odometer;
    odometer = 0;
    interrupt_global_enable();
    // 门槛按预期圈长取一半，避免轻微推车/误按把一圈标定成 0.2m
    if (snap > LAP_PULSE_DEFAULT / 2) {
        lapPulse = snap;
        // 提前刹车量按"刹车距离 / 一圈总距离"的比例换算，不依赖 PULSE_PER_METER 的绝对值
        brakePulse = lapPulse * BRAKE_DISTANCE_CM / (LAP_DISTANCE_CM + STOP_OFFSET_CM);
        if (brakePulse >= lapPulse) brakePulse = lapPulse >> 3; //参数填错时兜底，避免发车即进刹车段
        lapCalibrated = 1; // 标定成功，才允许发车
    }
}

void raceStopNow(void) {
    elementReset();          // 急停后同样复位元素/控制状态，发车时不会带旧阶段
    stop = 1;
    raceState = RACE_STOPPED;
    gpio_set_level(IO_P90, 1); // 蜂鸣器提示已停车
}

void raceTask(void) {
    int32 snap;
    interrupt_global_disable();
    snap = odometer;
    interrupt_global_enable();
    if (raceState == RACE_RUNNING && snap >= lapPulse - brakePulse) {
        raceState = RACE_BRAKING;
    }
    if (raceState == RACE_BRAKING) {
        // 每帧重新压住制动目标，防止 PIT/元素逻辑把 setSpeed 抬回去
        handle = 1;
        setSpeed = 0;
        // 低速只在已进入停车点附近时才允许提前收车，避免刹车段起点误停
        if (snap >= lapPulse ||
            (snap >= lapPulse - brakePulse / 4 && func_abs(currentSpeed) < 30)) {
            elementReset();   // 完赛停车也复位元素/控制状态
            raceState = RACE_STOPPED;
            stop = 1;
            gpio_set_level(IO_P90, 1);
        }
    }
}

void raceShow(void) {
    /* 屏幕是整机最贵的操作（一次文本刷新要重设窗口 + 走 SPI）。
     * 待发车阶段按帧调用会把大量算力耗在刷屏上，这里降到约 1/30 帧率（≈5 Hz），
     * 读数依然跟得上手推车标定，但不再和视觉/控制抢 CPU。 */
    static uint8 div = 0;
    int32 snap;
    if (++div < 30) return;
    div = 0;
    interrupt_global_disable();
    snap = odometer;
    interrupt_global_enable();
    ips200_show_string(0, 200, "ODO:");
    ips200_show_int32(64, 200, snap, 10);
    ips200_show_string(0, 220, "LAP:");
    ips200_show_int32(64, 220, lapPulse, 10);
    ips200_show_string(0, 240, "THR:");
    ips200_show_int16(64, 240, LThreshold);
    ips200_show_int16(120, 240, RThreshold);
}
