//
// 发车 / 里程统计 / 定点停车
//

#include "Stop.h"

volatile int32 odometer = 0;
int32 lapPulse = LAP_PULSE_DEFAULT, brakePulse = BRAKE_PULSE_DEFAULT;
uint8 raceState = RACE_IDLE;

void stopInit(void) {
    odometer = 0;
    lapPulse = LAP_PULSE_DEFAULT;
    brakePulse = BRAKE_PULSE_DEFAULT;
    raceState = RACE_IDLE;
}

void raceStart(void) {
    odometer = 0;
    stop = 0;
    handle = 0;
    enableDifSpeed = 1;
    gpio_set_level(IO_P90, 0);
    raceState = RACE_RUNNING;
}

void raceSetLap(void) {
    // 推车从起跑线走完一圈、再前进到需要的停车点（约 1m 处）后按 BOTTOM 键
    if (odometer > 200) { // 里程过小视为误操作
        lapPulse = odometer;
        brakePulse = odometer / 8; // 最后 1/8 圈开始刹车
    }
    odometer = 0;
}

void raceStopNow(void) {
    stop = 1;
    raceState = RACE_STOPPED;
    gpio_set_level(IO_P90, 1); // 蜂鸣器提示已停车
}

void raceTask(void) {
    if (raceState == RACE_RUNNING) {
        if (odometer >= lapPulse - brakePulse) {
            raceState = RACE_BRAKING;
            handle = 1;   // 锁定速度，避免曲率限速重新拉高 setSpeed
            setSpeed = 0; // 电机 PID 目标 0，主动制动
        }
    } else if (raceState == RACE_BRAKING) {
        if (odometer >= lapPulse || func_abs(currentSpeed) < 30) {
            raceState = RACE_STOPPED;
            stop = 1;
            gpio_set_level(IO_P90, 1);
        }
    }
}

void raceShow(void) {
    ips200_show_string(0, 200, "ODO:");
    ips200_show_int32(64, 200, odometer, 10);
    ips200_show_string(0, 220, "LAP:");
    ips200_show_int32(64, 220, lapPulse, 10);
    ips200_show_string(0, 240, "THR:");
    ips200_show_int16(64, 240, LThreshold);
    ips200_show_int16(120, 240, RThreshold);
}
