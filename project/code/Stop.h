//
// 发车 / 里程统计 / 定点停车
// 适用：2026 南京工业大学智能汽车竞速组（传统赛道）
//
// 规则：车模从起跑线后出发，完成赛道 1 周后返回，停在起跑线前方 2 米内。
// 实现：用编码器累计里程（脉冲）判定一圈，进入刹车段后由电机 PID 主动制动。
//

#ifndef HFK_STC32G144K_STOP_H
#define HFK_STC32G144K_STOP_H

#include "zf_common_headfile.h"

/* ---------------- 里程参数：上车后必须实测标定 ---------------- */
#define PULSE_PER_METER     1000    // 每米脉冲数（推车走 1 米读取 odometer）
#define LAP_DISTANCE_CM     1200    // 赛道一圈的长度 cm
#define STOP_OFFSET_CM      100     // 过起跑线后继续前进的停车点 cm（必须 0~200）
#define BRAKE_DISTANCE_CM   150     // 提前开始刹车的距离 cm

// 注意：C251 的 int 为 16 位，乘法必须先转 int32，否则 (1300*1000) 会溢出
#define LAP_PULSE_DEFAULT   ((int32) ((int32) (LAP_DISTANCE_CM + STOP_OFFSET_CM) * PULSE_PER_METER / 100))
#define BRAKE_PULSE_DEFAULT ((int32) ((int32) BRAKE_DISTANCE_CM * PULSE_PER_METER / 100))

/* ---------------- 比赛状态 ---------------- */
#define RACE_IDLE       0   // 等待发车（此状态可推车标定里程）
#define RACE_RUNNING    1   // 正常跑圈
#define RACE_BRAKING    2   // 已进入刹车段，仍在循迹
#define RACE_STOPPED    3   // 已停车

extern volatile int32 odometer;     // 发车后累计里程（脉冲）
extern int32 lapPulse;              // 一圈总脉冲（含停车余量）
extern int32 brakePulse;            // 提前刹车脉冲
extern uint8 raceState;

void stopInit(void);        // 初始化（选档结束后调用一次）
void raceStart(void);       // 发车：清里程、解除刹车
void raceSetLap(void);      // 把当前里程记为一圈长度（推车标定）
void raceStopNow(void);     // 急停
void raceTask(void);        // 每帧调用：里程判定与制动
void raceShow(void);        // 等待发车状态下在 IPS200 显示里程

#endif //HFK_STC32G144K_STOP_H
