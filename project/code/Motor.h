//
// Created by 31663 on 2026/3/27.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
#include "zf_common_headfile.h"
#define DEBUG_MODE 0

/* 稳定优先：全局目标速度天花板，单位与 c5 / setSpeed 一致（脉冲 / 5 ms）。
 * 8 档 c5 中 520 / 450 / 500 三档会被压到该值，其余档位不受影响。
 * sqrt_int() 修复后弯道限速变大，在实车重新标定前先由这里兜住上限；
 * 需要提速时只改这一个宏，不必重标 8 组参数。 */
#define SAFE_SPEED_CEILING 400

/* ===== 低速试车模式 =====
 * 0 = 正常比赛
 * 1 = 试车：把所有档位的目标速度压到 LOW_SPEED_TEST_CAP，
 *     并临时关闭"两侧边界全丢 / 视野 5 点全暗"的出界保护
 *     （桌面上没有赛道、摄像头看不到线时也能让车慢慢跑，用来验证电机/舵机/编码器）
 * 正式比赛前必须改回 0 */
#define LOW_SPEED_TEST      0
#define LOW_SPEED_TEST_CAP  150

/* ===== 异常状态检测（faultFlags 位标志，屏幕显示 ERR:<十进制>） =====
 * 编码器/堵转在 5 ms 控制中断里判，图像类在主循环按帧判；
 * 编码器掉线与堵转属于致命故障，会立即停车并置 FAULT_STOPPED。 */
#define FAULT_ENC_L_DOWN   0x01   // 左编码器长时间无反馈（有速度指令但计数为 0）
#define FAULT_ENC_R_DOWN   0x02   // 右编码器长时间无反馈
#define FAULT_ENC_SKEW     0x04   // 左右轮速差超过 50%（打滑 / 一侧编码器异常）
#define FAULT_STALL        0x08   // 有速度指令但两轮都几乎不转（堵转 / 驱动未上电）
#define FAULT_NO_FRAME     0x10   // 摄像头停帧（约 1 s 无新帧）
#define FAULT_IMAGE_DARK   0x20   // 图像几乎没有对比（全局阈值退回 128）
#define FAULT_NO_BORDER    0x40   // 连续多帧看不到赛道边界
#define FAULT_STOPPED      0x80   // 因故障被强制停车
#define FAULT_PID_SAT      0x0100 // PID 输出长时间打满（占空比饱和，车却跑不动）
#define FAULT_PID_REV      0x0200 // PID 输出与编码器反馈方向相反（速度环正反馈，飞车前兆）

extern volatile uint16 faultFlags;
void faultReset(void);

/* 蜂鸣器电平：报警时输出 BEEP_ON_LEVEL。
 * 若报警时没有声音（或一直响），把这两个值对调即可。 */
#define BEEP_ON_LEVEL      1
#define BEEP_OFF_LEVEL     0

#define FORWARD 114
#define CROSS 514
#define R_ROUND 1919
#define L_ROUND 810
#define ALONG_L 88
#define ALONG_R 99

volatile extern int16 setSpeed, handle, encoderL, encoderR;
void motorInit(int16 c11, int16 c33, int16 c44, int16 c55, int16 c77);
void setServoDuty(int8 duty);
void alongLine(Point roadLine[], int16 count, int16 aimPos);
int8 getAimPos();
void controlReset(void);      // 发车/急停时清空 PID 历史与控制量
extern volatile int16 enableDifSpeed, currentSpeed;
extern volatile int8 stop;
#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
