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
#define FORWARD 114
#define CROSS 514
#define R_ROUND 1919
#define L_ROUND 810
#define ALONG_L 88
#define ALONG_R 99

volatile extern int16 setSpeed, handle, encoderL, encoderR;
void motorInit(int16 c11, int16 c22, int16 c33, int16 c44, int16 c55, int16 c66, int16 c77);
void setServoDuty(int8 duty);
void alongLine(Point roadLine[], int16 count, int16 aimPos);
int8 getAimPos();
void controlReset(void);      // 发车/急停时清空 PID 历史与控制量
extern volatile int16 enableDifSpeed, currentSpeed;
extern volatile int8 stop;
#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
