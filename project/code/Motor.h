//
// Created by 31663 on 2026/3/27.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
#include "zf_common_headfile.h"
#define DEBUG_MODE 0
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
extern int16 enableDifSpeed, currentSpeed;
extern int8 stop;
#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
