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

/* ===== 目标速度硬上限：不超过 2 m/s =====
 * setSpeed 的单位是「脉冲 / 5 ms」，换算公式：
 *     上限 = v(m/s) × PULSE_PER_METER / 200
 * 例：实测 PULSE_PER_METER = 34600 时，2 m/s → 346。
 * ⚠ PULSE_PER_METER 目前仍是占位值 1000（未实测），直接套公式只有 10，车会完全不动，
 *   所以这里先给实测前的保守值；推车 1 m 标定完 PULSE_PER_METER 后请按公式重算。 */
#define MAX_SPEED_MPS      2
#define MAX_SET_SPEED      180   // 调试期先压到很低，确认能稳住再往上提（300 ≈ 原先的最低档满速）

/* 实际生效的速度上限：目标速度与左右轮目标都按它限幅（稳定后可整体提高） */
#if (MAX_SET_SPEED < SAFE_SPEED_CEILING)
#define SPEED_LIMIT        MAX_SET_SPEED
#else
#define SPEED_LIMIT        SAFE_SPEED_CEILING
#endif

/* 上电舵机自检摆幅：左右各 15°。
 * 200 Hz 下 1 duty = 0.5 µs；常见舵机 0.5 ms ≈ 60°，故 15° ≈ 250 duty。 */
#define SERVO_TEST_SWING   250

/* ===== 低速试车模式 =====
 * 0 = 正常比赛
 * 1 = 试车：把所有档位的目标速度压到 LOW_SPEED_TEST_CAP，
 *     并临时关闭"两侧边界全丢 / 视野 5 点全暗"的出界保护
 *     （桌面上没有赛道、摄像头看不到线时也能让车慢慢跑，用来验证电机/舵机/编码器）
 * 正式比赛前必须改回 0 */
#define LOW_SPEED_TEST      0
#define LOW_SPEED_TEST_CAP  150

/* ===== 异常状态检测（faultFlags 位标志，屏幕显示 ERR:<十进制>） =====
 * 包含 PID 异常、编码器掉线与紧急停车，全部在 5 ms 控制中断里判定；
 * 任一异常置位即视为致命故障，立即停车并置 FAULT_STOPPED。 */
#define FAULT_ENC_L_DOWN   0x0002 // 左编码器无反馈（有速度指令但计数为 0 持续 300 ms）
#define FAULT_ENC_R_DOWN   0x0004 // 右编码器无反馈
#define FAULT_PID_SAT      0x0100 // PID 输出长时间打满且速度不达标（堵转 / 欠压 / 轮子卡住）
#define FAULT_PID_REV      0x0200 // PID 输出与编码器反馈方向相反（速度环正反馈，飞车前兆）
#define FAULT_STOPPED      0x0001 // 因异常被强制紧急停车（同时蜂鸣报警）

extern volatile uint16 faultFlags;
void faultReset(void);

/* 蜂鸣器电平：报警时输出 BEEP_ON_LEVEL。
 * 若报警时没有声音（或一直响），把这两个值对调即可。 */
#define BEEP_ON_LEVEL      1
#define BEEP_OFF_LEVEL     0

/* ===== 状态指示：核心板蓝色 LED（本车没有蜂鸣器，用蓝灯代替） =====
 * 编码器反馈异常 / PID 异常时点亮，正常熄灭。
 * 引脚按实际接线改；若亮灭相反，把 ON / OFF 两个值对调。 */
#define STATUS_LED_PIN      IO_P52
/* 该核心板的蓝灯是「阴极接引脚、阳极经电阻接 3.3V」，即 **低电平点亮**：
 * 上电后代码一直写 OFF(0)，灯却常亮，说明 0 才是亮。
 * 若换板后灯变成常灭，把这两个值对调回 1 / 0。 */
#define STATUS_LED_ON       0
#define STATUS_LED_OFF      1

/* ===== 屏幕开关 =====
 * 本车 IPS200 已损坏：置 0 后所有 ips200_* 调用被替换成空操作，
 * 编译期直接去掉显示代码（不影响控制、不影响任何状态机）。
 * 换回好屏时改 1 即可恢复全部显示。 */
#define ENABLE_SCREEN      0

#if (!ENABLE_SCREEN)
/* 展开为空：调用点留下一个空语句 `;`，不产生告警也不产生代码。
 * 注意 ips200_displayimage03x 是库里的宏（展开成 ips200_show_gray_image），
 * 不能在这里重定义，调用点用 #if (ENABLE_SCREEN) 包住。 */
#define ips200_init()
#define ips200_clear(c)
#define ips200_show_string(x, y, s)
#define ips200_show_int16(x, y, v)
#define ips200_show_int32(x, y, v, w)
#define ips200_draw_point(x, y, c)
#endif

#define FORWARD 114
#define CROSS 514
#define R_ROUND 1919
#define L_ROUND 810
#define ALONG_L 88
#define ALONG_R 99

volatile extern int16 setSpeed, handle, encoderL, encoderR;
/* ===== 电机功率硬上限（真正限制"实际速度"的那个） =====
 * MAX_SET_SPEED 限的是「目标速度」，PID 打满时照样能给 100% 占空比；
 * MOTOR_DUTY_LIMIT 直接限制 PID 的输出（= 占空比），无论目标多大、积分怎么积，
 * 电机功率都不会超过这个值。调试期先压到 30%，稳了再逐级提到 6000/8000/10000。 */
#define MOTOR_DUTY_LIMIT   1000   // 10%：目前的最低档，先确认"能有多慢"

/* ===== 起跑助推 =====
 * 发车后先用固定占空比推一段时间，破开静摩擦再交给 PID。
 * 目的：避免"车起不来 → PID 积分打满 → 一推就全速冲出去"。 */
#define MOTOR_KICK_TICKS   60      // 60 × 5 ms = 300 ms
#define MOTOR_KICK_DUTY    2000    // 20%：起跑助推比巡航上限高一档，用来破静摩擦

void motorKickStart(void);  // 由 raceStart() 调用
void servoSelfTest(void);   // 上电舵机自检：左右各摆 SERVO_TEST_SWING（≈15°）
void motorInit(int16 c11, int16 c33, int16 c44, int16 c55, int16 c77);
void setServoDuty(int8 duty);
void alongLine(Point roadLine[], int16 count, int16 aimPos);
int8 getAimPos();
void controlReset(void);      // 发车/急停时清空 PID 历史与控制量
extern volatile int16 enableDifSpeed, currentSpeed;
extern volatile int8 stop;
#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
