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
#define LOW_SPEED_TEST      0    // 0 = 出界保护生效（比赛模式）
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

/* 屏幕（IPS200）已从硬件上移除，工程内不再有任何 ips200_* 调用。
 * 若以后加回屏幕，直接调用库函数即可（头文件仍在 zf_common_headfile.h 里）。 */

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
#define MOTOR_DUTY_LIMIT   3100   // 31%：闭环 PID 的占空比硬上限

/* ===== 开环模式：电机不跑速度闭环，占空比直接由目标速度线性映射 =====
 *   1 = 开环（编码器只用于蓝灯指示与里程，不参与控制）
 *   0 = 正常闭环 PID
 * 用途：排除"反馈符号/增益不匹配"这一类问题，先确认「给多少占空比就跑多快」。 */
#define MOTOR_OPEN_LOOP        0
#define MOTOR_DUTY_PER_UNIT    8    // 每 1 单位 setSpeed 对应的占空比（180 × 8 = 1440，低于 16% 上限）

/* ===== 占空比爬坡诊断（临时） =====
 * 设 1 时：发车后占空比从 MOTOR_RAMP_START 开始，每 5 ms 加 MOTOR_RAMP_STEP，
 * 一直加到 MOTOR_RAMP_MAX 为止。用来找"刚好能让轮子转起来"的临界占空比：
 *   从按 B2 到轮子开始转，约经过 t 秒 → 临界值 ≈ 10% + 20% × t
 * 找到之后把本宏改回 0，并用 MOTOR_RUN_DUTY 固定在该值上。 */
#define MOTOR_DUTY_RAMP        0      // 0 = 不做爬坡，直接按 MOTOR_RUN_DUTY 固定输出
#define MOTOR_RAMP_START       1000   // 10%
#define MOTOR_RAMP_MAX         6000   // 60%（够高，保证一定能转）
#define MOTOR_RAMP_STEP        10     // 每 5 ms +0.1%  → 约 20%/秒

/* 开环固定巡航占空比：地面绝对水平、不需要爬坡，直接给一个「刚能稳定转起来」的值。
 * 14%（1400）实测轮子不转，30%（3000）能跑；这里取中间偏下，作为最慢稳定档。
 * 若仍不转 → 2500 / 3000；若嫌快 → 2000。 */
#define MOTOR_RUN_DUTY         1800   // 18%（按用户指定）

/* ===== 电机方向电平 =====
 * DIR = 该电平时车轮向前（与编码器计数为正的方向一致）。
 * 实测：DIR=0 时推车向前、电机反而往后退 → 说明 DIR=1 才是前进。
 * （之前看到的"逆时针"是站在车左侧看左轮，前进本来就是逆时针。） */
/* 开环模式占空比是恒定的、没有 PID 干扰，所以这次观察是干净的：
 * 发车后车往后退 → DIR=1 对应倒转，前进应为 DIR=0。 */
#define MOTOR_DIR_FORWARD      0

/* ===== 起跑助推 =====
 * 发车后先用固定占空比推一段时间，破开静摩擦再交给 PID。
 * 目的：避免"车起不来 → PID 积分打满 → 一推就全速冲出去"。 */
#define MOTOR_KICK_TICKS   0       // 0 = 关闭起跑助推（开环固定占空比不需要它）
#define MOTOR_KICK_DUTY    4500    // 45%：仅起步 300 ms，用来破静摩擦

void motorKickStart(void);  // 由 raceStart() 调用
void motorForceStop(void);  // 强制停车（B3 急停调用，直接清零占空比）
void servoSelfTest(void);   // 上电舵机自检：左右各摆 SERVO_TEST_SWING（≈15°）
void motorInit(int16 c11, int16 c33, int16 c44, int16 c55, int16 c77);
void setServoDuty(int8 duty);
void alongLine(Point roadLine[], int16 count, int16 aimPos);
int8 getAimPos();
void controlReset(void);      // 发车/急停时清空 PID 历史与控制量
extern volatile int16 enableDifSpeed, currentSpeed;
extern volatile int8 stop;
#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
