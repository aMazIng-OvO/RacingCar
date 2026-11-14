//
// Created by 31663 on 2026/3/27.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
#include "zf_common_headfile.h"
#define DEBUG_MODE 0

/* 目标速度上限，单位是「脉冲 / 5 ms」。
 * 8 档 c5 与弯道限速 sqrt(R·c4) 的结果都由它封顶，是唯一的速度天花板。
 * 换算：上限 = v(m/s) × PULSE_PER_METER / 200。
 * 例：PULSE_PER_METER 实测 34600 时，400 → 2.31 m/s。
 * 需要提速只改这一个宏。 */
#define SPEED_LIMIT        400

/* ===== 舵机（200 Hz，steerMid = 2975，1 duty = 0.5 µs） =====
 * 单边 395 duty = 197.5 µs 脉宽偏移，实际转角取决于舵机自身的行程标定
 * （常见 0.5 ms/60° 的舵机约 24°）。超过机械限位只会堵转发热，不产生额外转角，
 * 换舵机后按它的行程重设这个值。 */
#define STEER_DUTY_LIMIT   395
/* 舵机换算：duty 偏移 = 图像角度 × c1 / STEER_DUTY_DIV。
 * c1 = 260 时每 1 度图像角度对应 13 duty，图像角度打满 30 度得到 390 duty。
 * c1 就是转向增益，8 档参数里 200~260 对应 10~13 duty/度。 */
#define STEER_DUTY_DIV     20

/* 入弯减速：borderIn = 1（车头已进弯）时把目标速度压到该百分比。
 * 用比例而不是绝对赋值，目标速度不会因此被抬高。 */
#define BORDER_IN_SPEED_RATIO   50

/* ===== 轨迹偏置的标定项 =====
 * 三个偏置互相独立，默认 0 表示不引入额外修正。车持续压一侧边线时按顺序排查：
 *   1. STEER_TRIM：舵机机械中值装偏。正值让舵机输出右偏，负值左偏，
 *      单位是 duty（1 duty = 0.5 µs）。按 395 duty ≈ 24° 估算，1° ≈ 16.5 duty。
 *      逐次实测收敛：右偏 2° → -33，差 3° → -83，差 2° → -116。
 *      停车分支之前漏了本项，读数被 steerMid 覆盖，修好覆盖后 -116 过量，取一半 -58。
 *      `STEER_DUTY_LIMIT` 相对 steerMid + STEER_TRIM 限幅，左右行程仍然对称。
 *   2. TRACK_BIAS_PX：摄像头装偏或支架倾斜。画面中心列 94 对应的不是车头正前方，
 *      纯跟踪拿到的横向偏差带一个恒定分量。正值把瞄准点右移，单位是像素。
 *   3. MOTOR_DIFF_TRIM：左右轮实际速度不对称（电机个体差、轮胎直径差）。
 *      正值让左轮比右轮快，把车往右拉。单位与 setSpeed 一致（脉冲 / 5 ms）。
 * 三者都只影响行驶过程，停车回中用的是 steerMid，不叠加 STEER_TRIM。 */
#define STEER_TRIM       (-58)  // 舵机中值修正（duty）：机械右偏约 3.5°，反向拉到中位
#define TRACK_BIAS_PX      0    // 纯跟踪瞄准点的横向偏置（像素，正值向右）
#define MOTOR_DIFF_TRIM    0    // 左右轮目标速度差补偿（正值左轮快）

/* ===== 弯道转向增强 =====
 * 只在弯道减速状态生效：把纯跟踪算出的图像角度按该百分比放大后再送给舵机与曲率限速。
 * 直道保持原增益，避免整体调高 c1 带来的直道摆振。
 * 放大后的角度同时进入 tani 查表，曲率限速会跟着再降一档。 */
#define BEND_STEER_PCT      105   // 缓弯的基础增益，直道与小幅弯不过冲
#define BEND_STEER_MAX      155   // 急弯的增益上限
#define BEND_GAIN_BEND      15    // 强度超过该值开始加增益
#define BEND_GAIN_RATE      5     // 强度每多 1 单位增加的百分点；强度 25 时正好到 155

/* ===== 弯道提前减速 =====
 * 触发信号是中线的远端趋势变化量加当前转角，车还没到弯心就能看到弯。
 * bendLevel 超过 BEND_SLOW_START 后，每多 1 单位把目标速度降低
 * BEND_SLOW_GAIN 个百分点，最低降到 BEND_SLOW_MIN。
 * 处于减速状态时再用 BEND_SPEED_CAP 做一次硬限幅，保证入弯初期就慢下来。
 * BORDER_IN_SPEED_RATIO 与这个比例取更严格的一个，两者不会连乘。 */
#define BEND_SLOW_START     4
#define BEND_SLOW_GAIN      3
#define BEND_SLOW_MIN       40
#define BEND_SPEED_CAP      200   // 单位与 setSpeed 一致（脉冲 / 5 ms）
#define BEND_RELEASE        2     // 弯道强度回落到该值以下才解除减速，出弯前不中途加速

/* 上电舵机自检摆幅：左右各 30°。
 * 200 Hz 下 1 duty = 0.5 µs；按 395 duty ≈ 24° 的行程估算，1° ≈ 16.5 duty，
 * 30° ≈ 495 duty。自检以 steerMid + STEER_TRIM 为中心，结束时回到该位置，
 * 即机械 90°。若机械限位不足 30°，摆到两端会堵转，听到持续响声就调小。 */
#define SERVO_TEST_SWING   495

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
#define FAULT_OUT          0x0400 // 出界保护触发（两侧边界全丢 / 视野采样点全暗）
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

/* ===== 速度目标的唯一入口 =====
 * setSpeed 只由 5 ms 控制中断写入（从 desiredSpeed 取一个副本），
 * 主循环与 raceTask 一律通过 setSpeedCommand() 写 desiredSpeed，
 * 写入过程关中断，避免 16 位量被控制中断看到半新半旧的值。
 * 速度上限在这里统一夹紧，任何调用者都提交不出超过 SPEED_LIMIT 的目标。 */
extern volatile int16 desiredSpeed;
void setSpeedCommand(int16 v);

/* ===== 全局毫秒节拍（由 5 ms 控制中断累加） =====
 * 主循环里的按键长按、帧看门狗、打印节流全部以它为准，
 * 不再用「循环次数 × system_delay_ms(1)」当时间，避免图像处理耗时把计时拉长。 */
extern volatile uint16 msTick;

/* ===== 出界判定结论（主循环算，控制中断读） =====
 * 出界要看 5 个像素点与两侧边界点数，这些量在控制中断里读会与帧搬运冲突（读到半帧）。
 * 改由主循环在处理完一帧后判定一次，控制中断只对这个结论做消抖。 */
extern volatile uint8 outOfTrack;

/* ===== 电机功率硬上限 =====
 * 速度目标限制的是「跑多快」，这里限制 PID 能给出的最大占空比。
 * 取满量程，让 PID 有足够权限追上 SPEED_LIMIT；
 * 真正的安全边界是 SPEED_LIMIT 与出界保护，不是削功率。 */
#define MOTOR_DUTY_LIMIT   10000

/* 定点停车刹车段允许的最大反向占空比（电子制动）。
 * 速度环在目标为 0、实际仍为正时会输出负占空比，不加这一层限幅，
 * 刹车段会从"减速"变成"倒车"，停车点直接退回去。 */
#define MOTOR_BRAKE_DUTY   1500

/* ===== 开环模式：电机不跑速度闭环，占空比直接由目标速度线性映射 =====
 *   1 = 开环（编码器只用于蓝灯指示与里程，不参与控制）
 *   0 = 正常闭环 PID
 * 用途：排除"反馈符号/增益不匹配"这一类问题，先确认「给多少占空比就跑多快」。 */
#define MOTOR_OPEN_LOOP        0
#define MOTOR_DUTY_PER_UNIT    8    // 每 1 单位 setSpeed 对应的占空比（400 × 8 = 3200，即 32%）

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
void servoSelfTest(void);   // 上电舵机自检：左右各摆 SERVO_TEST_SWING（≈30°），结束回机械 90°
void motorInit(int16 c11, int16 c33, int16 c44, int16 c55, int16 c77);
void setServoDuty(int8 duty);
void alongLine(Point roadLine[], int16 count, int16 aimPos);
int8 getAimPos();
void controlReset(void);      // 发车/急停时清空 PID 历史与控制量
extern volatile int16 enableDifSpeed, currentSpeed;
extern volatile int8 stop;
#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
