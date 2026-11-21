//
// Created by 31663 on 2026/3/27.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_MOTOR_H
#include "zf_common_headfile.h"
#include "Image.h"      // 纵向参考行取自视野窗口（VIEW_NEAR_ROW）

/* 目标速度上限，单位是「脉冲 / 5 ms」。
 * 8 档 c5 与弯道限速 sqrt(R·c4) 的结果都由它封顶，是唯一的速度天花板。
 * 换算：上限 = v(m/s) × PULSE_PER_METER / 200。
 * 例：PULSE_PER_METER 实测 34600 时，400 → 2.31 m/s。
 * 需要提速只改这一个宏。 */
#define SPEED_LIMIT        400

/* ===== 起步速度斜坡 =====
 * 发车后前 SPEED_RAMP_MS 内，实际目标速度每个 5 ms 控制拍最多上升
 * SPEED_RAMP_STEP，避免从 0 直接跳到 c5（日志里 300 ms 就到 362）。
 * 只限制“上升”；目标速度下降（入弯、刹车、急停）仍然立即生效。 */
#define SPEED_RAMP_MS      1200   // 斜坡窗口，覆盖发车到第一个弯
#define SPEED_RAMP_STEP      2    // 每 5 ms 允许增加的脉冲/5ms 目标；2 约 1 s 到 400

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
#define BORDER_IN_SPEED_RATIO   40

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

/* ===== 纯跟踪的纵向参考行 =====
 * dx = STEER_REF_ROW - 中线点行 + DEVICE_H。
 * 近端裁掉 20 行以后，中线最靠下的点也在行 99 附近；若仍按画面底边 120 计算，
 * dx 平白多出 20，同一横向偏差算出的转角只有裁切前的 3/4 左右，
 * 实车表现就是"转弯幅度不够、弯道推头"。
 * 取窗口下沿 VIEW_NEAR_ROW 相当于把被裁掉的近端从几何里扣掉，
 * 让 c1 / c3 这套按整幅画面标定的参数继续可用。
 * 想保留更长的前瞻、更柔的转向，把它改回 MT9V03X_H（120）再重调 c1。
 *
 * 注意 DEVICE_H 是双语义参数：dx 里保留"车头到镜头"的几何补偿，
 * 转角分母 ay = 2·DEVICE_H·dy 里是"纯跟踪等效轴距 L"。
 * 当前两处都由 PP_LOOKAHEAD_PX 初始化（28 px），它是图像域整定值；
 * 车体轮距 18.2 cm 不能直接填进来，换摄像头安装后要按实车重新整定。 */
#define STEER_REF_ROW   VIEW_NEAR_ROW

/* ===== 纯跟踪等效轴距（像素，即上面 ay 里的 L） =====
 * 同样的横向偏差 dy、纵向距离 dx，L 越大解出的转角越大：
 *   tan(d) = 2·L·dy / (dx² + dy²)，L 就是这里的值。
 *
 * 18 → 26 的来由（实车"转弯幅度只有一半"的定量核算）：
 *   弯道 R≈26 cm（画面约 80 px），前瞻点横向偏差 dy ≈ dx²/(2R) ≈ 36 px，
 *   dx ≈ 76 px。L=18 时 d = atan(2·18·36/(76²+36²)) ≈ 10.4°，
 *   叠加弯道增益后 ≈16°，而跟住该半径实际需要约 31°——只有一半，
 *   同时 DUTY 才 209（上限 395），说明是增益不足而不是舵机打不动。
 *   L=26 把基础角抬到 ≈15°，配合下面的弯道增益能到 ≈28°，DUTY ≈360 仍有余量。
 *
 * 再往上加（35 / 45）会接近满舵 30° 的硬夹取，直道噪声也被同比例放大，
 * 每加一档都要先确认直道不摆。反过来若直道开始左右摆，就退回 22。
 * 注意它只该影响转角分母，不要把它接进 dx ——那里的一半是摄像头安装几何。
 * 车体参考（2026-11）：全长 29 cm、两轮间距 18.2 cm、轮半径 3.1 cm；
 * 这些物理尺寸用于底盘/里程标定，不直接代入本像素公式。 */
#define PP_LOOKAHEAD_PX     28

/* 纵向距离下限：dx = STEER_REF_ROW - 点行 + DEVICE_H。
 * 十字补线或中线被截短时，前瞻点可能贴到车头行，dx 掉到 18 以下，
 * atan(2·L·dy, dx²+dy²) 会直接打满舵（30° / duty 390）。取 30 保证最坏
 * 情况下的打角仍可控，正常循迹的 dx 常态在 45~90，不受影响。 */
#define PP_DX_MIN           30

/* ===== 前瞻点取法：按下标 还是 按弧长 =====
 * 中线由 resampleRoad(dis=2) 生成，相邻点间距约 2 像素，所以"第 pos 个点"
 * 本身就是一个弧长度量（≈ pos × 2 像素）。两者给出不同的前瞻距离：
 *
 *   0 = 旧行为：pos 先取 clamp(aimPos, 2, count-1)，再被 count-1-BEND_FAR_AHEAD
 *       夹一道。为了给远端点留 12 个点，前瞻距离被永久扣掉 24 像素；
 *       线越短缩得越狠。以 c3=26、满速（aimPos=36）为例：
 *         中线 45 点 → pos=32 → 前瞻 64 像素
 *         中线 20 点 → pos=7  → 前瞻 14 像素（dx 只剩 32，dx²+dy² 让增益翻 3 倍）
 *       同一个弯道中线点数一变，打角就跟着变——这是"同一弯道两次跑法不同、
 *       直道摆、入弯抖"的几何根源，也是 c1 只能折中标定的原因。
 *   1 = 新行为：沿中线累计弧长取点，前瞻距离 = min(aimPos×2, 中线长度) 像素，
 *       不再扣除 12 点。线不够长时前瞻点退到线末（始终落在中线上，不外推），
 *       远端点允许沿末端切线外推（PP_ARC_EXT_MAX）以保证前馈不退化。
 *
 * 两种取法的目标弧长只差那 24 像素，可以直接 A/B。打开后同一个弯道的打角
 * 会明显变小（前瞻变远、增益变低），需要把 c1 或 PP_LOOKAHEAD_PX 加回去；
 * 串口 AIM 字段打印本帧前瞻点行号，切换前后对比这个数即可确认生效。 */
#define PP_ARC_MODE         0
/* 远端点沿末端切线允许外推的最大弧长（像素）。
 * 取 BEND_FAR_AHEAD×2 正好等于常规的"前瞻点→远端点"间距，
 * 短线时前馈量与长线时一致；再大就没有意义了（切线方向在弯里会拐走）。 */
#define PP_ARC_EXT_MAX      24

/* ===== 弯道转向增强 =====
 * 只在弯道减速状态生效：把纯跟踪算出的图像角度按该百分比放大后再送给舵机与曲率限速。
 * 直道保持原增益，避免整体调高 c1 带来的直道摆振。
 * 放大后的角度同时进入 tani 查表，曲率限速会跟着再降一档。 */
#define BEND_STEER_PCT      110   // 缓弯的基础增益，直道与小幅弯不过冲
#define BEND_STEER_MAX      185   // 急弯的增益上限
#define BEND_GAIN_BEND      15    // 强度超过该值开始加增益
#define BEND_GAIN_RATE      6     // 强度每多 1 单位增加的百分点；强度 27.5 到 185

/* ===== 弯道提前减速 =====
 * 触发信号是中线的远端趋势变化量加当前转角，车还没到弯心就能看到弯。
 * bendLevel 超过 BEND_SLOW_START 后，每多 1 单位把目标速度降低
 * BEND_SLOW_GAIN 个百分点，最低降到 BEND_SLOW_MIN。
 * 处于减速状态时再用 BEND_SPEED_CAP 做一次硬限幅，保证入弯初期就慢下来。
 * BORDER_IN_SPEED_RATIO 与这个比例取更严格的一个，两者不会连乘。 */
#define BEND_SLOW_START     4
#define BEND_SLOW_GAIN      3
#define BEND_SLOW_MIN       40
#define BEND_SPEED_CAP      170   // 弯道减速的硬顶；原 200 仍偏快，先降到 170
#define BEND_RELEASE        2     // 弯道强度回落到该值以下才解除减速，出弯前不中途加速

/* 上电舵机自检摆幅：左右各 30°。
 * 200 Hz 下 1 duty = 0.5 µs；按 395 duty ≈ 24° 的行程估算，1° ≈ 16.5 duty，
 * 30° ≈ 495 duty。自检以 steerMid + STEER_TRIM 为中心，结束时回到该位置，
 * 即机械 90°。若机械限位不足 30°，摆到两端会堵转，听到持续响声就调小。 */
#define SERVO_TEST_SWING   495
/* 自检每步的 duty 增量。它必须能整除 SERVO_TEST_SWING，否则三段循环的终止条件
 * 会落在端点之外：原来步长写死 25，495/25 = 19.8，于是第一段最大只到 475、
 * 第二段从 495 直接跳到 -480、第三段到 -20 就退出（i == 0 永不执行），
 * 自检结束后舵机停在离中位 20 duty 的位置。495 = 33 × 15，三段都精确落在
 * +495 / -495 / 0 上。改 SWING 时同步改步长，保持整除关系。 */
#define SERVO_TEST_STEP    33

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

#define FORWARD 114
#define ALONG_L 88
#define ALONG_R 99

/* ===== 元素模块共用的输出量 =====
 * SCurve / Round / XCross 三个模块不直接碰舵机和速度环，只按帧写这两个量，
 * 由 alongLine() / setServoDuty() 统一消费。三个模块之间没有共享状态，
 * 谁在执行就谁写，退出时清回默认值，天然不会互相干扰。
 *   elemSteerBiasDeg  叠加在最终打角上的固定偏置（度，正 = 往右）——环岛用
 *   elemSpeedRatio    目标速度比例（%），100 = 不限——环岛 50、路口 60、S 弯 70
 * 还有一个只读量：
 *   lastSteerDeg      上一帧实际输出的打角，十字路口进弯时读它做"保持朝向"
 */
extern int8  elemSteerBiasDeg;
extern uint8 elemSpeedRatio;
extern int8  lastSteerDeg;

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
