//
// Created by 31663 on 2026/3/27.
//
#include "Motor.h"

#define motorDirR IO_P77
#define motorPwmR PWMB_CH3_P76
#define motorDirL IO_P75
#define motorPwmL PWMB_CH1_P74
#define steerMid 2975

volatile int16 encoderL, encoderR, setSpeed = 0, handle = 0, duty = 0;
volatile int16 desiredSpeed = 0;
volatile uint16 msTick = 0;
volatile uint8 outOfTrack = 0;
const int16 tani[31];        // 索引域 0~30：atan2_int 的返回值上限就是 ±30
extern volatile uint8 borderIn;
static uint8 DEVICE_H = PP_LOOKAHEAD_PX;   // 初值即纯跟踪等效轴距，motorInit 可覆盖
volatile int8 stop;
int16 c1, c3, c4, c5; //仅保留实际参与运算的整定参数
volatile int16 R, LSetSpeed, RSetSpeed, enableDifSpeed = 1;
volatile int16 currentSpeed;
int16 currentDuty;
extern volatile int32 odometer;

void setSpeedCommand(int16 v) {
    v = (int16) func_limit_ab((int32) v, 0, SPEED_LIMIT);
    interrupt_global_disable();
    desiredSpeed = v;
    interrupt_global_enable();
}

/* ===== 异常状态检测（编码器掉线 + PID 异常 + 紧急停车） ===== */
volatile uint16 faultFlags = 0;
static uint16 graceCnt = 0; // 16 位：宽限期按 5 ms 计，需要超过 255
static uint8 encLostL = 0, encLostR = 0; //编码器无反馈计时
static uint8 satCnt = 0, revCntL = 0, revCntR = 0; //PID 饱和 / 正反馈计时
static uint8 outCnt = 0;                           //出界判定消抖计数
static uint16 kickCnt = 0;                         //起跑助推剩余节拍
static uint16 rampDuty = MOTOR_RAMP_START;         //占空比爬坡当前值（诊断用）
static volatile uint16 speedRampEndMs = 0;         //起步斜坡结束时刻
static int16 pidOutL = 0, pidOutR = 0;             //PID 实际输出，供异常检测读取
static volatile uint8 servoTestBusy = 0;           //舵机自检进行中：控制中断暂时不写舵机

/* 舵机输出的唯一出口：自检期间由自检函数独占，
 * 否则控制中断每 5 ms 把舵机拉回中值，自检看起来就是"舵机不动"。 */
static void steerWrite(int16 d) {
    if (!servoTestBusy) pwm_set_duty(PWME_CH3P_PA4, (uint16) d);
}

void faultReset(void) {
    faultFlags = 0;
    graceCnt = 0;
    encLostL = encLostR = 0;
    satCnt = revCntL = revCntR = 0;
    outCnt = 0;
}

/* 在 5 ms 控制中断里调用：只读编码器与 setSpeed，不改变任何控制量，
 * 唯一的动作是判定为致命故障时置 stop、切到 RACE_STOPPED。 */
static void faultScan(void) {
    int16 aL, aR, baseL, baseR;
    if (raceState != RACE_RUNNING && raceState != RACE_BRAKING) {
        graceCnt = 0;
        encLostL = encLostR = 0;
        satCnt = revCntL = revCntR = 0;
        return;
    }
    if (graceCnt < 400) { // 发车后约 2 s 内不判：PID 起转、齿轮间隙、静摩擦都需要时间
        graceCnt++;
        return;
    }

    aL = func_abs(encoderL);
    aR = func_abs(encoderR);

    /* ---- 编码器掉线：有速度指令，但该轮 300 ms 内一个计数都没有。
     * 断线时 PID 会一路加满而速度恒为 0，必须在它冲起来之前停掉。 ---- */
    /* 只有「确实已经给出较大占空比」才判编码器掉线：
     * 起步爬坡阶段输出还没起来、车本来就没动，那不是编码器故障。 */
    if (setSpeed > 60 && (func_abs(pidOutL) > 2000 || func_abs(pidOutR) > 2000)) {
        if (aL < 3) {
            if (encLostL < 250) encLostL++;
        } else
            encLostL = 0;
        if (aR < 3) {
            if (encLostR < 250) encLostR++;
        } else
            encLostR = 0;
        if (encLostL >= 60) faultFlags |= FAULT_ENC_L_DOWN;
        if (encLostR >= 60) faultFlags |= FAULT_ENC_R_DOWN;
    } else {
        encLostL = encLostR = 0;
    }

    /* ---- PID 异常 ---- */
    baseL = pidOutL; // 增量式 PID 的实际输出（-10000~+10000）
    baseR = pidOutR;

    /* 饱和判定：输出打满 **且** 速度连目标的一半都达不到。
     * 只看输出会误判——大油门起步、上坡、低摩擦时 PID 打满 1 s 是正常现象，
     * 真正的异常是"打满了却跑不起来"（堵转 / 欠压 / 轮子卡住）。
     * 判据里的上限必须与 PID 的实际限幅 MOTOR_DUTY_LIMIT 一致，
     * 否则写成固定的 9800 时，限幅 3100 的工程里这一位永远不可能置起。 */
    if ((func_abs(baseL) >= (MOTOR_DUTY_LIMIT / 100 * 96) ||
         func_abs(baseR) >= (MOTOR_DUTY_LIMIT / 100 * 96)) &&
        ((int32) currentSpeed * 2 < (int32) setSpeed)) {
        if (satCnt < 250) satCnt++;
        if (satCnt >= 200) faultFlags |= FAULT_PID_SAT; // 1 s
    } else
        satCnt = 0;

    /* 正反馈：给的是正向输出，编码器却往反方向计数。
     * 这是速度环最危险的一种故障（电机线接反 / 编码器 A/B 反），不停车会一路加速。
     * 只判「正输出 + 负反馈」这一个方向：负输出 + 正反馈是正常的减速制动，
     * 原来的乘积判据把它也算进来，弯道减速持续半秒就会误报停车。 */
    if (setSpeed > 60 && baseL > 1500 && encoderL < -5) {
        if (revCntL < 250) revCntL++;
        if (revCntL >= 100) faultFlags |= FAULT_PID_REV; // 500 ms（起步时轻微后溜不算）
    } else
        revCntL = 0;
    if (setSpeed > 60 && baseR > 1500 && encoderR < -5) {
        if (revCntR < 250) revCntR++;
        if (revCntR >= 100) faultFlags |= FAULT_PID_REV;
    } else
        revCntR = 0;

    /* 任一项异常都是致命故障：立即紧急停车 */
    if (faultFlags & (FAULT_ENC_L_DOWN | FAULT_ENC_R_DOWN | FAULT_PID_SAT | FAULT_PID_REV)) {
        faultFlags |= FAULT_STOPPED;
        stop = 1;
        if (raceState == RACE_RUNNING || raceState == RACE_BRAKING) raceState = RACE_STOPPED;
    }
}

typedef struct {
    int16 Kp;
    int16 Ki;
    int16 Kd;
    uint8 which;
    int16 last_error;
    int16 llast_error;
    int16 base;
    int16 max_out;
    int16 min_out;   // 负向限幅：刹车段收窄，避免目标 0 时把车反向驱动
} PID_t;

PID_t steerPid, motorLPID, motorRPID;

void PIDInit(PID_t *pid, uint8 which, int16 Kp, int16 Ki, int16 Kd, int16 max_out) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->last_error = 0;
    pid->llast_error = 0;
    pid->base = 0;
    pid->which = which;
    pid->max_out = max_out;
    pid->min_out = (int16) -max_out;
}

int16 steerPIDCalculate(PID_t *pid, int16 error) {
    int32 p_term, d_term;
    int16 pid_output;
    p_term = (int32) pid->Kp * error;
    d_term = (int32) pid->Kd * (error - pid->last_error);
    pid->last_error = error;
    pid_output = (p_term + d_term) / 100;
    return func_limit(pid_output, pid->max_out);
}

int16 motorPIDCalculate(PID_t *pid, int16 target, int16 actual) {
    int32 err, delta_err, delta_u, base;
    err = target - actual;
    delta_err = err - pid->last_error;
    delta_u = (int32) pid->Kp * delta_err +
                 (int32) pid->Ki * err +
                 (int32) pid->Kd * (delta_err - (pid->last_error - pid->llast_error));
    pid->llast_error = pid->last_error;
    pid->last_error = err;
    // 先在 32 位上累加再限幅；原来 int16 base 先溢出会导致方向翻转和负占空比
    base = (int32) pid->base + delta_u / 100;
    base = func_limit_ab(base, pid->min_out, pid->max_out);
    pid->base = (int16) base;
    if (pid->which == 0)
        pidOutL = (int16) base; // 左轮
    else if (pid->which == 1)
        pidOutR = (int16) base; // 右轮
    gpio_set_level((gpio_pin_enum) pid->which ? motorDirR : motorDirL,
                   (base >= 0) ? MOTOR_DIR_FORWARD : (uint8) (!MOTOR_DIR_FORWARD));
    return (int16) func_abs(base);
}

static void pitHandler() {
    int32 cntR, cntL, avg;
    int16 diff;
    msTick += 5;               // 全局毫秒节拍：按键长按、帧看门狗都以它为基准
    /* 起步斜坡：只限制目标速度的上升沿。
     * 下降沿（入弯、刹车、急停）立即跟随 desiredSpeed，避免减速也被拖延。 */
    if (desiredSpeed > setSpeed &&
        (int16) (speedRampEndMs - msTick) > 0) {
        setSpeed = (int16) (setSpeed + SPEED_RAMP_STEP);
        if (setSpeed > desiredSpeed) setSpeed = desiredSpeed;
    } else {
        setSpeed = desiredSpeed;
    }
    // 全程 32 位：C251 的 int 只有 16 位，负数左移与两轮求和都会溢出
    cntR = -(int32) encoder_get_count(PWMA_ENCODER) * 2;
    cntL = (int32) encoder_get_count(PWMC_ENCODER) * 2;
    encoder_clear_count(PWMA_ENCODER);
    encoder_clear_count(PWMC_ENCODER);
    encoderR = (int16) func_limit(cntR, 32767);
    encoderL = (int16) func_limit(cntL, 32767);
    avg = (cntR + cntL) / 2;
    currentSpeed = (int16) func_limit(avg, 32767);
    odometer += avg;   // IDLE 下同样累加，供推车标定一圈里程
    faultScan();       // 异常状态检测（只读，不干预控制，除致命故障外）
    /* 冲出赛道保护：结论由主循环在处理完一帧后给出（那时快照不会被搬运覆盖），
     * 这里只对结论做连续 3 拍（15 ms）消抖——十字路口 / 环岛中央那片黑区
     * 会让采样点同时变暗，单拍即停会在过元素时把车"莫名其妙停死"。 */
    if (raceState == RACE_RUNNING || raceState == RACE_BRAKING) {
        if (outOfTrack) {
            if (outCnt < 10) outCnt++;
        } else
            outCnt = 0;
    } else
        outCnt = 0;

    if (stop
#if (!LOW_SPEED_TEST)
        || (outCnt >= 3)                  // 出界消抖：连续 15 ms 成立
#endif
       ) {
        // 出界保护必须与比赛状态同步，否则 raceTask 永远等不到里程且 TOP 键无法恢复
        if (!stop && (raceState == RACE_RUNNING || raceState == RACE_BRAKING)) {
#if (!LOW_SPEED_TEST)
            if (outCnt >= 3) faultFlags |= FAULT_OUT; // 出界与正常完赛分开记录
#endif
            raceState = RACE_STOPPED;
        }
        stop = 1;
        /* 停车也回到机械中位，而不是电信号中位 steerMid。
         * 之前这里写 steerMid，自检结束后控制中断立刻把舵机拉回未修正的位置，
         * STEER_TRIM 只在行驶中生效，看起来就是"怎么调都差同一个角度"。 */
        steerWrite(steerMid + STEER_TRIM);
        /* 停车分支直接给 0，不再经过 PID（开环模式下 PID 可能还留着上次的 base） */
        pwm_set_duty(motorPwmR, 0);
        pwm_set_duty(motorPwmL, 0);
        kickCnt = 0;
    } else if (kickCnt) {
        /* 起跑助推：固定占空比、固定正转，绕过 PID；300 ms 后交给 PID */
        kickCnt--;
        gpio_set_level(motorDirR, MOTOR_DIR_FORWARD);
        gpio_set_level(motorDirL, MOTOR_DIR_FORWARD);
        pwm_set_duty(motorPwmR, MOTOR_KICK_DUTY);
        pwm_set_duty(motorPwmL, MOTOR_KICK_DUTY);
        steerWrite(steerMid + STEER_TRIM + steerPIDCalculate(&steerPid, duty));
        if (kickCnt == 0) controlReset(); // 助推结束，PID 从干净状态接管
    } else {
        steerWrite(steerMid + STEER_TRIM + steerPIDCalculate(&steerPid, duty));
        /* 差速：曲率半径 |R| 越小，内外轮速差越大。
         * 差速量按目标速度的比例限幅（最多一半），再把单轮目标夹到 SPEED_LIMIT，
         * 避免急弯时一侧被上限截断、另一侧继续加速造成的转向突变。
         * MOTOR_DIFF_TRIM 叠加在差速之上：正值让左轮更快，把右偏不足的车往右拉。 */
        if (R && enableDifSpeed) {
            diff = (int16) ((int32) setSpeed * 8 / R);
            diff = (int16) func_limit(diff, setSpeed / 2);
            RSetSpeed = (int16) func_limit_ab((int32) setSpeed + diff - MOTOR_DIFF_TRIM, 0, SPEED_LIMIT);
            LSetSpeed = (int16) func_limit_ab((int32) setSpeed - diff + MOTOR_DIFF_TRIM, 0, SPEED_LIMIT);
        } else {
            RSetSpeed = (int16) func_limit_ab((int32) setSpeed - MOTOR_DIFF_TRIM, 0, SPEED_LIMIT);
            LSetSpeed = (int16) func_limit_ab((int32) setSpeed + MOTOR_DIFF_TRIM, 0, SPEED_LIMIT);
        }
        /* 刹车段的负向限幅要收窄：目标速度已经是 0，实际速度仍为正，
         * PID 会持续输出负占空比，限幅还是 -10000 时车会从"减速"变成"倒车"，
         * 定点停车反而退回去。只保留少量反向电流做电子制动。 */
        if (raceState == RACE_BRAKING) {
            motorLPID.min_out = -MOTOR_BRAKE_DUTY;
            motorRPID.min_out = -MOTOR_BRAKE_DUTY;
        } else {
            motorLPID.min_out = -MOTOR_DUTY_LIMIT;
            motorRPID.min_out = -MOTOR_DUTY_LIMIT;
        }
#if (MOTOR_OPEN_LOOP)
        /* 开环：不使用编码器反馈，方向固定正转 */
        gpio_set_level(motorDirR, MOTOR_DIR_FORWARD);
        gpio_set_level(motorDirL, MOTOR_DIR_FORWARD);
#if (MOTOR_DUTY_RAMP)
        /* 爬坡诊断：从 10% 开始慢慢加，直到 60% */
        if (rampDuty < MOTOR_RAMP_MAX) rampDuty += MOTOR_RAMP_STEP;
        pwm_set_duty(motorPwmR, rampDuty);
        pwm_set_duty(motorPwmL, rampDuty);
#else
        /* 固定占空比：地面水平，不需要按曲率调速。
         * 必须按比赛状态门控——否则上电（IDLE）时也会直接输出，车会自己跑。 */
        pwm_set_duty(motorPwmR, (raceState == RACE_RUNNING || raceState == RACE_BRAKING) ? MOTOR_RUN_DUTY : 0);
        pwm_set_duty(motorPwmL, (raceState == RACE_RUNNING || raceState == RACE_BRAKING) ? MOTOR_RUN_DUTY : 0);
#endif
#else
        pwm_set_duty(motorPwmR, motorPIDCalculate(&motorRPID, RSetSpeed, encoderR));
        pwm_set_duty(motorPwmL, motorPIDCalculate(&motorLPID, LSetSpeed, encoderL));
#endif
    }
}

/* 弯道前馈：中线远端点比前瞻点偏得更多时，说明前方正在入弯。
 * 把这段差值按比例叠加进横向偏差，纯跟踪公式会在车到达弯心之前给出转向角。
 * 直道上两端偏差接近，前馈接近 0，不影响直线稳定性。
 * 中线重采样间距 2 像素，BEND_FAR_AHEAD = 12 相当于再看远约 24 像素。 */
#define BEND_FAR_AHEAD   12
#define BEND_FEED_DIV    3

/* 本帧的弯道减速比例，100 = 不减速。由 alongLine() 计算、setServoDuty() 消费一次。
 * 元素状态机也走 alongLine()，所以减速在十字与环岛阶段同样生效。
 * 多帧记忆在 bendHold：它记住本次弯道里出现过的最严比例，弯心两端都偏、
 * bend 收窄时不会中途加速；bendBrake 只是"本帧给 setServoDuty 的快照"，
 * 后者读完立刻把它复位成 100，所以看起来像单帧量——真正的记忆别去 bendBrake 里找。 */
static uint8 bendBrake = 100;
static uint8 bendHold = 100;

/* 本帧前瞻点所在行，只给串口 AIM 字段观测用：
 * 两种取法（PP_ARC_MODE）切换后，靠它确认同一弯道的前瞻行不再随中线点数跳。 */
int16 aimRowDbg = 0;

/* 元素模块的三个输出量（见 Motor.h）。默认值 = 不干预。 */
int8  elemSteerBiasDeg = 0;
uint8 elemSpeedRatio = 100;
int8  lastSteerDeg = 0;

#if (PP_ARC_MODE)
/* 沿中线累计弧长取两个点：aim 在 aimArc 处（线不够长就退到最后一个点，
 * 前瞻点始终落在中线上、不外推到画面外），far 再远 BEND_FAR_AHEAD 个点距。
 * far 允许沿末端切线外推最多 PP_ARC_EXT_MAX 像素：短线时若让 far 也退到线末，
 * dyFar 会等于 dyAim，弯道前馈直接变成 0，等于把这条通路关掉。 */
/* 注意形参名不能叫 far：C251 里 far / near / data / xdata 都是内存空间关键字。 */
static void alongAimPoints(Point line[], int16 count, int16 aimArc, Point *aimPt, Point *farPt) {
    int16 i, acc, dn, rem, farArc, sx, sy;
    int16 lastSx = 0, lastSy = 0, lastDn = 0;
    uint8 gotAim = 0, gotFar = 0;

    *aimPt = line[0];
    *farPt = line[0];
    farArc = (int16) (aimArc + BEND_FAR_AHEAD * 2);
    acc = 0;
    for (i = 1; i < count; i++) {
        sx = (int16) line[i].x - (int16) line[i - 1].x;
        sy = (int16) line[i].y - (int16) line[i - 1].y;
        dn = (int16) sqrt_int((int32) sx * sx + (int32) sy * sy);
        if (!dn) continue;                       // 重采样可能有重复点
        if (!gotAim && acc + dn >= aimArc) {
            rem = (int16) (aimArc - acc);
            aimPt->x = (uint8) ((int16) line[i - 1].x + (int16) ((int32) sx * rem / dn));
            aimPt->y = (uint8) ((int16) line[i - 1].y + (int16) ((int32) sy * rem / dn));
            gotAim = 1;
        }
        if (acc + dn >= farArc) {
            rem = (int16) (farArc - acc);
            farPt->x = (uint8) ((int16) line[i - 1].x + (int16) ((int32) sx * rem / dn));
            farPt->y = (uint8) ((int16) line[i - 1].y + (int16) ((int32) sy * rem / dn));
            gotFar = 1;
            break;
        }
        acc = (int16) (acc + dn);
        lastSx = sx;
        lastSy = sy;
        lastDn = dn;
    }
    if (!gotFar) {
        /* 线不够长：从 aim 点沿末端切线外推。
         * 已经取到 aim 时剩下的距离恒为 BEND_FAR_AHEAD×2，不必再算累计弧长。 */
        if (!gotAim) *aimPt = line[count - 1];
        rem = gotAim ? (int16) (BEND_FAR_AHEAD * 2) : (int16) (farArc - acc);
        if (rem > PP_ARC_EXT_MAX) rem = PP_ARC_EXT_MAX;
        if (lastDn) {
            farPt->x = (uint8) func_limit_ab((int32) aimPt->x + (int32) lastSx * rem / lastDn,
                                             0, MT9V03X_H - 1);
            farPt->y = (uint8) func_limit_ab((int32) aimPt->y + (int32) lastSy * rem / lastDn,
                                             0, MT9V03X_W - 1);
        } else {
            *farPt = *aimPt;
        }
    }
}
#endif

void alongLine(Point roadLine[], int16 count, int16 aimPos) {
    int16 pos, dx, dy, dyAim, dyFar, bend, ratio;
    int16 aimRow, aimCol;
    int32 ax, ay;
    int8 d;
    /* 原来要求 count > 5，弯道被拐点截断后只剩几个点就直接返回、保持上一拍打角。
     * 中线融合后点数稳定，这里降到 3，点数很少时也能按近端跟随。 */
    if (count < 3) {
        bendBrake = bendHold = 100;
        return;
    }
    pos = func_limit_ab(aimPos, 2, count - 1);
#if (PP_ARC_MODE)
    /* 弧长取点：pos 只当作"目标弧长 = pos×2 像素"的索引，不再被中线长度夹逼。
     * 线短时由 alongAimPoints() 沿切线外推，前瞻距离与打角增益都不随点数变化。 */
    {
        Point aimPt, farPt;
        alongAimPoints(roadLine, count, (int16) (pos * 2), &aimPt, &farPt);
        aimRow = (int16) aimPt.x;
        aimCol = (int16) aimPt.y;
        dyAim = (int16) (94 - aimCol);
        dyFar = (int16) (94 - (int16) farPt.y);
    }
#else
    /* 给远端趋势留出采样余量：前瞻点贴到中线末端时远端点与它重合，
     * 前馈会退化成 0，弯道深处反而失去提前量。
     * 代价是线短时 pos 被压到 count-1-BEND_FAR_AHEAD，前瞻点往车头缩
     * （见 Motor.h 的 PP_ARC_MODE，那是另一种取法）。 */
    if (count > BEND_FAR_AHEAD + 3)
        pos = func_limit_ab(pos, 2, count - 1 - BEND_FAR_AHEAD);
    aimRow = (int16) roadLine[pos].x;
    aimCol = (int16) roadLine[pos].y;
    dyAim = (int16) (94 - aimCol);
    {
        int16 farPos = func_limit_ab(pos + BEND_FAR_AHEAD, pos, count - 1);
        dyFar = (int16) (94 - (int16) roadLine[farPos].y);
    }
#endif
    aimRowDbg = aimRow;
    /* TRACK_BIAS_PX：摄像头装偏或支架倾斜时，画面中心列对应的不是车头正前方，
     * 横向偏差里带一个恒定分量，纯跟踪只能靠稳态偏差去抵消，表现就是车贴着一侧跑。
     * 偏置直接加在最终偏差上，直道与弯道同时生效。 */
    dy = (int16) (dyAim + (dyFar - dyAim) / BEND_FEED_DIV + TRACK_BIAS_PX);
    dx = STEER_REF_ROW - aimRow + DEVICE_H;
    /* 前瞻点贴到车头行时 dx 会掉到 18 以下，atan 直接打满舵；给一个下限兜住。
     * 十字补线终点落在车头行、被截短的中线都可能触发这种极端几何。 */
    if (dx < PP_DX_MIN) dx = PP_DX_MIN;
    /* 16 位平方会溢出，必须先提升到 32 位。
     * dx 上面已经夹到 ≥ PP_DX_MIN，所以 ax ≥ PP_DX_MIN² = 900 恒成立，
     * 不需要再判 ax <= 0（那个分支数学上不可达，只会让人以为存在 dx=0 的路径）。 */
    ay = (int32) 2 * DEVICE_H * dy;
    ax = (int32) dx * dx + (int32) dy * dy;
    d = (int8) atan2_int(ay, ax);
    /* 弯道强度：远端趋势变化量 + 当前转角。
     * 前瞻点还在直道、远端已进弯时前一项先起作用，减速比 borderIn 早得多；
     * 车进弯后远端与前瞻点的差值收窄，转角那一项接力，减速不会中途松开。 */
    bend = (int16) (func_abs((int16) (dyFar - dyAim)) + func_abs((int16) d));
    if (bend <= BEND_RELEASE)
        bendHold = 100;                     // 出弯：解除保持
    else if (bend > BEND_SLOW_START) {
        ratio = 100 - (bend - BEND_SLOW_START) * BEND_SLOW_GAIN;
        ratio = func_limit_ab(ratio, BEND_SLOW_MIN, 100);
        if (ratio < bendHold) bendHold = (uint8) ratio;
    }
    /* 起始阈值与解除阈值之间保留上一帧的比例，避免阈值附近来回跳 */
    bendBrake = bendHold;
    /* 弯道转向增强：只在减速状态放大转角，直道保持原增益。
     * 放大后的 d 同时进入 currentDuty 与曲率限速的 tan 表，
     * 转向更狠的同时基础目标速度也跟着再降一档。
     * bend 用的是放大前的角度，减速触发阈值不受增益影响。 */
    if (bendBrake < 100) {
        /* 增益随弯道强度自适应。固定百分比只能照顾一种弯：加到 115% 时缓弯过冲，
         * 降回 105% 时急弯又转不进去。这里让强度 15 以下保持基础增益，
         * 之后每多 1 单位加 5 个百分点，强度 25 到达 155% 封顶。 */
        int16 pct = BEND_STEER_PCT;
        if (bend > BEND_GAIN_BEND)
            pct += (bend - BEND_GAIN_BEND) * BEND_GAIN_RATE;
        pct = func_limit_ab(pct, BEND_STEER_PCT, BEND_STEER_MAX);
        d = (int8) func_limit_ab((int32) d * pct / 100, -30, 30);
    }
    setServoDuty(d);
}

void setServoDuty(int8 d) {
    /* 注意：变量名不能叫 cap —— C251 里 cap 是保留字（和 far / near / data 同类），
     * 写成 int16 cap 会报 "syntax error near '4'"，错误位置还指在别的行上，很难查。 */
    int16 data t, r, ts, slow, speedCap;
    int32 data v;
    /* 减速量只消费一次：元素函数若直接调用本函数，不会残留上一帧的弯道比例 */
    slow = (int16) bendBrake;
    bendBrake = 100;
    /* BEND_SPEED_CAP(200) 只是"弯道减速刚起步时别以直道速度冲进弯"的兜底，
     * 只该跟弯道减速那一路绑定。以前它对所有 slow < 100 的情况都生效，
     * 结果元素模块按比例压出来的速度又被统一砍到 200：
     * S 弯想跑 70%（280）会变成 200，十字 60%（240）也变成 200，
     * 三个模块的比例参数实际上只有"要不要触发 200 上限"这一个含义。
     * 现在把 cap 绑在 bendBrake 上，元素比例按原本的百分比走。 */
    speedCap = (slow < 100) ? BEND_SPEED_CAP : 0x7FFF;
    if (borderIn && BORDER_IN_SPEED_RATIO < slow) slow = BORDER_IN_SPEED_RATIO;
    /* 元素模块的速度比例（环岛 50、十字路口 60、S 弯 70）：与 borderIn 取更严者，
     * 不做连乘，避免两个限速叠成 25% 把车拖停。 */
    if (elemSpeedRatio < slow) slow = elemSpeedRatio;
    d = (int8) func_limit(d, 30);
    /* 环岛状态机只写 elemSteerBiasDeg，若这里不读，状态机进了环岛、
     * 舵机却没有任何入环角。偏置同时作为该方向的最低打角：入口视觉中线
     * 偏向直行时，纯相加会被反向视觉转角抵消，表现为"车头偏过去又回正"。 */
    d = (int8) func_limit_ab((int32) d + (int32) elemSteerBiasDeg, -30, 30);
    if (elemSteerBiasDeg > 0 && d < elemSteerBiasDeg) d = elemSteerBiasDeg;
    else if (elemSteerBiasDeg < 0 && d > elemSteerBiasDeg) d = elemSteerBiasDeg;
    lastSteerDeg = d;      // 十字路口用它做"保持进入时的朝向"
    /* 舵机目标偏移：每 1 度图像角度对应 c1 / STEER_DUTY_DIV 个 duty，
     * 打满 30 度得到 390 duty，正好落在 STEER_DUTY_LIMIT 以内。
     * 转向量与横向偏差保持线性，弯道里有比例可言。 */
    currentDuty = (int16) func_limit_ab((int32) d * c1 / STEER_DUTY_DIV,
                                        -STEER_DUTY_LIMIT, STEER_DUTY_LIMIT);
    /* 曲率限速的 tan 表索引是「角度本身」，不是放大后的 duty。
     * 原来用 currentDuty 查表，d 一过 13 就固定查 tan(32°)，
     * 不同弯道算出的限速完全一样，曲率限速等于没接上。
     * 索引直接取 |d|：上面已把 d 夹到 ±30，表长也是 31，不需要再夹一次。 */
    t = tani[func_abs(d)];
    r = !t ? 0 : 20000 / t;
    ts = c5;
    // c4 是有符号标定参数，不能用 uint32 强转；sqrt_int 已支持 int32 全域。
    // 刹车段由 raceTask 锁定目标，这里不再按曲率重新抬高速度
    v = sqrt_int((int32) r * (int32) c4);
    if (!handle && raceState != RACE_BRAKING && c4 > 0) {
        // 先算目标再一次提交，避免读-改-写被控制中断插入
        int16 target = !r ? ts : (int16) func_min(v, (int32) ts);
        /* 先按比例压，再用弯道硬限幅兜住入弯初期：
         * 那时比例还接近 100%，没有 cap 的话车会以接近直道的速度冲进弯。 */
        if (slow < 100) {
            target = (int16) ((int32) target * slow / 100);
            target = (int16) func_min(target, speedCap);   // 只有弯道减速那一路有 200 硬顶
        }
#if (LOW_SPEED_TEST)
        target = (int16) func_min(target, LOW_SPEED_TEST_CAP);
#endif
        setSpeedCommand(target);
    }
    R = d < 0 ? -r : r;
    duty = currentDuty;
}

int8 getAimPos() {
    /* 前瞻随速度自适应：速度越高看得越远，进弯后目标速度下降、前瞻自动收近。
     * 固定用 c3 时，高速下前瞻不足会来回摆，低速下又反应迟钝。
     * 中线重采样间距 2 像素，c3 = 24 即约 48 像素前瞻；
     * currentSpeed 是「脉冲 / 5 ms」，满速 400 时额外加 10 个点（约 20 像素）。 */
    int16 aim = (int16) c3 + (int16) (currentSpeed / 40);
    /* 上限按速度天花板推导：SPEED_LIMIT / 40 = 10，满速时正好用满。
     * 原来写死 c3 + 20，要 800 脉冲 / 5 ms 才顶得到，等于恒不生效。 */
    return (int8) func_limit_ab(aim, c3, c3 + SPEED_LIMIT / 40);
}

/* 强制停车：直接把两路占空比清零、取消助推，不依赖 PID 状态。
 * 由 B3 急停调用，保证"按了必停"。 */
void motorForceStop(void) {
    stop = 1;
    kickCnt = 0;
    pwm_set_duty(motorPwmR, 0);
    pwm_set_duty(motorPwmL, 0);
    gpio_set_level(motorDirR, 0);
    gpio_set_level(motorDirL, 0);
}

/* ===== 起跑助推 ===== */
void motorKickStart(void) {
    kickCnt = MOTOR_KICK_TICKS;
    rampDuty = MOTOR_RAMP_START;
}

/* 上电舵机自检：机械中位 → 一侧 30° → 另一侧 30° → 回机械中位，全程约 1.5 s。
 * 用来确认舵机供电、信号线、机械行程是否正常；电机完全不参与。
 * 中心取 steerMid + STEER_TRIM：舵机装偏时电信号中位对应不到机械 90°，
 * 用这个值摆动，结束后停下的位置就是车轮正中。
 * 步长必须整除摆幅（见 Motor.h 的 SERVO_TEST_STEP），否则三段都停不到端点上。 */
void servoSelfTest(void) {
    int16 i;
    int16 center = (int16) (steerMid + STEER_TRIM);
    servoTestBusy = 1;                 // 自检期间独占舵机，控制中断不再写中值
    pwm_init(PWME_CH3P_PA4, 200, center);
    system_delay_ms(200);
    for (i = 0; i <= SERVO_TEST_SWING; i += SERVO_TEST_STEP) { // 机械中位 → 一端
        pwm_set_duty(PWME_CH3P_PA4, (uint16) (center - i));
        system_delay_ms(20);
    }
    for (i = SERVO_TEST_SWING; i >= -SERVO_TEST_SWING; i -= SERVO_TEST_STEP) { // → 另一端
        pwm_set_duty(PWME_CH3P_PA4, (uint16) (center - i));
        system_delay_ms(20);
    }
    for (i = -SERVO_TEST_SWING; i <= 0; i += SERVO_TEST_STEP) { // 回机械中位
        pwm_set_duty(PWME_CH3P_PA4, (uint16) (center - i));
        system_delay_ms(20);
    }
    servoTestBusy = 0;
}

void motorInit(int16 c11, int16 c33, int16 c44, int16 c55, int16 c77) {
    c1 = c11;
    c3 = c33;
    c4 = c44;
    c5 = c55;
    if (c77 > 0) DEVICE_H = (uint8) c77; //第7参数：纯跟踪等效轴距，传0则保持默认18
    /* 转向环：currentDuty 里已经含了 c1 增益，这里的 Kp 只做直通（100 = 1:1），
     * Kd 提供阻尼，抑制中线抖动直接变成打角抖振。 */
    PIDInit(&steerPid, 3,
             100, 0, 100,
             STEER_DUTY_LIMIT);
    PIDInit(&motorLPID, 0,
             1000, 50, 0,
             MOTOR_DUTY_LIMIT); // 输出上限 = 占空比上限
    PIDInit(&motorRPID, 1,
             1000, 50, 0,
             MOTOR_DUTY_LIMIT);
    pwm_init(PWME_CH3P_PA4, 200, steerMid + STEER_TRIM); // 初始化即停在机械中位
    gpio_init(motorDirR, GPO, 1, GPO_PUSH_PULL);
    gpio_init(motorDirL, GPO, 1, GPO_PUSH_PULL);
    pwm_init(motorPwmR, 17000, 0);
    pwm_init(motorPwmL, 17000, 0);
    encoder_quad_init(PWMA_ENCODER, PWMA_ENCODER_CH1P_P60, PWMA_ENCODER_CH2P_P62);
    encoder_quad_init(PWMC_ENCODER, PWMC_ENCODER_CH1P_P40, PWMC_ENCODER_CH2P_P42);
    system_delay_ms(200);
    pit_ms_init(TIM1_PIT, 5, pitHandler);
}

/* tan(角度) × 1000，索引就是图像角度。只到 30 度：atan2_int() 的返回值上限是 ±30，
 * alongLine() 与 setServoDuty() 又各自夹过一次，原表的 31、32 两档永远用不到。 */
const int16 tani[31] = {0,
                        17,  35,  52,  70,  87,  105, 123, 141, 158, 176,
                        194, 213, 231, 249, 268, 287, 306, 325, 344, 364,
                        384, 404, 424, 445, 466, 488, 510, 532, 554, 577};

void controlReset(void) {
    // 发车/急停时清空 PID 历史，避免上一次的积分与误差继续作用
    steerPid.base = steerPid.last_error = steerPid.llast_error = 0;
    motorLPID.base = motorLPID.last_error = motorLPID.llast_error = 0;
    motorRPID.base = motorRPID.last_error = motorRPID.llast_error = 0;
    setSpeedCommand(0);   // 目标速度只经这一个入口，控制中断下一拍就会读到 0
    /* 发车从 0 开始爬坡，并把斜坡窗口从本次发车时刻重新计时。
     * 两个 16 位量都在关中断下写，避免控制中断读到半新半旧的值。 */
    interrupt_global_disable();
    setSpeed = 0;
    speedRampEndMs = (uint16) (msTick + SPEED_RAMP_MS);
    interrupt_global_enable();
    duty = handle = 0;
    R = currentDuty = 0;
    bendHold = bendBrake = 100;   // 发车时清掉上一圈弯道的减速保持
    enableDifSpeed = 1;
}
