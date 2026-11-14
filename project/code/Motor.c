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
const int16 tani[33];
extern uint8 switch1;             // 仅 DEBUG_MODE=1 的画面叠加使用
extern volatile uint8 borderIn;
static uint8 DEVICE_H = 18;
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
    setSpeed = desiredSpeed;   // 本拍唯一一次读取目标速度，之后全部用这个副本
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
 * bendHold 记住本次弯道里出现过的最严比例，弯心两端都偏、bend 收窄时不会中途加速。 */
static uint8 bendBrake = 100;
static uint8 bendHold = 100;

void alongLine(Point roadLine[], int16 count, int16 aimPos) {
    int16 pos, farPos, dx, dy, dyAim, dyFar, bend, ratio;
    int32 ax, ay;
    int8 d;
    /* 原来要求 count > 5，弯道被拐点截断后只剩几个点就直接返回、保持上一拍打角。
     * 中线融合后点数稳定，这里降到 3，点数很少时也能按近端跟随。 */
    if (count < 3) {
        bendBrake = bendHold = 100;
        return;
    }
    pos = func_limit_ab(aimPos, 2, count - 1);
    /* 给远端趋势留出采样余量：前瞻点贴到中线末端时远端点与它重合，
     * 前馈会退化成 0，弯道深处反而失去提前量。 */
    if (count > BEND_FAR_AHEAD + 3)
        pos = func_limit_ab(pos, 2, count - 1 - BEND_FAR_AHEAD);
    dyAim = (int16) (94 - (int16) roadLine[pos].y);
    farPos = func_limit_ab(pos + BEND_FAR_AHEAD, pos, count - 1);
    dyFar = (int16) (94 - (int16) roadLine[farPos].y);
    /* TRACK_BIAS_PX：摄像头装偏或支架倾斜时，画面中心列对应的不是车头正前方，
     * 横向偏差里带一个恒定分量，纯跟踪只能靠稳态偏差去抵消，表现就是车贴着一侧跑。
     * 偏置直接加在最终偏差上，直道与弯道同时生效。 */
    dy = (int16) (dyAim + (dyFar - dyAim) / BEND_FEED_DIV + TRACK_BIAS_PX);
    dx = MT9V03X_H - roadLine[pos].x + DEVICE_H;
    // 16 位平方会溢出；dx=dy=0 时 atan2 会打满舵，这里直接保持上一拍打角
    ay = (int32) 2 * DEVICE_H * dy;
    ax = (int32) dx * dx + (int32) dy * dy;
    if (ax <= 0) {
        bendBrake = bendHold = 100;
        return;
    }
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
#if (DEBUG_MODE)
    if (switch1)
        ips200_draw_point(roadLine[pos].y, roadLine[pos].x + MT9V03X_H + 40, RGB565_RED);
#endif
    setServoDuty(d);
}

void setServoDuty(int8 d) {
    int16 data t, r, ts, slow;
    int32 data v;
    /* 减速量只消费一次：元素函数若直接调用本函数，不会残留上一帧的弯道比例 */
    slow = (int16) bendBrake;
    bendBrake = 100;
    if (borderIn && BORDER_IN_SPEED_RATIO < slow) slow = BORDER_IN_SPEED_RATIO;
    d = (int8) func_limit(d, 30);
    /* 舵机目标偏移：每 1 度图像角度对应 c1 / STEER_DUTY_DIV 个 duty，
     * 打满 30 度得到 390 duty，正好落在 STEER_DUTY_LIMIT 以内。
     * 转向量与横向偏差保持线性，弯道里有比例可言。 */
    currentDuty = (int16) func_limit_ab((int32) d * c1 / STEER_DUTY_DIV,
                                        -STEER_DUTY_LIMIT, STEER_DUTY_LIMIT);
#if (DEBUG_MODE)
    if (switch1) ips200_show_int16(MT9V03X_W / 2, MT9V03X_H + 20, currentDuty);
#endif
    /* 曲率限速的 tan 表索引是「角度本身」，不是放大后的 duty。
     * 原来用 currentDuty 查表，d 一过 13 就固定查 tan(32°)，
     * 不同弯道算出的限速完全一样，曲率限速等于没接上。 */
    t = tani[func_limit_ab(func_abs(d), 0, 32)];
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
            target = (int16) func_min(target, BEND_SPEED_CAP);
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
    return (int8) func_limit_ab(aim, c3, c3 + 20);
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

/* 上电舵机自检：机械中位 → 一侧 30° → 另一侧 30° → 回机械中位，全程约 1.4 s。
 * 用来确认舵机供电、信号线、机械行程是否正常；电机完全不参与。
 * 中心取 steerMid + STEER_TRIM：舵机装偏时电信号中位对应不到机械 90°，
 * 用这个值摆动，结束后停下的位置就是车轮正中。 */
void servoSelfTest(void) {
    int16 i;
    int16 center = (int16) (steerMid + STEER_TRIM);
    servoTestBusy = 1;                 // 自检期间独占舵机，控制中断不再写中值
    pwm_init(PWME_CH3P_PA4, 200, center);
    system_delay_ms(200);
    for (i = 0; i <= SERVO_TEST_SWING; i += 25) { // 机械中位 → 一端
        pwm_set_duty(PWME_CH3P_PA4, center - i);
        system_delay_ms(20);
    }
    for (i = SERVO_TEST_SWING; i >= -SERVO_TEST_SWING; i -= 25) { // → 另一端
        pwm_set_duty(PWME_CH3P_PA4, center - i);
        system_delay_ms(20);
    }
    for (i = -SERVO_TEST_SWING; i <= 0; i += 25) { // 回机械中位
        pwm_set_duty(PWME_CH3P_PA4, center - i);
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

const int16 tani[33] = {0,
                        17,  35,  52,  70,  87,  105, 123, 141, 158, 176,
                        194, 213, 231, 249, 268, 287, 306, 325, 344, 364,
                        384, 404, 424, 445, 466, 488, 510, 532, 554, 577, 600, 628};

void controlReset(void) {
    // 发车/急停时清空 PID 历史，避免上一次的积分与误差继续作用
    steerPid.base = steerPid.last_error = steerPid.llast_error = 0;
    motorLPID.base = motorLPID.last_error = motorLPID.llast_error = 0;
    motorRPID.base = motorRPID.last_error = motorRPID.llast_error = 0;
    setSpeedCommand(0);   // 目标速度只经这一个入口，控制中断下一拍就会读到 0
    duty = handle = 0;
    R = currentDuty = 0;
    bendHold = bendBrake = 100;   // 发车时清掉上一圈弯道的减速保持
    enableDifSpeed = 1;
}
