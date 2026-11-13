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
const int16 tani[33];
extern int16 RUN_MODE;
extern uint8 switch1, ALONG;
extern volatile uint8 borderIn;
static uint8 DEVICE_H = 18;
volatile int8 slow, stop;
int16 c1, c3, c4, c5; //仅保留实际参与运算的整定参数
volatile int16 R, LSetSpeed, RSetSpeed, enableDifSpeed = 1;
volatile int16 currentSpeed;
int16 currentDuty;
extern volatile int32 odometer;
extern volatile int16 LRoadCount, RRoadCount;

/* ===== 异常状态检测（编码器掉线 + PID 异常 + 紧急停车） ===== */
volatile uint16 faultFlags = 0;
static uint16 graceCnt = 0; // 16 位：宽限期按 5 ms 计，需要超过 255
static uint8 encLostL = 0, encLostR = 0; //编码器无反馈计时
static uint8 satCnt = 0, revCntL = 0, revCntR = 0; //PID 饱和 / 正反馈计时
static uint8 outCnt = 0;                           //出界判定消抖计数
static uint16 kickCnt = 0;                         //起跑助推剩余节拍
static uint16 rampDuty = MOTOR_RAMP_START;         //占空比爬坡当前值（诊断用）
static int16 pidOutL = 0, pidOutR = 0;             //PID 实际输出，供异常检测读取

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
     * 真正的异常是"打满了却跑不起来"（堵转 / 欠压 / 轮子卡住）。 */
    if ((func_abs(baseL) >= 9800 || func_abs(baseR) >= 9800) &&
        ((int32) currentSpeed * 2 < (int32) setSpeed)) {
        if (satCnt < 250) satCnt++;
        if (satCnt >= 200) faultFlags |= FAULT_PID_SAT; // 1 s
    } else
        satCnt = 0;

    /* 正反馈：有明显输出，编码器却往反方向计数。
     * 这是速度环最危险的一种故障（接反 / 编码器 A/B 反），不停车会一路加速。 */
    if (setSpeed > 60 && func_abs(baseL) > 1500 && aL > 5 &&
        ((int32) baseL * (int32) encoderL) < 0) {
        if (revCntL < 250) revCntL++;
        if (revCntL >= 100) faultFlags |= FAULT_PID_REV; // 500 ms（起步时轻微后溜不算）
    } else
        revCntL = 0;
    if (setSpeed > 60 && func_abs(baseR) > 1500 && aR > 5 &&
        ((int32) baseR * (int32) encoderR) < 0) {
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
    base = func_limit(base, pid->max_out);
    pid->base = (int16) base;
    if (pid->which == 0)
        pidOutL = (int16) base; // 左轮
    else if (pid->which == 1)
        pidOutR = (int16) base; // 右轮
    gpio_set_level((gpio_pin_enum) pid->which ? motorDirR : motorDirL, base >= 0);
    return (int16) func_abs(base);
}

static void pitHandler() {
    int32 cntR, cntL, avg;
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
    /* 冲出赛道保护：跑车过程中两侧边界全部丢失，或视野内 5 个采样点全暗。
     * A2：改为连续 3 拍（15 ms）都成立才判定——十字路口 / 环岛中央那片黑区会同时命中
     * 这 5 个点，原来单拍即停，会在过元素时把车"莫名其妙停死"。 */
    if (raceState == RACE_RUNNING || raceState == RACE_BRAKING) {
        if ((!LRoadCount && !RRoadCount) ||
            (mt9v03x_frame[84][95] < threshold && mt9v03x_frame[88][47] < threshold &&
             mt9v03x_frame[87][141] < threshold && mt9v03x_frame[75][120] < threshold &&
             mt9v03x_frame[74][64] < threshold)) {
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
        if (!stop && (raceState == RACE_RUNNING || raceState == RACE_BRAKING))
            raceState = RACE_STOPPED;
        stop = 1;
        pwm_set_duty(PWME_CH3P_PA4, steerMid);
        pwm_set_duty(motorPwmR, motorPIDCalculate(&motorRPID, 0, encoderR));
        pwm_set_duty(motorPwmL, motorPIDCalculate(&motorLPID, 0, encoderL));
    } else if (kickCnt) {
        /* 起跑助推：固定占空比、固定正转，绕过 PID；300 ms 后交给 PID */
        kickCnt--;
        gpio_set_level(motorDirR, 1);
        gpio_set_level(motorDirL, 1);
        pwm_set_duty(motorPwmR, MOTOR_KICK_DUTY);
        pwm_set_duty(motorPwmL, MOTOR_KICK_DUTY);
        pwm_set_duty(PWME_CH3P_PA4, steerMid + steerPIDCalculate(&steerPid, duty));
        if (kickCnt == 0) controlReset(); // 助推结束，PID 从干净状态接管
    } else {
        pwm_set_duty(PWME_CH3P_PA4, steerMid + steerPIDCalculate(&steerPid, duty));
        slow = !borderIn ? 1 : (currentSpeed < 350 ? 0 : slow);
#if (!LOW_SPEED_TEST)
        // 刹车段速度由 raceTask 锁定，环岛入弯减速不能把它抬回去
        if (!handle && raceState == RACE_RUNNING && borderIn && currentSpeed > 420 && slow) setSpeed = 200;
#endif
        RSetSpeed = setSpeed + (!R || !enableDifSpeed ? 0 : setSpeed * 8 / R);
        LSetSpeed = setSpeed - (!R || !enableDifSpeed ? 0 : setSpeed * 8 / R);
        /* 差速后的单轮目标同样限幅，保证「绝对不超过 SPEED_LIMIT」 */
        RSetSpeed = (int16) func_limit_ab((int32) RSetSpeed, 0, SPEED_LIMIT);
        LSetSpeed = (int16) func_limit_ab((int32) LSetSpeed, 0, SPEED_LIMIT);
#if (MOTOR_OPEN_LOOP)
        /* 开环：不使用编码器反馈，方向固定正转 */
        gpio_set_level(motorDirR, 1);
        gpio_set_level(motorDirL, 1);
#if (MOTOR_DUTY_RAMP)
        /* 爬坡诊断：从 10% 开始慢慢加，直到 60% */
        if (rampDuty < MOTOR_RAMP_MAX) rampDuty += MOTOR_RAMP_STEP;
        pwm_set_duty(motorPwmR, rampDuty);
        pwm_set_duty(motorPwmL, rampDuty);
#else
        /* 固定占空比：地面水平，不需要按曲率调速，速度恒定且最慢 */
        pwm_set_duty(motorPwmR, MOTOR_RUN_DUTY);
        pwm_set_duty(motorPwmL, MOTOR_RUN_DUTY);
#endif
#else
        pwm_set_duty(motorPwmR, motorPIDCalculate(&motorRPID, RSetSpeed, encoderR));
        pwm_set_duty(motorPwmL, motorPIDCalculate(&motorLPID, LSetSpeed, encoderL));
#endif
    }
}

void alongLine(Point roadLine[], int16 count, int16 aimPos) {
    int16 pos, dx, dy;
    int32 ax, ay;
    int8 d;
    if (count <= 5) return;
    pos = func_limit_ab(aimPos, 5, count - 2);
    dx = MT9V03X_H - roadLine[pos].x + DEVICE_H;
    dy = 94 - roadLine[pos].y;
    // 16 位平方会溢出；dx=dy=0 时 atan2 会打满舵，这里直接保持上一拍打角
    ay = (int32) 2 * DEVICE_H * dy;
    ax = (int32) dx * dx + (int32) dy * dy;
    if (ax <= 0) return;
    d = (int8) atan2_int(ay, ax);
#if (DEBUG_MODE)
    if (switch1)
        ips200_draw_point(roadLine[pos].y, roadLine[pos].x + MT9V03X_H + 40, RGB565_RED);
#endif
     setServoDuty(func_limit(d, 32));
}

void setServoDuty(int8 d) {
    int16 data t, r, ts;
    int32 data v;
    currentDuty = (int16) d * c1 / 100;
    t = tani[func_limit_ab(func_abs(currentDuty), 0, 32)];
#if (DEBUG_MODE)
    if (switch1) ips200_show_int16(MT9V03X_W / 2, MT9V03X_H + 20, currentDuty);
#endif
    r = !t ? 0 : 20000 / t;
    // if (borderIn) ts = func_min(c5, 400);
    // else ts = c5;
    ts = c5;
    // c4 是有符号标定参数，不能用 uint32 强转；sqrt_int 已支持 int32 全域。
    // 刹车段由 raceTask 锁定目标，这里不再按曲率重新抬高速度
    v = sqrt_int((int32) r * (int32) c4);
    if (!handle && raceState != RACE_BRAKING && c4 > 0) {
        // 稳定优先：先算目标再一次写入 volatile，避免读-改-写被 PIT 插入
        int16 target = !r ? ts : (int16) func_min(v, (int32) ts);
#if (LOW_SPEED_TEST)
        setSpeed = (int16) func_limit_ab((int32) target, 0, LOW_SPEED_TEST_CAP);
#else
        setSpeed = (int16) func_limit_ab((int32) target, 0, SPEED_LIMIT);
#endif
    }
    R = d < 0 ? -r : r;
    duty = (int16) d * 12;
}

int8 getAimPos() {
    return c3;
}

/* ===== 起跑助推 ===== */
void motorKickStart(void) {
    kickCnt = MOTOR_KICK_TICKS;
    rampDuty = MOTOR_RAMP_START;
}

/* 上电舵机自检：中值 → 一侧 15° → 另一侧 15° → 回中，全程约 0.9 s。
 * 用来确认舵机供电、信号线、机械行程是否正常；电机完全不参与。 */
void servoSelfTest(void) {
    int16 i;
    pwm_init(PWME_CH3P_PA4, 200, steerMid);
    system_delay_ms(200);
    for (i = 0; i <= SERVO_TEST_SWING; i += 25) { // 中值 → 一端
        pwm_set_duty(PWME_CH3P_PA4, steerMid - i);
        system_delay_ms(20);
    }
    for (i = SERVO_TEST_SWING; i >= -SERVO_TEST_SWING; i -= 25) { // → 另一端
        pwm_set_duty(PWME_CH3P_PA4, steerMid - i);
        system_delay_ms(20);
    }
    for (i = -SERVO_TEST_SWING; i <= 0; i += 25) { // 回中
        pwm_set_duty(PWME_CH3P_PA4, steerMid - i);
        system_delay_ms(20);
    }
}

void motorInit(int16 c11, int16 c33, int16 c44, int16 c55, int16 c77) {
    c1 = c11;
    c3 = c33;
    c4 = c44;
    c5 = c55;
    if (c77 > 0) DEVICE_H = (uint8) c77; //第7参数：纯跟踪等效轴距，传0则保持默认18
    PIDInit(&steerPid, 3,
             c1, 0, 50,
             395);
    PIDInit(&motorLPID, 0,
            3000, 50, 50,
            MOTOR_DUTY_LIMIT); // 输出上限 = 占空比上限
    PIDInit(&motorRPID, 1,
            3000, 50, 50,
            MOTOR_DUTY_LIMIT);
    pwm_init(PWME_CH3P_PA4, 200, steerMid);
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
    setSpeed = duty = handle = 0;
    R = currentSpeed = currentDuty = 0;
    slow = 0;
    enableDifSpeed = 1;
}
