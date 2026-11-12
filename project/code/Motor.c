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

/* ===== 异常状态检测 ===== */
volatile uint8 faultFlags = 0;
static uint8 encLostL = 0, encLostR = 0, stallCnt = 0, graceCnt = 0;

void faultReset(void) {
    faultFlags = 0;
    encLostL = encLostR = stallCnt = graceCnt = 0;
}

/* 在 5 ms 控制中断里调用：只读编码器与 setSpeed，不改变任何控制量，
 * 唯一的动作是判定为致命故障时置 stop、切到 RACE_STOPPED。 */
static void faultScan(void) {
    int16 aL, aR, dif, sum;
    if (raceState != RACE_RUNNING && raceState != RACE_BRAKING) {
        encLostL = encLostR = stallCnt = graceCnt = 0;
        return;
    }
    if (graceCnt < 200) { // 发车后约 1 s 内不判：PID 起转、齿轮间隙都需要时间
        graceCnt++;
        return;
    }

    aL = func_abs(encoderL);
    aR = func_abs(encoderR);

    if (setSpeed > 60) { // 有明确速度指令才判编码器
        if (aL < 3) {
            if (encLostL < 250) encLostL++;
        } else
            encLostL = 0;
        if (aR < 3) {
            if (encLostR < 250) encLostR++;
        } else
            encLostR = 0;
        if (encLostL >= 60) faultFlags |= FAULT_ENC_L_DOWN; // 300 ms 无反馈
        if (encLostR >= 60) faultFlags |= FAULT_ENC_R_DOWN;
    } else {
        encLostL = encLostR = 0;
    }

    if (aL > 30 && aR > 30) { // 两轮都转起来后比较速差
        dif = (aL > aR) ? (int16) (aL - aR) : (int16) (aR - aL);
        sum = (int16) (aL + aR);
        if ((int16) (dif * 2) > sum)
            faultFlags |= FAULT_ENC_SKEW; // 差 > 50%
        else
            faultFlags &= (uint8) (~FAULT_ENC_SKEW);
    }

    if (setSpeed > 150 && aL < 8 && aR < 8) { // 有指令但两轮几乎不转
        if (stallCnt < 250) stallCnt++;
        if (stallCnt >= 100) faultFlags |= FAULT_STALL; // 500 ms
    } else
        stallCnt = 0;

    /* 致命故障：编码器掉线会让速度环正反馈飞车，必须立即停车 */
    if (faultFlags & (FAULT_ENC_L_DOWN | FAULT_ENC_R_DOWN | FAULT_STALL)) {
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
    // 冲出赛道保护：跑车过程中两侧边界全部丢失，或视野内 5 个采样点全暗，判定出界停车
    if (stop
#if (!LOW_SPEED_TEST)
        || ((raceState == RACE_RUNNING || raceState == RACE_BRAKING) &&
            ((!LRoadCount && !RRoadCount) ||
             (mt9v03x_frame[84][95] < threshold && mt9v03x_frame[88][47] < threshold &&
              mt9v03x_frame[87][141] < threshold && mt9v03x_frame[75][120] < threshold &&
              mt9v03x_frame[74][64] < threshold)))
#endif
       ) {
        // 出界保护必须与比赛状态同步，否则 raceTask 永远等不到里程且 TOP 键无法恢复
        if (!stop && (raceState == RACE_RUNNING || raceState == RACE_BRAKING))
            raceState = RACE_STOPPED;
        stop = 1;
        pwm_set_duty(PWME_CH3P_PA4, steerMid);
        pwm_set_duty(motorPwmR, motorPIDCalculate(&motorRPID, 0, encoderR));
        pwm_set_duty(motorPwmL, motorPIDCalculate(&motorLPID, 0, encoderL));
    } else {
        pwm_set_duty(PWME_CH3P_PA4, steerMid + steerPIDCalculate(&steerPid, duty));
        slow = !borderIn ? 1 : (currentSpeed < 350 ? 0 : slow);
#if (!LOW_SPEED_TEST)
        // 刹车段速度由 raceTask 锁定，环岛入弯减速不能把它抬回去
        if (!handle && raceState == RACE_RUNNING && borderIn && currentSpeed > 420 && slow) setSpeed = 200;
#endif
        RSetSpeed = setSpeed + (!R || !enableDifSpeed ? 0 : setSpeed * 8 / R);
        LSetSpeed = setSpeed - (!R || !enableDifSpeed ? 0 : setSpeed * 8 / R);
        pwm_set_duty(motorPwmR, motorPIDCalculate(&motorRPID, RSetSpeed, encoderR));
        pwm_set_duty(motorPwmL, motorPIDCalculate(&motorLPID, LSetSpeed, encoderL));
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
        setSpeed = (int16) func_limit_ab((int32) target, 0, SAFE_SPEED_CEILING);
#endif
    }
    R = d < 0 ? -r : r;
    duty = (int16) d * 12;
}

int8 getAimPos() {
    return c3;
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
            10000);
    PIDInit(&motorRPID, 1,
            3000, 50, 50,
            10000);
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
