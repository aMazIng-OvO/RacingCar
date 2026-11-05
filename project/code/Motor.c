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
extern uint8 switch1, borderIn, ALONG;
static uint8 DEVICE_H = 18;
int8 slow, stop, smoothStop;
int16 R, c1, c2, c3, c4, c5, c6, LSetSpeed, RSetSpeed, enableDifSpeed = 1, ct2 = 0;
int16 currentDuty, longc = 300, LPWM, currentSpeed;
extern volatile int32 odometer;
extern int16 LRoadCount, RRoadCount;

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
    int32 err, delta_err, delta_u;
    err = target - actual;
    delta_err = err - pid->last_error;
    delta_u = (int32) pid->Kp * delta_err +
                 (int32) pid->Ki * err +
                 (int32) pid->Kd * (delta_err - (pid->last_error - pid->llast_error));
    pid->llast_error = pid->last_error;
    pid->last_error = err;
    pid->base += delta_u / 100;
    pid->base = func_limit(pid->base, pid->max_out);
    gpio_set_level((gpio_pin_enum) pid->which ? motorDirR : motorDirL, pid->base >= 0);
    return func_abs(pid->base);
}

static void pitHandler() {
    encoderR = -encoder_get_count(PWMA_ENCODER) << 1;
    encoderL = encoder_get_count(PWMC_ENCODER) << 1;
    encoder_clear_count(PWMA_ENCODER);
    encoder_clear_count(PWMC_ENCODER);
    odometer += (int32) ((encoderL + encoderR) >> 1); //里程累计（供停车判定使用）
    // 冲出赛道保护：跑车过程中两侧边界全部丢失，或视野内 5 个采样点全暗，判定出界停车
    if (stop || ((raceState == RACE_RUNNING || raceState == RACE_BRAKING) &&
                 ((!LRoadCount && !RRoadCount) ||
                  (mt9v03x_frame[84][95] < threshold && mt9v03x_frame[88][47] < threshold &&
                   mt9v03x_frame[87][141] < threshold && mt9v03x_frame[75][120] < threshold &&
                   mt9v03x_frame[74][64] < threshold)))) {
        stop = 1;
        pwm_set_duty(PWME_CH3P_PA4, steerMid);
        pwm_set_duty(motorPwmR, motorPIDCalculate(&motorRPID, 0, encoderR));
        pwm_set_duty(motorPwmL, motorPIDCalculate(&motorLPID, 0, encoderL));
    } else {
        pwm_set_duty(PWME_CH3P_PA4, steerMid + steerPIDCalculate(&steerPid, duty));
        currentSpeed = encoderR + encoderL >> 1;
        slow = !borderIn ? 1 : currentSpeed < 350 ? 0 : slow;
        if (borderIn && currentSpeed > 420 && slow) setSpeed = 200;
        RSetSpeed = setSpeed + (!R || !enableDifSpeed ? 0 : setSpeed * 8 / R);
        LSetSpeed = setSpeed - (!R || !enableDifSpeed ? 0 : setSpeed * 8 / R);
        // if (longc > 150) RSetSpeed = LSetSpeed = c4;
        // else RSetSpeed = LSetSpeed = c5;
        pwm_set_duty(motorPwmR, motorPIDCalculate(&motorRPID, RSetSpeed, encoderR));
        pwm_set_duty(motorPwmL, motorPIDCalculate(&motorLPID, LSetSpeed, encoderL));
    }
}

void alongLine(Point roadLine[], int16 count, int16 aimPos) {
    int16 pos, dx, dy;
    int8 d;
    if (count <= 5) return;
    pos = func_limit_ab(aimPos, 5, count - 2);
    dx = MT9V03X_H - roadLine[pos].x + DEVICE_H;
    dy = 94 - roadLine[pos].y;
    d = atan2_int(2 * DEVICE_H * dy, dx * dx + dy * dy);
#if (DEBUG_MODE)
    if (switch1)
        ips200_draw_point(roadLine[pos].y, roadLine[pos].x + MT9V03X_H + 40, RGB565_RED);
#endif
     setServoDuty(func_limit(d, 32));
}

void setServoDuty(int8 d) {
    int16 data t, r, ts;
    currentDuty = (int16) d * c1 / 100;
    t = tani[func_limit_ab(func_abs(currentDuty), 0, 32)];
#if (DEBUG_MODE)
    if (switch1) ips200_show_int16(MT9V03X_W / 2, MT9V03X_H + 20, currentDuty);
#endif
    r = !t ? 0 : 20000 / t;
    // if (borderIn) ts = func_min(c5, 400);
    // else ts = c5;
    ts = c5;
    if (!handle)
        setSpeed = !r ? ts : func_min(sqrt_int(r * (uint32) c4), ts);
    R = d < 0 ? -r : r;
    duty = (int16) d * 12;
}

int8 getAimPos() {
    return c3;
}

void motorInit(int16 c11, int16 c22, int16 c33, int16 c44, int16 c55, int16 c66, int16 c77) {
    c1 = c11;
    c2 = c22;
    c3 = c33;
    c4 = c44;
    c5 = c55;
    c6 = c66;
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
