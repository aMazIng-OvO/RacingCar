/*********************************************************************************************************************
* STC32G144K Opensourec Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G144K 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-11-20        大W            first version
* 2026-4            lscyzq         init
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "BendEntry.h"   // 弯道入口过渡区识别（独立模块，ENABLE_BEND_ENTRY 控制）

/* 赛道元素开关 0=关闭 1=启用（先跑通纯循迹，再逐个打开调试） */
#define ENABLE_CROSS 1
#define ENABLE_ROUND 1

/* ---- 元素识别判据（按 IPS200 画面现场调整） ---- */
#define ROUND_ROW       37    //环岛判定所用重采样点序号（间距3px → 约111px前瞻）
#define ROUND_NEAR_X    20    //该点 x 小于此值：边界已收到车头附近
#define ROUND_FAR_MAX   11    //对侧边界点数量上限
#define CROSS_MAX       14    //十字判定：两侧边界点数上限

/* 调试：上电即自动进入 RUNNING，以开环固定占空比自动前进（不用按 B2）。
 * 0 = 正常流程（按 B2 发车）。上车前必须改回 0 */
#define TEST_AUTO_RUN   0

/* ===== 调试：开环固定占空比（不跑 PID） =====
 *   1 = 上电即以 TEST_OPEN_DUTY 让两轮慢速前进，仅用蓝灯指示编码器方向，
 *       用于把「电机/驱动/编码器链路」与「PID 参数」分开排查；0 = 正常比赛流程 */
#define TEST_OPEN_LOOP     0
#define TEST_OPEN_DUTY     1500    // 0~10000，1500 = 15% 占空比


int16 LCorner, RCorner, centerLineCount, RUN_MODE = FORWARD;
volatile int16 LRoadCount = expRoadCount, RRoadCount = expRoadCount; //中断里做出界判断，需要 volatile
static int16 borderInTriggerDistance = 25;
volatile uint8 borderIn; //主循环写、中断读
uint8 switch1, switch2, ALONG = ALONG_R;
Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount];
extern int16 currentDuty;

/* 8 组整定参数：{舵机增益 c1, 前瞻点序号 c3, 弯道限速系数 c4, 直道限速 c5}
 *   c1：每 1 度图像角度对应的舵机 duty = c1 / STEER_DUTY_DIV（见 Motor.h）
 *   c3：前瞻点序号，中线重采样间距 2 像素，c3 = 24 即约 48 像素前瞻
 *   c4：侧向加速度上限，弯道目标速度 v = sqrt(R · c4)
 *       v 的单位是「脉冲 / 5 ms」，R 的单位是去畸变后的像素；
 *       c4 = 2000 约对应 0.67 g（PULSE_PER_METER = 34600、1 像素 ≈ 1 cm 时）
 *   c5：直道目标速度上限，单位与 setSpeed 一致，最终再由 SPEED_LIMIT 封顶
 * 拨码 SW1/SW2 选择基址（0/2/4/6），每组的奇数档（更激进）被选为当前档位。 */
static const int16 speedSet[8][4] = {
    {260, 26, 2400, 400}, // 1
    {200, 22, 2000, 300}, // 2
    {210, 23, 2000, 320}, // 3
    {220, 23, 2000, 340}, // 4
    {230, 24, 2000, 360}, // 5
    {240, 24, 2000, 380}, // 6
    {250, 25, 2000, 400}, // 7
    {260, 25, 2000, 400}  // 8
};

static void applySpeedSet(uint8 idx) {
    motorInit(speedSet[idx][0], speedSet[idx][1], speedSet[idx][2], speedSet[idx][3], 0);
}

void loadRoad();

/* ===== 按键模块（B2/B3/B4，低电平有效） =====
 * 时间基准是控制中断维护的 msTick，不是循环次数：
 * 主循环处理一帧图像要 5~10 ms，用循环次数计时会把 20 ms 消抖拉成上百毫秒、
 * 把 2 s 长按拉成十几秒，现场表现就是"按键失效"。
 *   B2：短按 = 发车（需已识别到赛道）    长按 1 s = 强制发车
 *   B3：短按 = 紧急停车（任何状态都生效）
 *   B4：短按 = 舵机自检                  长按 2 s = 标定一圈 */
#define BTN_B2          0x01
#define BTN_B3          0x02
#define BTN_B4          0x04
#define BTN_DEBOUNCE_MS 20      // 连续 20 ms 采样一致才认
#define BTN_LONG_MS     2000    // B4 长按阈值

static uint8 btnDown = 0;        // 稳定状态位图（1 = 按下）
static uint16 btnChangeMs = 0;   // 最近一次采样出现差异的起始时刻
static uint16 btnDownMs = 0;     // 确认按下的时刻
static uint8 btnLongFired = 0;   // B4 本次按住已触发过长按
static uint8 btnPressEdge = 0;   // 本次扫描刚按下的位
static uint8 btnReleaseEdge = 0; // 本次扫描刚松开的位
static uint8 btnLongEvent = 0;   // B4 达到长按阈值（一次性）
static uint8 btnShortB4 = 0;     // B4 短按（松开时判定，一次性）

static void btnScan(void) {
    uint8 raw = 0, changed;
    uint16 now = msTick, held;

    btnPressEdge = 0;
    btnReleaseEdge = 0;
    if (!gpio_get_level(IO_PB2)) raw |= BTN_B2;
    if (!gpio_get_level(IO_PB3)) raw |= BTN_B3;
    if (!gpio_get_level(IO_PB4)) raw |= BTN_B4;

    if (raw != btnDown) {
        /* 采样与稳定值不同：从第一次出现差异起计时，满 20 ms 才确认。
         * 中途抖回稳定值会走下面的 else 分支把计时清零，所以不会误触发。 */
        if ((uint16) (now - btnChangeMs) >= BTN_DEBOUNCE_MS) {
            changed = (uint8) (raw ^ btnDown);
            btnPressEdge = (uint8) (changed & raw);        // 松 → 按
            btnReleaseEdge = (uint8) (changed & btnDown);  // 按 → 松
            if (btnReleaseEdge & BTN_B4) btnShortB4 = (uint8) !btnLongFired;
            btnChangeMs = now;
            btnDownMs = now;
            btnDown = raw;
            btnLongFired = 0;
        }
    } else {
        btnChangeMs = now;
        held = (uint16) (now - btnDownMs);
        if ((btnDown & BTN_B4) && held >= BTN_LONG_MS && !btnLongFired) {
            btnLongFired = 1;
            btnLongEvent = 1;
        }
    }
}

/* 发车/急停时统一复位：元素状态机、补线偏移、控制量与比赛相关全局量。
 * 只在 main.c 实现，由 Stop.c 的 raceStart()/stopInit() 调用。 */
void elementReset(void) {
    crossReset();
    lroundReset();
    rroundReset();
    controlReset();
    RUN_MODE = FORWARD;
    ALONG = ALONG_R;
    borderIn = 0;
    centerLineDistance = 18;
    banUpdateThreshold = 0;
    centerLineCount = 0;
    LCorner = RCorner = 0;
    LRoadCount = RRoadCount = expRoadCount;
    outOfTrack = 0;
    gpio_set_level(IO_P90, 0);
}

/* ===== USB CDC 调试输出 =====
 * 屏幕已从硬件上移除，里程标定、阈值、故障码全部靠 USB CDC 读取（debug_init 已打开）。
 * 串口没有连接时 usb_cdc_write_buffer 会立即返回，不占运行时间。
 *   待发车：每 250 ms 打印全部读数，推车 1 米看 ODO 的增量即可标定 PULSE_PER_METER
 *   跑车中：每 1 s 打印一行，确认目标速度、实际速度、占空比与故障码 */
static void debugReport(uint8 running) {
    static uint16 lastMs = 0;
    uint16 now = msTick;
    int32 odo;

    if ((uint16) (now - lastMs) < (uint16) (running ? 1000 : 250)) return;
    lastMs = now;
    interrupt_global_disable();
    odo = odometer;
    interrupt_global_enable();
    printf("ODO=%ld L=%d R=%d C=%d THR=%d MODE=%d AL=%d STA=%d ERR=%d SPD=%d DS=%d DUTY=%d\n",
           odo, LRoadCount, RRoadCount, centerLineCount, threshold,
           RUN_MODE, ALONG, trackIsOn(), faultFlags, currentSpeed, setSpeed, currentDuty);
}

/* 出界采样的 5 个点：车头前方与两侧。返回低于阈值的个数。
 * 原来把「两侧边界全丢」单独作为出界条件，弯道里边界跑出窄列搜索范围、
 * 两侧起点同时失败时会被当成冲出赛道，发车十几毫秒后就停车。 */
static uint8 darkPoints(void) {
    uint8 n = 0;

    if (mt9v03x_frame[84][95] < threshold) n++;
    if (mt9v03x_frame[88][47] < threshold) n++;
    if (mt9v03x_frame[87][141] < threshold) n++;
    if (mt9v03x_frame[75][120] < threshold) n++;
    if (mt9v03x_frame[74][64] < threshold) n++;
    return n;
}

void main(void) {
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    /* ===== 最早期初始化：状态灯与按键最先就绪 =====
     * 之前按键在摄像头之后才初始化，一旦摄像头初始化失败，按键会全部失效。 */
    gpio_init(STATUS_LED_PIN, GPO, STATUS_LED_OFF, GPO_PUSH_PULL); // 蓝色状态灯
    { /* 上电自检：蓝灯闪 3 下（亮/灭各 300 ms） */
        uint8 n;
        for (n = 0; n < 3; n++) {
            gpio_set_level(STATUS_LED_PIN, STATUS_LED_ON);
            system_delay_ms(300);
            gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
            system_delay_ms(300);
        }
    }
    { //摄像头初始化：最多重试约 3 秒（150 × 20ms）。原写法无限重试，现场无法判断是摄像头故障；
      //但预算太短又会在上电瞬间误判失败，所以留足 3 秒。
        uint16 camRetry = 0;
        while (mt9v03x_init() && ++camRetry < 150) system_delay_ms(20);
        if (camRetry >= 150) {
            /* 摄像头初始化失败：蓝灯慢闪（500/500），程序停在这里 */
            while (1) {
                gpio_set_level(STATUS_LED_PIN, STATUS_LED_ON);
                system_delay_ms(500);
                gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
                system_delay_ms(500);
            }
        }
    }
    /* 稳定优先：纠正中断优先级倒挂。
     * 库在 mt9v03x_init() 内把 INT1(VSYNC) 与 LCM_DMA 都设为最高 3，
     * 而 TIM1 的 5 ms 电机 PID 中断未设置优先级（默认最低 0），无法抢占 DMA。
     * 这里把 TIM1 提到 3、把整帧搬运所在的 LCM_DMA 降到 1：
     * 控制中断可抢占搬运，同时保留 ISR 内搬运带来的帧原子性。
     * 注：IP/IPH 分支只置位（0->3 可用）；LCM_DMA 走 DMA_LCM_CFG 可双向设置。 */
    interrupt_set_priority(TIMER1_IRQn, 3);
    interrupt_set_priority(LCM_DMA_IRQn, 1);
    /* 按键与拨码必须在摄像头初始化之后再配置：
     * 实测放到摄像头之前会被 mt9v03x_init() 的端口配置覆盖，导致所有按键失效。 */
    gpio_init(IO_PB1, GPI, 0, GPI_PULL_UP);   // 拨码 SW2
    gpio_init(IO_PB0, GPI, 0, GPI_PULL_UP);   // 拨码 SW1
    gpio_init(IO_PB2, GPI, 0, GPI_PULL_UP);   // B2 发车
    gpio_init(IO_PB3, GPI, 0, GPI_PULL_UP);   // B3 急停
    gpio_init(IO_PB4, GPI, 0, GPI_PULL_UP);   // B4 自检 / 标定
    gpio_init(IO_P90, GPO, 0, GPO_PUSH_PULL); // 蜂鸣器（本车未装）
    gpio_init(STATUS_LED_PIN, GPO, STATUS_LED_OFF, GPO_PUSH_PULL); // 摄像头初始化可能改动端口，重新配置一次
    servoSelfTest(); //上电舵机自检：左右各摆约 30°，约 1.4 s，结束回机械 90°（电机不参与）
    bendEntryInit(); // 弯道入口状态机复位
#if (TEST_OPEN_LOOP)
    { /* ===== 调试：开环固定占空比（完全不用 PID） =====
       * 上电即让两轮以 TEST_OPEN_DUTY 慢速正转，只跑蓝灯指示：
       *   蓝灯快闪 = 编码器计数递增（方向对）；常亮 = 倒扣；熄灭 = 没计数
       * 用来把「电机/驱动/编码器接线」和「PID 参数」彻底分开。 */
        int16 data rawR, data rawL;
        pwm_init(PWMB_CH3_P76, 17000, 0);          // 右轮 PWM
        pwm_init(PWMB_CH1_P74, 17000, 0);          // 左轮 PWM
        gpio_init(IO_P77, GPO, 1, GPO_PUSH_PULL);  // 右轮 DIR：高=正转
        gpio_init(IO_P75, GPO, 1, GPO_PUSH_PULL);  // 左轮 DIR：高=正转
        encoder_quad_init(PWMA_ENCODER, PWMA_ENCODER_CH1P_P60, PWMA_ENCODER_CH2P_P62);
        encoder_quad_init(PWMC_ENCODER, PWMC_ENCODER_CH1P_P40, PWMC_ENCODER_CH2P_P42);
        system_delay_ms(200);
        pwm_set_duty(PWMB_CH3_P76, TEST_OPEN_DUTY);
        pwm_set_duty(PWMB_CH1_P74, TEST_OPEN_DUTY);
        while (1) {
            system_delay_ms(20);
            rawR = encoder_get_count(PWMA_ENCODER);
            rawL = encoder_get_count(PWMC_ENCODER);
            encoder_clear_count(PWMA_ENCODER);
            encoder_clear_count(PWMC_ENCODER);
            currentSpeed = (int16) ((-(int32) rawR * 2 + (int32) rawL * 2) / 2);
            trackMonitor();
        }
    }
#endif
    //gpio_init(IO_P04, GPO, 0, GPO_PUSH_PULL); //示波器
    frameBusy = 0;   // 丢弃舵机自检期间累积的旧帧，阈值必须按当前画面估计
    { //等 DMA 中断搬入第一帧完整快照再估阈值（带超时，防止摄像头异常时死等）
        uint16 wait = 0;
        while (!frameBusy && ++wait < 20000);
    }
    LThreshold = RThreshold = threshold = imageInitThreshold();
    frameBusy = 0;   // 消费开机首帧，让后续帧正常搬运
    /* ===== 上电自动选档：拨码 SW1/SW2 选 4 组，取该组中较保守的一档 =====
     * 按键已简化：B2 发车 / B3 紧急停车 / B4 舵机自检（长按 2 s = 标定一圈），
     * 因此不再需要按键选档，上电即按拨码定档并初始化电机与编码器。 */
    switch2 = gpio_get_level(IO_PB1);
    switch1 = gpio_get_level(IO_PB0);
    applySpeedSet((uint8) (((switch2 ? 2 : 0) + (switch1 ? 1 : 0)) * 2 + 1));

#if (TEST_AUTO_RUN)
    /* 调试：上电直接进入 RUNNING，开环固定占空比自动前进（不用按 B2）。
     * 按 B3 仍可立即停车。上车前把 TEST_AUTO_RUN 改回 0 */
    stop = 0;
    raceState = RACE_RUNNING;
#endif
    RUN_MODE = FORWARD;
    stopInit();
    frameBusy = 0;   // 丢弃选档期间累积的旧帧，第一拍处理发车后的当前画面
    while (1) {
        static uint16 lastFrameMs = 0;   // 最近一次收到完整帧的时刻
        static uint16 elemEnterMs = 0;   // 进入元素的时刻
        static uint16 runStartMs = 0;    // 最近一次进入 RACE_RUNNING 的时刻
        static uint8 lastRaceState = 0;  // 上一次的比赛状态，用于捕捉状态切换
        /* 记下发车时刻：出界判定在发车后 800 ms 内不生效 */
        if (raceState != lastRaceState) {
            if (raceState == RACE_RUNNING) runStartMs = msTick;
            lastRaceState = (uint8) raceState;
        }
        /* 按键与里程判定不依赖相机帧：摄像头停帧时仍能急停 / 收车 */
        btnScan();
        /* 按键确认反馈：任意一个键按下（边沿）→ 蓝灯亮 60 ms。
         * 非阻塞：只设一个保持时间，由 trackMonitor 让出 LED 驱动权。
         * 原来的 system_delay_ms(60) 会在发车瞬间白丢约 10 帧图像。 */
        if (btnPressEdge) trackLedHold(60);
        /* B2：发车（完赛或急停后也可重新发车）。
         * 与参考实现一致，发车不设赛道门槛：赛道状态只做指示，不阻断发车。 */
        if (btnPressEdge & BTN_B2) {
            if (raceState == RACE_IDLE || raceState == RACE_STOPPED) {
                faultReset(); // 每次发车清空上一轮故障位（停车时保留，便于赛后查看原因）
                raceStart();
            }
        }
        /* B3：紧急停车（不再限状态，任何时刻按下都停，避免"跑了停不下来"） */
        if (btnPressEdge & BTN_B3) {
            raceStopNow();
            motorForceStop(); // 双保险：直接清零两路占空比并取消助推
        }
        /* B4 长按 ≥2 s：标定一圈（按住到阈值时触发一次） */
        if (btnLongEvent) {
            btnLongEvent = 0;
            if (raceState == RACE_IDLE) raceSetLap();
        }
        /* B4 短按（松开时判定，且本次不是长按）：舵机左右各摆 15°。
         * 只在停车状态执行：自检要占用 0.9 秒，行驶中做这件事等于闭眼开车。 */
        if (btnShortB4) {
            btnShortB4 = 0;
            if (raceState == RACE_IDLE || raceState == RACE_STOPPED) servoSelfTest();
        }
        raceTask();

        trackMonitor(); // 蓝灯：故障常亮 / 正向快闪 / 反向常亮 / 静止熄灭

        if (frameBusy) {
            lastFrameMs = msTick;   // 收到完整帧，喂狗
            loadRoad();
            /* 出界判定放在这里：此刻快照不会被 DMA 搬运覆盖，读像素是安全的；
             * 结论交给控制中断做 15 ms 消抖，控制中断不再直接访问整帧图像。
             * 判据与参考实现一致：两侧边界全丢，或 5 个采样点全暗。 */
            trackEvaluate();        // 用同一份边界点数更新赛道状态，供发车许可与蓝灯使用
#if (ENABLE_BEND_ENTRY)
            /* 入弯过渡区识别：在直道与弯道的连接处，画面里没有边界是正常现象。
             * 模块自己维护状态机，只向这里提供出界宽容等级。 */
            bendEntryUpdate(centerLine, centerLineCount, getAimPos());
#endif
            {
                uint8 dark = darkPoints();
                uint8 guard = 0;
#if (ENABLE_BEND_ENTRY)
                guard = bendEntryGuard();
#endif
                if (guard >= 2) {
                    /* 连接处时间窗内：完全免判，画面里的黑区是弯道外侧的底布 */
                    outOfTrack = 0;
                } else if (guard == 1) {
                    /* 已确认入弯：单侧丢边属于常态，只有 5 点全暗才算冲出 */
                    outOfTrack = (uint8) (dark >= 5);
                } else {
                    /* 5 点全暗：画面里确实没有赛道，直接出界。
                     * 两侧边界全丢时要求至少 3 点暗：弯道里边界跑出搜索范围很常见，
                     * 只要画面还有亮区就说明车没冲出去，不能停。 */
                    outOfTrack = (uint8) ((dark >= 5) ||
                                          ((!LRoadCount && !RRoadCount) && dark >= 3));
                }
                /* 发车后的宽限期：起步瞬间视觉还没稳定，先不判出界 */
                if ((uint16) (msTick - runStartMs) < 800) outOfTrack = 0;
            }
            if (raceState == RACE_RUNNING || raceState == RACE_BRAKING) {
                if (ENABLE_CROSS && (RUN_MODE == CROSS ||
                                     (RUN_MODE == FORWARD && ((RCorner && RRoadCount > 2) || (LCorner && LRoadCount > 2)) &&
                                      LRoadCount < CROSS_MAX && RRoadCount < CROSS_MAX))) {
                    RUN_MODE = CROSS;
                    runCross();
                } else if (ENABLE_ROUND && (RUN_MODE == R_ROUND ||
                                            (RUN_MODE == FORWARD && LRoadCount > ROUND_ROW && LRoad[ROUND_ROW].x < ROUND_NEAR_X &&
                                             RCorner && RRoadCount < ROUND_FAR_MAX))) {
                    RUN_MODE = R_ROUND;
                    borderIn = 1;
                    runRRound(LCorner, RCorner);
                } else if (ENABLE_ROUND && (RUN_MODE == L_ROUND ||
                                            (RUN_MODE == FORWARD && RRoadCount > ROUND_ROW && RRoad[ROUND_ROW].x < ROUND_NEAR_X &&
                                             LCorner && LRoadCount < ROUND_FAR_MAX))) {
                    RUN_MODE = L_ROUND;
                    borderIn = 1;
                    runLRound(LCorner, RCorner);
                } else {
                    int8 beTurn = 0;
                    /* 连接处中线点数不足时用入弯前冻结的转角维持转向，
                     * 视觉一丢失就把方向打直，车会切进内侧或冲出去 */
                    if (ENABLE_BEND_ENTRY && bendEntryTurn(&beTurn))
                        setServoDuty(beTurn);
                    else
                        alongLine(centerLine, centerLineCount, getAimPos());
                }

                /* 元素超时兜底：卡在十字/环岛超过 8 s 就强制回 FORWARD，
                 * 否则会一直用固定 aim 打角、全速冲出赛道。
                 * 计时按真实毫秒而不是帧数，帧率波动时阈值不会跟着漂。 */
                if (RUN_MODE == FORWARD)
                    elemEnterMs = msTick;
                else if ((uint16) (msTick - elemEnterMs) > 8000) {
                    elemEnterMs = msTick;
                    elementReset();
                }
            } else {
                elemEnterMs = msTick;
            }
            frameBusy = 0;   // 消费完毕，允许 ISR 搬入下一帧
        } else if ((uint16) (msTick - lastFrameMs) > 1000) {
            /* 稳定优先：摄像头掉帧 / DMA 异常时，不允许保持上一拍打角继续全速跑 */
            lastFrameMs = msTick;
            // 约 1 s 无新帧：紧急停车（不置故障位，避免与 PID 异常混淆）
            if (raceState == RACE_RUNNING || raceState == RACE_BRAKING) raceStopNow();
        }
        debugReport(raceState == RACE_RUNNING || raceState == RACE_BRAKING);
        system_delay_ms(1);   // 主循环 1 ms 节拍只用于让出 CPU，所有计时都以 msTick 为准
    }
}

/* ===== 弯道识别与中线融合 =====
 * 拐点序号要交给元素状态机，参考实现直接把 LRoadCount / RRoadCount 裁到拐点处，
 * 弯道里拐点往往落在第 5~10 个点，中线只剩这几个点，前瞻点被夹到车头附近，
 * 纯跟踪拿不到远处的弯曲趋势，表现就是"看到弯了也来不及转"。
 * 这里保留一份未裁短的点列专门生成中线，全局点数仍按参考实现裁短供元素判据使用。 */
#define MERGE_WIDTH_MIN   12    // 双侧配对的最小赛道宽度（像素）
#define MERGE_WIDTH_MAX   130   // 双侧配对的最大赛道宽度
#define MERGE_ROW_TOL     15    // 左右边界配对允许的行号差

static Point midL[expRoadCount], midR[expRoadCount];

/* 以点数较多的一侧为骨架生成中线。两侧同一行都有点时取列号中点，
 * 只有一侧时按 centerLineDistance 外推。输出点数等于骨架点数，
 * 弯道外侧边界被 turn>=4 打断时中线长度不会跟着骤降。 */
static void buildCenterLine(Point ml[], int16 mlc, Point mr[], int16 mrc) {
    Point *skel, *other;
    int16 sc, oc, i, j = 0, k = 0, t, best, bestD, d, width, yl, yr, dy, halfW;
    static int16 lastHalfW = 0;   // 跨帧记忆的实测半宽，只有单侧可见时继续用它外推
    uint8 leftIsSkel = (uint8) (mlc >= mrc);

    if (mlc <= 0 && mrc <= 0) {
        centerLineCount = 0;
        return;
    }
    if (leftIsSkel) {
        skel = ml; sc = mlc; other = mr; oc = mrc;
    } else {
        skel = mr; sc = mrc; other = ml; oc = mlc;
    }
    halfW = lastHalfW ? lastHalfW : (int16) centerLineDistance;
    for (i = 0; i < sc && k < expRoadCount; i++) {
        best = -1;
        bestD = 0x7FFF;
        /* 两条边界都按近→远排列（行号递减），游标只向前推进 */
        while (j + 1 < oc && other[j].x > skel[i].x) j++;
        /* 游标附近取行号最接近的点，容忍重采样带来的 1~2 像素错位 */
        for (t = j - 2; t <= j + 2; t++) {
            if (t < 0 || t >= oc) continue;
            d = func_abs((int16) other[t].x - (int16) skel[i].x);
            if (d < bestD) {
                bestD = d;
                best = t;
            }
        }
        centerLine[k].x = skel[i].x;
        if (best >= 0 && oc > 0 && bestD <= MERGE_ROW_TOL) {
            yl = leftIsSkel ? (int16) skel[i].y : (int16) other[best].y;
            yr = leftIsSkel ? (int16) other[best].y : (int16) skel[i].y;
            width = yr - yl;
            if (width >= MERGE_WIDTH_MIN && width <= MERGE_WIDTH_MAX) {
                /* 记下本帧实测半宽，后面只有单侧时用它外推。
                 * 固定用 centerLineDistance(18) 时，赛道实际半宽 30 会让断裂段的
                 * 中线偏向骨架侧十几个像素，弯道里表现为转向偏晚。 */
                halfW = (int16) func_limit_ab((width + 1) >> 1, 8, 60);
                /* 1/4 权重平滑，避免单帧噪声把半宽带偏 */
                lastHalfW = lastHalfW ?
                            (int16) (((int32) lastHalfW * 3 + halfW + 2) >> 2) : halfW;
                centerLine[k].y = (uint8) ((yl + yr) >> 1);
                k++;
                continue;
            }
        }
        /* 单侧有效：按本帧实测半宽外推，缺侧不会让整条中线消失 */
        dy = (int16) skel[i].y + (leftIsSkel ? halfW : -halfW);
        centerLine[k].y = (uint8) func_limit_ab(dy, 0, MT9V03X_W - 1);
        k++;
    }
    centerLineCount = k;
}

void loadRoad() {
    int16 idx, midLc, midRc;
    uint8 leftFound = 0, rightFound = 0;
    Point LStart = getLStart(), RStart = getRStart();
    if (RUN_MODE == FORWARD) centerLineDistance = 18; //普通循迹恢复默认补线偏移
    LRoadCount = expRoadCount, RRoadCount = expRoadCount;
    getLBorder(LRoad, LStart, &LRoadCount);
    getRBorder(RRoad, RStart, &RRoadCount);
    LBorderCount = func_limit_ab(LRoadCount, 0, expRoadCount);
    memcpy(LBorder, LRoad, LBorderCount * sizeof(Point));
    RBorderCount = func_limit_ab(RRoadCount, 0, expRoadCount);
    memcpy(RBorder, RRoad, RBorderCount * sizeof(Point));
    getGodRoadLine(LRoad, LRoadCount);
    getGodRoadLine(RRoad, RRoadCount);
    /* 用两侧边界的实测宽度修正单侧外推的假设半宽。
     * 参考实现把 centerLineDistance 固定成 18 像素，实际赛道更宽时，
     * 只有一侧可见的那几行会按 18 外推，中线整体偏向可见侧，
     * 纯跟踪随之给出一个持续的同向转角，表现就是轨迹贴着一侧边线跑。
     * 两侧都有效、行号能对上的点取宽度平均，作为本帧的半宽。 */
    if (RUN_MODE == FORWARD && LRoadCount > 10 && RRoadCount > 10) {
        int16 i2, j2 = 0, n2 = 0, w, hw;
        int32 sumW = 0;
        for (i2 = 0; i2 < LRoadCount && n2 < 24; i2 += 2) {
            if (j2 + 1 < RRoadCount) {
                while (j2 + 1 < RRoadCount && RRoad[j2].x > LRoad[i2].x) j2++;
            }
            if (func_abs((int16) RRoad[j2].x - (int16) LRoad[i2].x) > 3) continue;
            w = (int16) RRoad[j2].y - (int16) LRoad[i2].y;
            if (w < MERGE_WIDTH_MIN || w > MERGE_WIDTH_MAX) continue;
            sumW += w;
            n2++;
        }
        if (n2 >= 5) {
            hw = (int16) (sumW / n2 / 2);
            centerLineDistance = (uint8) func_limit_ab(hw, 8, 60);
        }
    }
    filterRoad(LRoad, LRoadCount);
    filterRoad(RRoad, RRoadCount);
    resampleRoad(LRoad, &LRoadCount, 0, 3);
    resampleRoad(RRoad, &RRoadCount, 0, 3);
    LCorner = findCorner(LRoad, LRoadCount, 0);
    RCorner = findCorner(RRoad, RRoadCount, 0);
    /* 下面的拐点分支会把全局点数裁到拐点处供元素判据使用。
     * 中线必须用裁短之前的完整点列，否则弯道里前瞻点被夹到车头附近。 */
    midLc = LRoadCount;
    midRc = RRoadCount;
    if (midLc > 0) memcpy(midL, LRoad, midLc * sizeof(Point));
    if (midRc > 0) memcpy(midR, RRoad, midRc * sizeof(Point));
    /* borderIn：左右各自独立判定，标志绝不跨分支复用；索引前先确认 count>0。
     * 只在普通循迹（FORWARD）时重算——环岛/十字阶段由元素函数维持 borderIn=1，
     * 若每帧都在这里覆盖，setServoDuty 的入弯减速就永远拿不到 1。 */
    if (RUN_MODE == FORWARD) {
        for (idx = 0; idx < LRoadCount; idx += 2)
            if (LRoad[idx].y > 94) { leftFound = 1; break; }
        if (leftFound && LRoadCount > 0 &&
            LRoad[func_limit_ab(idx, 0, LRoadCount - 1)].x > borderInTriggerDistance)
            borderIn = 1;
        else {
            for (idx = 0; idx < RRoadCount; idx += 2)
                if (RRoad[idx].y < 94) { rightFound = 1; break; }
            borderIn = (uint8) (rightFound && RRoadCount > 0 &&
                                RRoad[func_limit_ab(idx, 0, RRoadCount - 1)].x > borderInTriggerDistance);
        }
    }
    if (LCorner) {
        if (LCorner < 3) LCorner = 0;
        else if (LCorner < 15 && RUN_MODE == FORWARD && LRoad[func_limit_ab(LCorner + 3, 0, LRoadCount - 1)].y > LRoad[LCorner].y) {
            ALONG = ALONG_R;
            centerLineDistance = 9;
        }
        LRoadCount = LCorner;
    }
    if (RCorner) {
        if (RCorner < 3) RCorner = 0;
        else if (RCorner < 15 && RUN_MODE == FORWARD && RRoad[RCorner].y > RRoad[func_limit_ab(RCorner + 3, 0, RRoadCount - 1)].y) {
            ALONG = ALONG_L;
            centerLineDistance = 9;
        }
        RRoadCount = RCorner;
    }
    /* 中线由完整的左右边界融合得到，不再只靠 ALONG 选中的单侧平移。
     * 单侧平移假设赛道半宽恒为 centerLineDistance，弯道里整条中线会偏向一侧；
     * 双侧取中点让宽度误差自动对消，只有一侧有效时才退回外推。 */
    buildCenterLine(midL, midLc, midR, midRc);
    if (centerLineCount >= 2) {
        int16 startPos = 0;
        for (idx = 1; idx < centerLineCount; idx++)
            if (centerLine[idx].x < MT9V03X_H - 1) { startPos = idx - 1; break; }
        centerLine[startPos].x = MT9V03X_H - 1;
        centerLine[startPos].y = centerLine[func_limit_ab(startPos + 1, 0, centerLineCount - 1)].y;
        filterRoad(centerLine, centerLineCount);
        resampleRoad(centerLine, &centerLineCount, startPos, 2);
    } else {
        centerLineCount = 0;   // 边界全丢时不写脏点，控制量沿用到下一帧判定
    }
    /* 普通循迹由两侧点数投票决定单侧外推的备选边；元素阶段由状态机维持 ALONG，
     * 每帧无条件重算会把环岛五个阶段设置的值覆盖掉。 */
    if (RUN_MODE == FORWARD)
        ALONG = LRoadCount > RRoadCount ? ALONG_L : ALONG_R;
}

/*　　┏┓　　┏┓
   　　┏┛┻━━━┛┻┓
   　　┃　　　　　　　 ┃ 　
   　　┃　　　━　　　 ┃
   　　┃　＞　　　＜┃
   　　┃　　　　　　　 ┃
   　　┃ . ⌒　..┃
   　　┃　　　　　　　 ┃
   　　┗━┓　　　┏━┛
     　　　　┃　　　┃　Guardian beasts, protect me; bad luck, stay away!　　　　　
     　　　　┃　　　┃  神兽护体，水逆退散！
     　　　　┃　　　┃　　　　　　　　　　　
     　　　　┃　　　┃ 　　　　　　
     　　　　┃　　　┃
     　　　　┃　　　┃　　　　　　　　　　　
     　　　　┃　　　┗━━━┓
     　　　　┃　　　　　　　┣┓
     　　　　┃　　　　　　　┏┛
     　　　　┗┓┓┏━┳┓┏┛
      　　　　　┃┫┫　┃┫┫
      　　　　　┗┻┛　┗┻┛
 */
