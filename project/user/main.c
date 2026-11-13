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
#define TEST_AUTO_RUN   1

/* ===== 调试：开环固定占空比（不跑 PID） =====
 *   1 = 上电即以 TEST_OPEN_DUTY 让两轮慢速前进，仅用蓝灯指示编码器方向，
 *       用于把「电机/驱动/编码器链路」与「PID 参数」分开排查；0 = 正常比赛流程 */
#define TEST_OPEN_LOOP     0
#define TEST_OPEN_DUTY     1500    // 0~10000，1500 = 15% 占空比

int16 LCorner, RCorner, centerLineCount, RUN_MODE = FORWARD;
volatile int16 LRoadCount = expRoadCount, RRoadCount = expRoadCount; //中断里做出界判断，需要 volatile
static int16 imBtn, borderInTriggerDistance = 25;
volatile uint8 borderIn; //主循环写、中断读
uint8 switch1, switch2, ALONG = ALONG_R, topBtn, bottomBtn;
Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount];
extern int16 currentDuty;
int16 data i;

/* 8 组整定参数：{舵机增益 c1, 前瞻点序号 c3, 弯道限速系数 c4, 直道限速 c5}
 * 拨码 SW1/SW2 选择基址（0/2/4/6），TOP 取同组奇数档、BOTTOM 取偶数档 */
static const int16 speedSet[8][4] = {
    {260, 26, 4500, 520}, // 1
    {200, 22, 4000, 300}, // 2
    {210, 23, 4000, 320}, // 3
    {220, 23, 4000, 340}, // 4
    {230, 24, 4000, 360}, // 5
    {240, 24, 4000, 380}, // 6
    {250, 25, 4000, 450}, // 7
    {260, 25, 4000, 500}  // 8
};

static void applySpeedSet(uint8 idx) {
    motorInit(speedSet[idx][0], speedSet[idx][1], speedSet[idx][2], speedSet[idx][3], 0);
}

void loadRoad();

/* ===== 蓝灯指示（不含赛道检测） =====
 *   编码器/PID 故障  → 常亮
 *   编码器计数递增   → 快闪 5 Hz（向前推行）
 *   编码器计数倒扣   → 常亮（反向）
 *   无变化           → 熄灭 */
static void updateStatusLed(void) {
    static uint16 ledMs = 0;
    static uint8 ledPhase = 0;
    uint8 ledWant;

    if (faultFlags & (FAULT_ENC_L_DOWN | FAULT_ENC_R_DOWN | FAULT_PID_SAT | FAULT_PID_REV)) {
        ledWant = STATUS_LED_ON; // 故障：常亮
        ledPhase = 1;
        ledMs = 0;
    } else if (currentSpeed >= 3) {
        if (++ledMs >= 100) { // 前进：快闪
            ledMs = 0;
            ledPhase = (uint8) (!ledPhase);
        }
        ledWant = ledPhase ? STATUS_LED_ON : STATUS_LED_OFF;
    } else if (currentSpeed <= -3) {
        ledWant = STATUS_LED_ON; // 反向：常亮
        ledPhase = 1;
        ledMs = 0;
    } else {
        ledWant = STATUS_LED_OFF; // 静止：熄灭
        ledPhase = 0;
        ledMs = 0;
    }
    gpio_set_level(STATUS_LED_PIN, ledWant);
}

/* ===== 按键模块（B2/B3/B4，低电平有效，主循环 1 ms 调用一次） =====
 * 20 ms 消抖，提供「按下沿 / 松开沿」事件，B4 支持长按。
 *   B2 = 发车      B3 = 紧急停车      B4：短按 = 舵机自检，长按 ≥2 s = 标定一圈 */
#define BTN_B2          0x01
#define BTN_B3          0x02
#define BTN_B4          0x04
#define BTN_DEBOUNCE    20      // 连续 20 次（约 20 ms）采样一致才认
#define BTN_LONG_MS     2000    // B4 长按阈值（ms）

static uint8 btnDown = 0;        // 稳定状态位图（1 = 按下）
static uint8 btnDebCnt = 0;      // 消抖计数
static uint16 btnHoldMs = 0;     // B4 按住时长
static uint8 btnLongFired = 0;   // B4 本次按住已触发过长按
static uint8 btnLastWasLong = 0; // 上一次 B4 松开时是否为长按
static uint8 btnPressEdge = 0;   // 本次扫描刚按下的位
static uint8 btnReleaseEdge = 0;  // 本次扫描刚松开的位
static uint8 btnLongEvent = 0;   // B4 达到长按阈值（一次性）

static void btnScan(void) {
    uint8 raw = 0, changed;

    btnPressEdge = 0;
    btnReleaseEdge = 0;
    if (!gpio_get_level(IO_PB2)) raw |= BTN_B2;
    if (!gpio_get_level(IO_PB3)) raw |= BTN_B3;
    if (!gpio_get_level(IO_PB4)) raw |= BTN_B4;

    if (raw != btnDown) {
        if (++btnDebCnt >= BTN_DEBOUNCE) {
            btnDebCnt = 0;
            changed = (uint8) (raw ^ btnDown);
            btnPressEdge = (uint8) (changed & raw);        // 松 → 按
            btnReleaseEdge = (uint8) (changed & btnDown);  // 按 → 松
            btnDown = raw;
        }
    } else {
        btnDebCnt = 0;
    }

    if (btnDown & BTN_B4) { // 长按计时
        if (btnHoldMs < BTN_LONG_MS) btnHoldMs++;
        if (btnHoldMs >= BTN_LONG_MS && !btnLongFired) {
            btnLongFired = 1;
            btnLongEvent = 1;
        }
    } else {
        btnLastWasLong = (uint8) (btnHoldMs >= BTN_LONG_MS);
        btnHoldMs = 0;
        btnLongFired = 0;
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
    gpio_set_level(IO_P90, 0);
}

void main(void) {
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    { //摄像头初始化：最多重试约 3 秒（150 × 20ms）。原写法无限重试，现场无法判断是摄像头故障；
      //但预算太短又会在上电瞬间误判失败，所以留足 3 秒。
        uint16 camRetry = 0;
        while (mt9v03x_init() && ++camRetry < 150) system_delay_ms(20);
        if (camRetry >= 150) {
            /* 屏幕已舍弃：摄像头初始化失败时蓝灯常亮，程序停在这里 */
            gpio_init(STATUS_LED_PIN, GPO, STATUS_LED_ON, GPO_PUSH_PULL);
            while (1) system_delay_ms(100);
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
    gpio_init(IO_PB1, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_PB0, GPI, 0, GPI_PULL_UP);
    /* 三个独立按键：按车上实测接线标定（原工程为 P02/P01/P32，本车不适用）
     *   B2 = PB2 → TOP    （发车 / 选奇数档）
     *   B3 = PB3 → BOTTOM （标定一圈 / 选偶数档）
     *   B4 = PB4 → IMAGE  （急停）
     * PB0/PB1 是拨码开关、PB6/PB7 是摄像头 SCCB，均不冲突 */
    gpio_init(IO_PB2, GPI, 0, GPI_PULL_UP);  // TOP
    gpio_init(IO_PB3, GPI, 0, GPI_PULL_UP);  // BOTTOM
    gpio_init(IO_PB4, GPI, 0, GPI_PULL_UP);  // IMAGE
    gpio_init(IO_P90, GPO, 0, GPO_PUSH_PULL); //蜂鸣器（本车未装，保留不影响）
    gpio_init(STATUS_LED_PIN, GPO, STATUS_LED_OFF, GPO_PUSH_PULL); //蓝色状态灯
    /* 上电自检：蓝灯慢闪 3 次（每次亮 300ms / 灭 300ms）。
     * 「闪不闪」只说明这个引脚能不能控制灯，与高低电平极性无关；
     * 用来区分「引脚错/灯坏了」和「极性反了」两种情况。 */
    {
        uint8 n;
        for (n = 0; n < 3; n++) {
            gpio_set_level(STATUS_LED_PIN, STATUS_LED_ON);
            system_delay_ms(300);
            gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
            system_delay_ms(300);
        }
    }
    servoSelfTest(); //上电舵机自检：左右各摆约 15°，约 0.9 s（电机不参与）
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
            updateStatusLed();
        }
    }
#endif
    //gpio_init(IO_P04, GPO, 0, GPO_PUSH_PULL); //示波器
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
        static uint16 noFrameTicks = 0;  // 帧看门狗：2 ms 节拍累计，约 1 s 无新帧则安全停车
        static uint16 elemTicks = 0;       // 元素状态机停留帧数（超时兜底）
        /* 按键与里程判定不依赖相机帧：摄像头停帧时仍能急停 / 收车 */
        btnScan();
        /* B2：发车（完赛或急停后也可重新发车） */
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
        /* B4 短按（松开时判定，且本次不是长按）：舵机左右各摆 15° */
        if ((btnReleaseEdge & BTN_B4) && !btnLastWasLong) servoSelfTest();
        raceTask();

        updateStatusLed(); // 蓝灯：故障常亮 / 正向快闪 / 反向常亮 / 静止熄灭

        if (frameBusy) {
            noFrameTicks = 0;   // 收到完整帧，喂狗
            loadRoad();
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
                } else
                    alongLine(centerLine, centerLineCount, getAimPos());

                /* A3：元素状态机超时兜底——卡在十字/环岛超过 300 帧（约 3 s）就强制
                 * 回 FORWARD，否则会一直用固定 aim 打角、全速冲出赛道 */
                if (RUN_MODE == FORWARD)
                    elemTicks = 0;
                else if (++elemTicks > 300) {
                    elemTicks = 0;
                    elementReset();
                }
            } else if (raceState == RACE_IDLE) {
                raceShow();
            }
            frameBusy = 0;   // 消费完毕，允许 ISR 搬入下一帧
        } else if (++noFrameTicks > 1000) {
            /* 稳定优先：摄像头掉帧 / DMA 异常时，不允许保持上一拍打角继续全速跑 */
            noFrameTicks = 0;
            // 约 1 s 无新帧：紧急停车（不置故障位，避免与 PID 异常混淆）
            if (raceState == RACE_RUNNING || raceState == RACE_BRAKING) raceStopNow();
        }
        system_delay_ms(1);   // 主循环 1 ms 节拍：按键去抖 + 帧看门狗计时基准（越小越不压帧延迟）
    }
}

void loadRoad() {
    int16 idx;
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
    filterRoad(LRoad, LRoadCount);
    filterRoad(RRoad, RRoadCount);
    resampleRoad(LRoad, &LRoadCount, 0, 3);
    resampleRoad(RRoad, &RRoadCount, 0, 3);
    LCorner = findCorner(LRoad, LRoadCount, 0);
    RCorner = findCorner(RRoad, RRoadCount, 0);
    /* borderIn：左右各自独立判定，标志绝不跨分支复用；索引前先确认 count>0。
     * B4：只在普通循迹（FORWARD）时重算——环岛/十字阶段由元素函数维持 borderIn=1，
     * 若每帧都在这里覆盖，alongLine 的入弯减速（slow）就永远拿不到 1。 */
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
    if (ALONG == ALONG_L) {
        getCenterLineByLBorder(LRoad, centerLine, LRoadCount);
        centerLineCount = LRoadCount;
    } else {
        getCenterLineByRBorder(RRoad, centerLine, RRoadCount);
        centerLineCount = RRoadCount;
    }
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
