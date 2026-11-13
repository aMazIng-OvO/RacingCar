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

/* ===== 视觉单测开关（当前为比赛版，已关闭） =====
 * 1 = 纯视觉测试：只跑「采集 → 双缓冲快照 → 爬线 → 去畸变 → 中线 → 拐点」这条链路，
 *     不调用 motorInit()，不启动 5ms 控制 PIT、不初始化 PWM/编码器、不驱动电机与舵机；
 *     仅需「主板 + 摄像头 + 屏幕」即可标定视觉参数。
 * 0 = 正常竞速流程（选档 → 发车 → 循迹/元素/停车）。 */
#define VISION_ONLY     0

/* 画面刷新策略（实测刷一整屏图像约 190 ms，是全机最贵的操作）：
 *   VISION_PREVIEW = 1 → 显示原始灰度图 + 左边界(蓝)/右边界(红)/中线(绿)，用于标定
 *   VISION_PREVIEW = 0 → 只刷下方数字，纯粹测帧率
 * VISION_REFRESH_MS 为刷新周期（ms）；FPS 已扣除刷屏耗时，不受它影响。 */
#define VISION_PREVIEW      1
#define VISION_REFRESH_MS   1000

int16 LCorner, RCorner, centerLineCount, RUN_MODE = FORWARD;
volatile int16 LRoadCount = expRoadCount, RRoadCount = expRoadCount; //中断里做出界判断，需要 volatile
static int16 imBtn, borderInTriggerDistance = 25;
volatile uint8 borderIn; //主循环写、中断读
uint8 switch1, switch2, ALONG = ALONG_R, topBtn, bottomBtn;
Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount];
extern int16 currentDuty;
int16 data i;

#if (VISION_ONLY)
/* 视觉单测用的 1 ms 计时：只用于统计纯处理帧率（此模式不启动电机 PIT，TIM1 空闲） */
static volatile uint16 visionMs = 0;
static void visionTick(void) { visionMs++; }
#endif

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

/* 是否检测到赛道（由每次 loadRoad 的结果更新），上电后若一直不在赛道上，蓝灯慢闪 */
static uint8 onTrackFlag = 0;

/* ===== 蓝灯统一指示（优先级从上到下） =====
 *   编码器/PID 故障  → 常亮
 *   不在赛道上       → 慢闪 1 Hz（上电自检用：放回赛道后自动停闪）
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
    } else if (!onTrackFlag) {
        if (++ledMs >= 500) { // 不在赛道：慢闪
            ledMs = 0;
            ledPhase = (uint8) (!ledPhase);
        }
        ledWant = ledPhase ? STATUS_LED_ON : STATUS_LED_OFF;
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
    ips200_init();
    { //摄像头初始化：最多重试约 3 秒（150 × 20ms）。原写法无限重试，现场无法判断是摄像头故障；
      //但预算太短又会在上电瞬间误判失败，所以留足 3 秒。
        uint16 camRetry = 0;
        while (mt9v03x_init() && ++camRetry < 150) system_delay_ms(20);
        if (camRetry >= 150) {
            ips200_show_string(0, 120, "CAMERA INIT FAIL");
            ips200_show_string(0, 140, "check ribbon/power");
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
    //gpio_init(IO_P04, GPO, 0, GPO_PUSH_PULL); //示波器
    { //等 DMA 中断搬入第一帧完整快照再估阈值（带超时，防止摄像头异常时死等）
        uint16 wait = 0;
        while (!frameBusy && ++wait < 20000);
    }
    LThreshold = RThreshold = threshold = imageInitThreshold();
    frameBusy = 0;   // 消费开机首帧，让后续帧正常搬运
#if (ENABLE_SCREEN)
    ips200_displayimage03x(&mt9v03x_frame[0][0], MT9V03X_W, MT9V03X_H, 0, 0); //上电显示一帧原始画面，确认摄像头正常
#endif
#if (VISION_ONLY)
    { /* ===== 视觉单测：只需主板 + 摄像头 =====
       * 每帧都跑 loadRoad()；屏幕每 500 ms 才刷一次（刷屏要几十 ms，每帧都刷会把帧率压到 20~30）。
       * 这样 FPS 显示的是「纯处理帧率」，不受刷屏拖累，可直接判断算法够不够快。
       * 不调用 motorInit()，所以不启动 5 ms 控制 PIT，也不驱动电机/舵机/编码器。 */
        uint16 frames = 0, fps = 0, lastFrame = 0, lastSec = 0, lastDraw = 0, drawAcc = 0;
        pit_ms_init(TIM1_PIT, 1, visionTick); // 1 ms 计时，仅用于统计帧率
        while (1) {
            if (!frameBusy) continue;
            loadRoad();
            frames++;
            /* 帧率每秒结算一次，并把刷屏耗时从分母里扣掉，得到「纯处理帧率」。
             * 实测一次文本刷新要数百毫秒（每个字符都要重设显示窗口），若把刷屏时间算进去，
             * 读到的数字会远低于真实处理能力（之前总显示 5 就是这么来的）。 */
            if ((uint16) (visionMs - lastSec) >= 1000) {
                uint16 elapsed = (uint16) (visionMs - lastSec);
                if (elapsed > drawAcc)
                    fps = (uint16) ((uint32) (frames - lastFrame) * 1000 / (elapsed - drawAcc));
                lastFrame = frames;
                lastSec = visionMs;
                drawAcc = 0;
            }
            if ((uint16) (visionMs - lastDraw) >= VISION_REFRESH_MS) { // 读数刷新周期
                uint16 t0 = visionMs;
                lastDraw = visionMs;
#if (VISION_PREVIEW)
                // 刷屏期间 frameBusy 仍为 1，ISR 不会换帧，画面不会撕裂
                ips200_displayimage03x(&mt9v03x_frame[0][0], MT9V03X_W, MT9V03X_H, 0, 0);
                // 隔点绘制：点间距约 3 px，隔点仍然连贯，但绘制调用直接减半（draw_point 每次都要重设窗口，很贵）
                for (i = 0; i < LRoadCount; i += 2) ips200_draw_point(LRoad[i].y, LRoad[i].x, RGB565_BLUE);
                for (i = 0; i < RRoadCount; i += 2) ips200_draw_point(RRoad[i].y, RRoad[i].x, RGB565_RED);
                for (i = 0; i < centerLineCount; i += 2) ips200_draw_point(centerLine[i].y, centerLine[i].x, RGB565_GREEN);
#endif
                ips200_show_string(0, MT9V03X_H + 6, "L:");
                ips200_show_int16(16, MT9V03X_H + 6, LRoadCount);
                ips200_show_string(120, MT9V03X_H + 6, "R:");
                ips200_show_int16(136, MT9V03X_H + 6, RRoadCount);
                ips200_show_string(0, MT9V03X_H + 26, "C:");
                ips200_show_int16(16, MT9V03X_H + 26, centerLineCount);
                ips200_show_string(120, MT9V03X_H + 26, "THR:");
                ips200_show_int16(152, MT9V03X_H + 26, threshold);
                ips200_show_string(0, MT9V03X_H + 46, "LC:");
                ips200_show_int16(24, MT9V03X_H + 46, LCorner);
                ips200_show_string(120, MT9V03X_H + 46, "RC:");
                ips200_show_int16(144, MT9V03X_H + 46, RCorner);
                ips200_show_string(0, MT9V03X_H + 66, "FPS:");
                ips200_show_int16(40, MT9V03X_H + 66, (int16) fps);
                ips200_show_string(96, MT9V03X_H + 66, "FRAME:");
                ips200_show_int16(160, MT9V03X_H + 66, (int16) frames);
                drawAcc += (uint16) (visionMs - t0); // 累计本次刷屏耗时，供帧率结算扣除
            }
            frameBusy = 0; // 处理完才允许 ISR 搬下一帧
        }
    }
#endif
    /* ===== 上电自动选档：拨码 SW1/SW2 选 4 组，取该组中较保守的一档 =====
     * 按键已简化：B2 发车 / B3 紧急停车 / B4 舵机自检（长按 2 s = 标定一圈），
     * 因此不再需要按键选档，上电即按拨码定档并初始化电机与编码器。 */
    switch2 = gpio_get_level(IO_PB1);
    switch1 = gpio_get_level(IO_PB0);
    applySpeedSet((uint8) (((switch2 ? 2 : 0) + (switch1 ? 1 : 0)) * 2 + 1));
    RUN_MODE = FORWARD;
    stopInit();
    ips200_clear(RGB565_WHITE);
    ips200_show_string(0, 140, "TOP   : START");
    ips200_show_string(0, 160, "BOTTOM: SET LAP");
    ips200_show_string(0, 180, "IMAGE : STOP");
    frameBusy = 0;   // 丢弃选档期间累积的旧帧，第一拍处理发车后的当前画面
    while (1) {
        static uint16 noFrameTicks = 0;  // 帧看门狗：2 ms 节拍累计，约 1 s 无新帧则安全停车
        static uint8 lastRaceState = RACE_IDLE; // 用于检测 raceState 变化并刷新屏幕提示（初始 IDLE，保留发车提示）
        static uint16 lastFault = 0;       // 故障位变化时也刷一次屏（16 位，含 PID 异常位）
        static uint16 elemTicks = 0;       // 元素状态机停留帧数（超时兜底）
        static uint16 b4Hold = 0;          // B4 按住时长（1 ms 节拍），用于区分短按/长按
        /* 按键与里程判定不依赖相机帧：摄像头停帧时仍能急停 / 收车 */
        uint8 b2 = gpio_get_level(IO_PB2), b3 = gpio_get_level(IO_PB3), b4 = gpio_get_level(IO_PB4);
        /* B2：发车（完赛或急停后也可重新发车） */
        if (topBtn && !b2 && (raceState == RACE_IDLE || raceState == RACE_STOPPED)) {
            faultReset(); // 每次发车清空上一轮故障位（停车时保留，便于赛后查看原因）
            raceStart();
        }
        /* B3：紧急停车 */
        if (bottomBtn && !b3 && (raceState == RACE_RUNNING || raceState == RACE_BRAKING)) raceStopNow();
        /* B4：短按 = 舵机左右各摆 15°（自检）；长按 ≥2 s = 标定一圈 */
        if (!b4) {
            if (b4Hold < 2000) {
                if (++b4Hold == 2000 && raceState == RACE_IDLE) raceSetLap(); // 长按到 2 s 触发一次
            }
        } else {
            if (imBtn && b4Hold > 0 && b4Hold < 2000) servoSelfTest(); // 松开时判定为短按
            b4Hold = 0;
        }
        topBtn = b2;
        bottomBtn = b3;
        imBtn = b4;
        raceTask();
        /* 状态或故障位变化时刷一次屏（只在切换点发生，开销可忽略）；
         * 之前发车后屏幕仍停在三行静态提示上，容易被误认为停车了 */
        if (lastRaceState != raceState || lastFault != faultFlags) {
            lastRaceState = raceState;
            lastFault = faultFlags;
            ips200_clear(RGB565_WHITE);
            if (raceState == RACE_RUNNING)
                ips200_show_string(0, 130, "RUN");
            else if (raceState == RACE_BRAKING)
                ips200_show_string(0, 130, "BRAKE");
            else if (raceState == RACE_STOPPED)
                ips200_show_string(0, 130, "STOP");
            else
                ips200_show_string(0, 130, "READY");
            ips200_show_string(0, 160, "ERR:");
            ips200_show_int16(64, 160, (int16) faultFlags);
        }

        updateStatusLed(); // 蓝灯：故障 / 不在赛道 / 编码器方向 统一指示

        if (frameBusy) {
            noFrameTicks = 0;   // 收到完整帧，喂狗
            loadRoad();
            /* 是否在赛道上：任一侧边界超过 10 点即认为在赛道（上电自检与蓝灯都用它） */
            onTrackFlag = (uint8) (LRoadCount > 10 || RRoadCount > 10);
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
