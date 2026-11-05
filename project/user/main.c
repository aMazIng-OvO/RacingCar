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

int16 LCorner, RCorner, centerLineCount, RUN_MODE = FORWARD;
volatile int16 LRoadCount = expRoadCount, RRoadCount = expRoadCount; //中断里做出界判断，需要 volatile
static int16 imBtn, borderInTriggerDistance = 25;
volatile uint8 borderIn; //主循环写、中断读
uint8 switch1, switch2, ALONG = ALONG_R, topBtn, bottomBtn;
Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount];
extern int16 currentDuty;
int16 data i, j, pos;

void loadRoad();

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
    { //摄像头初始化最多重试 10 次；失败则提示并停在安全状态（原写法会无限重试，现场无法判断是摄像头故障）
        uint8 camRetry = 0;
        while (mt9v03x_init() && ++camRetry < 10) system_delay_ms(20);
        if (camRetry >= 10) {
            ips200_show_string(0, 120, "CAMERA INIT FAIL");
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
    gpio_init(IO_P02, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P01, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P32, GPI, 0, GPI_PULL_UP);
    gpio_init(IO_P90, GPO, 0, GPO_PUSH_PULL); //蜂鸣器
    //gpio_init(IO_P04, GPO, 0, GPO_PUSH_PULL); //示波器
    { //等 DMA 中断搬入第一帧完整快照再估阈值（带超时，防止摄像头异常时死等）
        uint16 wait = 0;
        while (!frameBusy && ++wait < 20000);
    }
    LThreshold = RThreshold = threshold = imageInitThreshold();
    frameBusy = 0;   // 消费开机首帧，让后续帧正常搬运
    ips200_displayimage03x(&mt9v03x_frame[0][0], MT9V03X_W, MT9V03X_H, 0, 0); //上电显示一帧原始画面，确认摄像头正常
    while (1) {
        system_delay_ms(50);
        switch2 = gpio_get_level(IO_PB1);
        switch1 = gpio_get_level(IO_PB0);
        topBtn = gpio_get_level(IO_P02);
        bottomBtn = gpio_get_level(IO_P01);
        imBtn = gpio_get_level(IO_P32);
        // 图像调试预览已移除（原依赖 outputImage，已删除以腾出 XRAM 给双缓冲）
        if (!switch1 && !switch2) {
            ips200_show_string(0, MT9V03X_H + 10, "T: SPEED 1");
            ips200_show_string(0, MT9V03X_H + 40, "B: SPEED 2");
            if (!topBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 130, "SELECT: 1");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(260, 1000, 26, 4500, 520, 0, 0);
                break;
            }
            if (!bottomBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 160, "SELECT: 2");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(200, 800, 22, 4000, 300, 0, 0);
                break;
            }
        } else if (switch1 && !switch2) {
            ips200_show_string(0, MT9V03X_H + 10, "T: SPEED 3");
            ips200_show_string(0, MT9V03X_H + 40, "B: SPEED 4");
            if (!topBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 130, "SELECT: 3");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(210, 900, 23, 4000, 320, 0, 0);
                break;
            }
            if (!bottomBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 160, "SELECT: 4");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(220, 1000, 23, 4000, 340, 0, 0);
                break;
            }
        } else if (!switch1 && switch2) {
            ips200_show_string(0, MT9V03X_H + 10, "T: SPEED 5");
            ips200_show_string(0, MT9V03X_H + 40, "B: SPEED 6");
            if (!topBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 130, "SELECT: 5");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(230, 1000, 24, 4000, 360, 0, 0);
                break;
            }
            if (!bottomBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 160, "SELECT: 6");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(240, 1000, 24, 4000, 380, 0, 0);
                break;
            }
        } else {
            ips200_show_string(0, MT9V03X_H + 10, "T: SPEED 7");
            ips200_show_string(0, MT9V03X_H + 40, "B: SPEED 8");
            if (!topBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 130, "SELECT: 7");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(250, 1000, 25, 4000, 450, 0, 0);
                break;
            }
            if (!bottomBtn) {
                ips200_clear(RGB565_WHITE);
                ips200_show_string(0, 160, "SELECT: 8");
                system_delay_ms(500);
                system_delay_ms(500);
                motorInit(260, 1000, 25, 4000, 500, 0, 0);
                break;
            }
        }
    }
    RUN_MODE = FORWARD;
    stopInit();
    ips200_clear(RGB565_WHITE);
    ips200_show_string(0, 140, "TOP   : START");
    ips200_show_string(0, 160, "BOTTOM: SET LAP");
    ips200_show_string(0, 180, "IMAGE : STOP");
    frameBusy = 0;   // 丢弃选档期间累积的旧帧，第一拍处理发车后的当前画面
    while (1) {
        static uint16 noFrameTicks = 0;  // 帧看门狗：2 ms 节拍累计，约 1 s 无新帧则安全停车
        /* 按键与里程判定不依赖相机帧：摄像头停帧时仍能急停 / 收车 */
        uint8 tBtn = gpio_get_level(IO_P02), bBtn = gpio_get_level(IO_P01), sBtn = gpio_get_level(IO_P32);
        if (topBtn && !tBtn && (raceState == RACE_IDLE || raceState == RACE_STOPPED)) raceStart();
        if (bottomBtn && !bBtn && raceState == RACE_IDLE) raceSetLap();
        if (imBtn && !sBtn && (raceState == RACE_RUNNING || raceState == RACE_BRAKING)) raceStopNow();
        topBtn = tBtn;
        bottomBtn = bBtn;
        imBtn = sBtn;
        raceTask();

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
            } else if (raceState == RACE_IDLE) {
                raceShow();
            }
            frameBusy = 0;   // 消费完毕，允许 ISR 搬入下一帧
        } else if (++noFrameTicks > 500) {
            /* 稳定优先：摄像头掉帧 / DMA 异常时，不允许保持上一拍打角继续全速跑 */
            noFrameTicks = 0;
            if (raceState == RACE_RUNNING || raceState == RACE_BRAKING) raceStopNow();
        }
        system_delay_ms(2);   // 主循环 2 ms 节拍：按键去抖 + 帧看门狗计时基准
    }
}

void loadRoad() {
    int16 idx;
    uint8 leftFound = 0, rightFound = 0;
    Point LStart = getLStart(), RStart = getRStart();
    if (RUN_MODE == FORWARD) centerLineDistance = 18; //普通循迹恢复默认补线偏移
    LRoadCount = expRoadCount, RRoadCount = expRoadCount;
    getLBorder(LRoad, LStart, &LRoadCount, 0);
    getRBorder(RRoad, RStart, &RRoadCount, 0);
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
    /* borderIn：左右各自独立判定，标志绝不跨分支复用；索引前先确认 count>0 */
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