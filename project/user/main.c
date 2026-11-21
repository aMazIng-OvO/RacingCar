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

/* 元素判据与流程已经全部搬到三个独立模块里：
 *   第一部分 SCurve.c  连续 S 弯
 *   第二部分 Round.c   环岛（左右共用，靠 side 区分）
 *   第三部分 XCross.c  十字形环岛（X 路口）
 * main.c 只负责"派发谁执行"与"总时长兜底"，不再持有任何元素阈值。 */

/* 总预算：三个模块各自有单阶段超时，这里再压一层总上限，
 * 保证任何元素都不会无限占用。 */
#define ELEM_TOTAL_MS       20000

/* 发车宽限：起步瞬间视觉还没稳定，这段时间不判出界。
 * 与 OUT_BLIND_MS(200 ms) 是两套独立机制，宽限期一过立刻按正常判据走。 */
#define OUT_START_GRACE_MS  300

/* ===== 出界判定参数（宁可晚停，不可误停） =====
 * 摄像头视场远大于赛道，画面里本来就有大量场地外的暗区，靠几个像素点判"冲出赛道"
 * 非常容易误判。现在的结论要三条信号同时成立，且都要持续：
 *   1. 5 个采样点全部偏暗（采样点已放到车身顶边以上、赛道内侧，见 outSamplePt）；
 *   2. 距最近一次"看到赛道"已超过 OUT_BLIND_MS（短暂丢边不判）；
 *   3. 连续 OUT_CONFIRM_FRAMES 帧都成立，之后控制中断再做 3 拍（15 ms）消抖。
 * 实车若觉得冲出后停得太晚，先调小 OUT_BLIND_MS，再考虑减少确认帧数。 */
#define OUT_SAMPLE_POINTS   5     //采样点个数
#define OUT_SEEN_POINTS     3     //任一侧边界点数超过该值即算"看到赛道"
#define OUT_BLIND_MS        200   //连续这么久没看到赛道才允许判出界
#define OUT_CONFIRM_FRAMES  3     //连续成立帧数（按帧计，约 30~60 ms）

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
/* 原始边界点数（拐点裁短之前）：环岛状态机的"边界消失→恢复"必须看原始值。
 * 下面的拐点分支会把 LRoadCount/RRoadCount 裁成 LCorner/RCorner（3~10），
 * 若状态机继续用裁短值判 ">10 且无拐点"，两个条件永远不能同时成立，
 * READY → START 在数学上不可达——这是"环岛进得去、走不完"的直接原因。 */
int16 LRawCount = 0, RRawCount = 0;
/* 阈值重估请求：elementReset() 可能在主循环帧处理块之外被调用（发车 / 完赛），
 * 那时 frameBusy == 0，DMA 中断可能正在整帧拷贝快照，直接扫像素会读到半帧。
 * 这里只置标志，真正的重估放到下一帧的 loadRoad() 里做。 */
static uint8 threshReload = 0;
static int16 borderInTriggerDistance = 25;

/* ===== 元素仲裁：同一时刻只有一个模块在跑 =====
 * 优先级 十字形环岛 > 环岛 > 连续 S 弯。占用期间其他模块的检测不执行，
 * 所以不存在"一个元素触发另一个元素"的串扰。 */
#define ELEM_NONE     0
#define ELEM_SCURVE   1
#define ELEM_ROUND    2
#define ELEM_XCROSS   3
static uint8 elemActive = ELEM_NONE;
/* 串口可观测量（只写不参与控制） */
static uint8 elemStageDbg = 0;   // 元素内部阶段
static uint8 elemCandDbg = 0;    // 环岛入口候选位：bit0~3 右、bit4~7 左
static uint8 roundScoreDbg = 0;  // 环岛入口积分进度（到 3 触发）

/* ===== 黑匣子：跑车时记录元素状态，停车后按 B4 上传 =====
 * 现场限制：跑车时数据线不够长，只能停车后读串口。所以把关键状态按 20 Hz
 * 记进 RAM（512 条 × 50 ms ≈ 25 s，够跑一整圈），停车后按 B4 一次性上传。
 * 每条的格式（逗号分隔，前缀 LOG）：
 *   LOG  t,MODE,STG,CAND,RS,L,R,C,DUTY,SPD,DS,DARK,H
 * 全部记录发完后以 ENDDDDD 结尾，看连续的 t 就能还原"进环岛那几秒到底发生了什么"。
 * 整行必须压到 64 字节以内：USB CDC 端点包只有 64 字节，超长会被拆成两包，
 * 调试助手按包显示时前半行（含 CAND/RS）就被截掉，现场日志因此看不到候选位。 */
#define ELEM_LOG_N   512
/* usb.h 里 DEVSTATE_CONFIGURED = 4，只有到这一步 USB 才真的能发数据 */
#define USB_STATE_CONFIGURED   4
typedef struct {
    uint16 t;      // 相对发车时刻的毫秒（<65 s 不会回绕）
    uint16 mode;   // RUN_MODE：810/1919 不能截成 uint8，否则日志显示 42/127
    uint8  stg;    // 元素内部阶段
    uint8  cand;   // 环岛候选位
    uint8  rs;     // 环岛积分
    uint8  l;      // LRawCount（截到 255）
    uint8  r;      // RRawCount
    uint8  c;      // centerLineCount
    int8   duty;   // 舵机偏移 / 4（压缩到 ±99，乘以 4 还原）
    uint16 err;    // faultFlags：0x0400 出界、0x0100 PID 饱和、0x0002/4 编码器掉线
    int16  spd;    // currentSpeed（脉冲 / 5 ms）
    int16  ds;     // setSpeed（目标速度）
    uint8  dark;   // 出界采样点暗点数 0~5
    uint8  huan;   // 1 = 当前处于环岛状态机（MODE=810/1919）
} elemLog_t;

static elemLog_t elemLog[ELEM_LOG_N];
static uint16 logW = 0;         // 写指针（512 超过 uint8 范围）
static uint16 logCount = 0;     // 有效条数
static uint16 logStartMs = 0;    // 发车时刻，用于算相对时间
static uint16 logLastMs = 0;     // 上次记录时刻
static uint8  logDump = 0;       // 1 = 正在往外吐
static uint16 logDumpIdx = 0;   // 吐到第几条
static uint8  logDumpReq = 0;    // B4 已请求上传；停车后由 debugReport 启动
volatile uint8 borderIn; //主循环写、中断读
uint8 switch1, switch2, ALONG = ALONG_R;
Point centerLine[expRoadCount], LRoad[expRoadCount], RRoad[expRoadCount];
extern int16 currentDuty;
extern int16 aimRowDbg;       // alongLine 本帧前瞻点行号，debugReport 的 AIM 字段
/* USB 设备状态（usb.h 里是 extern BYTE DeviceState）。
 * 只有等于 DEVSTATE_CONFIGURED(4) 时 usb_cdc_write_buffer 才会真正发送，
 * 否则 printf 一个字节都出不去。这里手工声明，避免把整个 usb.h 拉进来。 */
extern uint8 DeviceState;
extern uint16 dropFrameCnt;   // Image.c 统计的丢帧数，debugReport 的 DROP 字段



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
    /* 第 7 参数是纯跟踪等效轴距。原来固定传 0（等于让 motorInit 保持内部默认），
     * 结果 Motor.h 里"调 c7 把等效轴距往 30~50 加"这条建议在工程里没有入口。
     * 现在统一由 PP_LOOKAHEAD_PX 给出，改那一个宏即可对 8 档同时生效。 */
    motorInit(speedSet[idx][0], speedSet[idx][1], speedSet[idx][2], speedSet[idx][3],
              PP_LOOKAHEAD_PX);
}

void loadRoad();

/* ===== 按键模块（B2/B3/B4，低电平有效） =====
 * 时间基准是控制中断维护的 msTick，不是循环次数：
 * 主循环处理一帧图像要 5~10 ms，用循环次数计时会把 20 ms 消抖拉成上百毫秒、
 * 把 2 s 长按拉成十几秒，现场表现就是"按键失效"。
 *   B2：短按 = 发车（需已识别到赛道）    长按 1 s = 强制发车
 *   B3：短按 = 紧急停车（任何状态都生效）
 *   B4：短按 = 上传黑匣子（停车后执行）  长按 2 s = 标定一圈 */
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
    scReset();
    rdReset();
    xcReset();
    elemActive = ELEM_NONE;
    controlReset();
    RUN_MODE = FORWARD;
    ALONG = ALONG_R;
    borderIn = 0;
    centerLineDistance = 18;
    centerLineCount = 0;
    LCorner = RCorner = 0;
    LRoadCount = RRoadCount = expRoadCount;
    LRawCount = RRawCount = 0;
    outOfTrack = 0;
    /* 元素输出量复位：否则超时/急停后车还带着环岛打角或元素限速 */
    elemSteerBiasDeg = 0;
    elemSpeedRatio = 100;
    /* 重新发车/元素超时复位时重估阈值：原来阈值只在 main() 初始化时估一次，
     * 若视觉曾进入"爬线失败→阈值冻结"的死锁，停车后按 B2 也起不来，
     * 只能断电。这里只置标志，由下一帧的 loadRoad() 在快照空闲时执行，
     * 避免在 DMA 整帧拷贝期间读到半帧。 */
    threshReload = 1;
    gpio_set_level(IO_P90, 0);
}

/* ===== 出界判定的时间/帧数状态 =====
 * outSeenMs 是最近一次"看到赛道"（任一侧边界点数够）的时刻，用来实现"短暂丢边不判"；
 * outFrameCnt 是"5 点全暗且长时间没看到赛道"连续成立的帧数；
 * darkCount 是最近一帧的暗点数，作为串口 DARK 字段输出，现场标定采样点时看它。 */
static uint16 outSeenMs = 0;
static uint8 outFrameCnt = 0;
static uint8 darkCount = 0;

/* ===== USB CDC 调试输出 =====
 * 默认只在按下 B4 后上传黑匣子；平时不发送周期性数据，避免跑车时占用 USB/主循环。
 * 黑匣子格式：LOG t,MODE,STG,CAND,RS,L,R,C,DUTY,SPD,DS,DARK,H；全部发完后固定输出 ENDDDDD。
 * 需要恢复原来的 ODO 周期调试行时，把 ENABLE_LIVE_TELEMETRY 改为 1。 */
#define ENABLE_LIVE_TELEMETRY   0

static void debugReport(uint8 running) {
    static uint16 lastMs = 0;
    uint16 now = msTick;

    /* B4 请求只在停车后启动，避免上传过程拖慢行驶中的控制循环。
     * 如果 B4 是在行驶中按下的，请求会保留到 RACE_STOPPED 后再执行。 */
    if (!logDump && logDumpReq && !running) {
        logDump = 1;
        logDumpIdx = 0;
        logDumpReq = 0;
        lastMs = (uint16) (now - 50);   // 下一拍立即发第一条
    }

#if (ENABLE_LIVE_TELEMETRY)
    if (!logDump && (uint16) (now - lastMs) < (uint16) (running ? 1000 : 250)) return;
#else
    if (!logDump) return;
    /* 上传节奏与记录节奏同为 50 ms：既不会比记录快而空转，
     * 也给 USB 主机侧留出取包时间，避免缓冲堆积后按包截断。 */
    if ((uint16) (now - lastMs) < 50) return;
#endif
    lastMs = now;

    /* ===== 黑匣子回放 =====
     * 跑车时数据线接不上（不够长），所以把这一轮的元素状态存在 RAM 里，
     * 停车后按 B4 再上传。格式：
     *   LOG  t,MODE,STG,CAND,RS,L,R,C,DUTY,SPD,DS,DARK,H
     * t 是相对发车时刻的毫秒，看相邻几条就能还原"进环岛那几秒"发生了什么。
     * SPD/DS 是实际/目标速度，DARK 是出界采样点暗点数，
     * H=1 表示当前处于环岛状态机（MODE=810/1919）。
     * 去掉 ERR、把 Huan 缩成 H，整行压到 64 字节以内：原来 14 字段约 65 字节，
     * 超过 USB CDC 的 64 字节端点包会被拆包，调试助手只显示后半行，
     * 前半行的 MODE/STG/CAND/RS 全部丢失。faultFlags 仍存在结构体里。 */
    if (logDump) {
        if (logDumpIdx < logCount) {
            int16 k = (int16) logW - (int16) logCount + (int16) logDumpIdx;
            char dl[80];
            int16 dn;
            while (k < 0) k += ELEM_LOG_N;
            dn = (int16) sprintf(dl, "LOG %u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%u,H%u\r\n",
                                 (unsigned) elemLog[k].t, (unsigned) elemLog[k].mode,
                                 (unsigned) elemLog[k].stg, (unsigned) elemLog[k].cand,
                                 (unsigned) elemLog[k].rs, (unsigned) elemLog[k].l,
                                 (unsigned) elemLog[k].r, (unsigned) elemLog[k].c,
                                 (int) elemLog[k].duty * 4,
                                 (int) elemLog[k].spd, (int) elemLog[k].ds,
                                 (unsigned) elemLog[k].dark, (unsigned) elemLog[k].huan);
            if (dn > 0) usb_cdc_write_buffer((const uint8 *) dl, (uint16) dn);
            logDumpIdx++;
            return;                      // 这一拍只发一条日志
        }
        /* 全部记录已发完：结束回放并给出固定结束标记。
         * 数据仍留在 RAM 里，再按一次 B4 可以从头再传一遍。 */
        logDump = 0;
        logDumpIdx = 0;
        {
            static const char endMark[] = "ENDDDDD\r\n";
            usb_cdc_write_buffer((const uint8 *) endMark, (uint16) (sizeof(endMark) - 1));
        }
        return;                          // ENDDDDD 之后不再输出任何内容
    }

#if (ENABLE_LIVE_TELEMETRY)
    {
        int32 odo;
        interrupt_global_disable();
        odo = odometer;
        interrupt_global_enable();
        /* 一次格式化 + 一次发送。
         * 不能用 printf：它经 putchar 逐字符调 usb_cdc_write_byte()，每个字符一次
         * usb_cdc_write_buffer(…,1)，而单次调用最多等 USB_TIME_OUT_COUNT(≈2 ms)。
         * 电脑端一旦读得慢（串口助手缓冲满、切窗口），一整行 130 个字符就会阻塞
         * 130 × 2 ms ≈ 260 ms，主循环直接瘫掉——实测 DROP 涨到 80 帧/秒就是这么来的。
         * 改成一次写 130 字节，最多 3 个 64 字节包，最坏也只等 3 次超时。 */
        {
            char line[168];
            int16 n;
            n = (int16) sprintf(line,
                    "ODO=%ld L=%d R=%d C=%d AIM=%d THR=%d DARK=%d DROP=%u MODE=%d STG=%d CAND=%d RS=%d AL=%d STA=%d ERR=%d SPD=%d DS=%d DUTY=%d\r\n",
                    odo, LRawCount, RRawCount, centerLineCount, aimRowDbg, threshold, darkCount,
                    dropFrameCnt, RUN_MODE, elemStageDbg, elemCandDbg, roundScoreDbg, ALONG,
                    trackIsOn(), faultFlags, currentSpeed, setSpeed, currentDuty);
            if (n > 0) usb_cdc_write_buffer((const uint8 *) line, (uint16) n);
        }
    }
#endif
}

/* ===== 出界采样点 =====
 * 采样点同时受三条约束：
 *   1. 行必须小于 BODY_ROW（车身在画面里的最上行），否则读到的是车身暗块。
 *      原来近端三点放在 84~88 行，车身顶边到 80 行后这三点长期就是暗的，
 *      远端两点一暗就凑齐"5 点全暗"，直道上偶发停车多半出在这里；
 *   2. 列要落在赛道内侧。摄像头视场远大于赛道，画面左右两侧是场地外暗区，
 *      原 47 / 141 在偏窄赛道上已经在线外，等于又白送两个暗点；
 *   3. 越远的行赛道越窄，列要同步内收，保持 5 个点都落在赛道里。
 *   4. 全部落在视野窗口内（行 34~99、列 34~153），窗口外是写成暗区的区域。
 * 现场标定：直道正常行驶时这 5 个点的暗点数应为 0（串口 DARK 字段可观察）。 */
#define OUT_ROW_NEAR  (BODY_ROW - 4)   //近端采样行 = 76，车身顶边以上留 4 像素余量

/* 左右侧点收到 ±20：仿真里窄赛道（宽 40，列 74~114）时 ±24 会落到赛道外，
 * 单侧再丢边就凑到 5 点全暗；±20 在窄赛道刚好压在边界上，宽赛道不受影响。 */
static const uint8 outSamplePt[OUT_SAMPLE_POINTS][2] = {
    {OUT_ROW_NEAR,      94},  //车头正前方
    {OUT_ROW_NEAR - 2,  74},  //左前
    {OUT_ROW_NEAR - 2, 114},  //右前
    {OUT_ROW_NEAR - 10, 80},  //左远（行更远、赛道更窄，列内收）
    {OUT_ROW_NEAR - 10, 108}  //右远
};

/* 返回 5 个采样点里低于阈值的个数。每点取 3×3 块均值，
 * 单像素反光或坏点不足以把整个采样点判成暗。 */
static uint8 darkPoints(void) {
    uint8 n = 0, k, i, j;
    uint16 sum;

    for (k = 0; k < OUT_SAMPLE_POINTS; k++) {
        sum = 0;
        for (i = outSamplePt[k][0] - 1; i <= outSamplePt[k][0] + 1; i++)
            for (j = outSamplePt[k][1] - 1; j <= outSamplePt[k][1] + 1; j++)
                sum += mt9v03x_frame[i][j];
        if ((uint8) (sum / 9) < threshold) n++;
    }
    return n;
}

void main(void) {
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();
    /* ===== 启动横幅 =====
     * USB CDC 是在 debug_init() 里初始化的，电脑要等枚举完成才能打开串口，
     * 所以先停 300 ms 再打——否则第一行会落在端口打开之前，看起来像"完全没输出"。
     * 这两行也是现场判断"程序跑到哪一步"的唯一依据：
     *   只看到 BOOT      → 卡在摄像头初始化
     *   看到 BOOT + CAM  → 摄像头正常，后面是按键/发车/主循环
     *   一行都没有       → USB 没枚举成功，或者端口选错（设备管理器里找"USB 串行设备"） */
    system_delay_ms(300);
    printf("BOOT v1.4\r\n");
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
    printf("CAM ok\r\n");   // 摄像头初始化成功：走到这里说明后面只剩按键/发车/主循环
    /* ===== 用状态灯编码 USB 状态（没有串口也能读）=====
     * 闪灯次数 = DeviceState：
     *   0 未连接 / 1 已上电 / 2 已复位 / 3 已分配地址 / 4 已配置 / 5 挂起
     * 读到 4 次：固件侧发送条件已满足，printf 一定把数据推进 USB，
     *            剩下的问题全在电脑端（端口没打开 / 工具不兼容）。
     * 读到 0~3：USB 没配置成功，usb_cdc_write_buffer() 会在第一行就 return，
     *            任何串口工具都不可能收到数据，要查线/供电/驱动/时钟。
     * 这一段在"上电闪 3 下"之后、舵机自检之前，靠时序也能区分。 */
    {
        uint8 st = (uint8) DeviceState;
        uint8 n;
        system_delay_ms(400);
        for (n = 0; n < st; n++) {
            gpio_set_level(STATUS_LED_PIN, STATUS_LED_ON);
            system_delay_ms(150);
            gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
            system_delay_ms(150);
        }
        system_delay_ms(600);
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
    gpio_init(IO_PB4, GPI, 0, GPI_PULL_UP);   // B4 黑匣子上传 / 标定
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
     * 按键已简化：B2 发车 / B3 紧急停车 / B4 黑匣子上传（长按 2 s = 标定一圈），
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
        static uint16 elemSessionMs = 0; // 本次元素会话的起始时刻（总预算用）
        static uint8 elemSessionOn = 0;  // 1 = 正处于某个元素中
        static uint16 runStartMs = 0;    // 最近一次进入 RACE_RUNNING 的时刻
        static uint8 lastRaceState = 0;  // 上一次的比赛状态，用于捕捉状态切换
        /* 记下发车时刻：出界判定在发车后 OUT_START_GRACE_MS 内不生效。
         * 同时清掉出界判定的盲区计时与帧计数，避免沿用上一轮的"很久没看到赛道"。 */
        if (raceState != lastRaceState) {
            if (raceState == RACE_RUNNING) {
                runStartMs = msTick;
                outSeenMs = msTick;
                outFrameCnt = 0;
                darkCount = 0;
                /* 发车：黑匣子从头开始记 */
                logStartMs = msTick;
                logLastMs = msTick;
                logW = 0;
                logCount = 0;
                logDump = 0;
                logDumpIdx = 0;
                logDumpReq = 0;   // 新一轮发车取消上一轮尚未执行的 B4 请求
            }
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
        /* B3：紧急停车（任何状态都生效）。黑匣子上传统一由 B4 触发，
         * 这样不会在还没插好数据线时把数据发进空气里。 */
        if (btnPressEdge & BTN_B3) {
            if (raceState == RACE_RUNNING || raceState == RACE_BRAKING) {
                raceStopNow();
                motorForceStop(); // 双保险：直接清零两路占空比并取消助推
            }
        }
        /* B4 长按 ≥2 s：标定一圈（按住到阈值时触发一次） */
        if (btnLongEvent) {
            btnLongEvent = 0;
            if (raceState == RACE_IDLE) raceSetLap();
        }
        /* B4 短按（松开时判定，且本次不是长按）：上传黑匣子。
         * 行驶中按下只登记请求，等停车后再上传；已经在上传时重按则从头再传。 */
        if (btnShortB4) {
            btnShortB4 = 0;
            if (logDump) {
                logDump = 0;
                logDumpIdx = 0;
            }
            logDumpReq = 1;
        }
        raceTask();

        trackMonitor(); // 蓝灯：故障常亮 / 正向快闪 / 反向常亮 / 静止熄灭

        if (frameBusy) {
            lastFrameMs = msTick;   // 收到完整帧，喂狗
            loadRoad();
            /* 出界判定放在这里：此刻快照不会被 DMA 搬运覆盖，读像素是安全的；
             * 结论交给控制中断做 15 ms 消抖，控制中断不再直接访问整帧图像。 */
            trackEvaluate();        // 用同一份边界点数更新赛道状态，供发车许可与蓝灯使用
#if (ENABLE_BEND_ENTRY)
            /* 入弯过渡区识别：在直道与弯道的连接处，画面里没有边界是正常现象。
             * 模块自己维护状态机，只向这里提供出界宽容等级。 */
            bendEntryUpdate(centerLine, centerLineCount, getAimPos());
#endif
            {
                uint8 dark = darkPoints();
                uint8 guard = 0;
                uint8 blind;
#if (ENABLE_BEND_ENTRY)
                guard = bendEntryGuard();
#endif
                darkCount = dark;   // 串口观测用：直道正常行驶时应为 0
                /* 任一侧边界点数够就算"看到赛道"，把盲区计时清零。
                 * 这样"看不见赛道"必须持续 OUT_BLIND_MS 才成立，短暂丢边不算。
                 * 必须用裁短前的 LRawCount / RRawCount：LRoadCount 在拐点分支里会被
                 * 覆盖成 LCorner（3~13），十字与环岛阶段恒小于等于门槛，
                 * 用裁短值判"看到赛道"会让 blind 在整个元素期间一直成立，
                 * 元素里的黑区一旦凑够暗点就会把车停死。 */
                if (LRawCount > OUT_SEEN_POINTS || RRawCount > OUT_SEEN_POINTS) outSeenMs = msTick;
                blind = (uint8) ((uint16) (msTick - outSeenMs) >= OUT_BLIND_MS);

                if (guard >= 2) {
                    /* 连接处时间窗内：完全免判，画面里的黑区是弯道外侧的底布 */
                    outFrameCnt = 0;
                    outOfTrack = 0;
                } else if (dark >= OUT_SAMPLE_POINTS && blind) {
                    /* 5 点全暗 + 已经 OUT_BLIND_MS 没看到赛道，两条同时成立才开始计帧。
                     * 任一条不成立立刻清零：直道上偶发的阴影、反光、丢边都不会停车。 */
                    if (outFrameCnt < 100) outFrameCnt++;
                    outOfTrack = (uint8) (outFrameCnt >= OUT_CONFIRM_FRAMES);
                } else {
                    outFrameCnt = 0;
                    outOfTrack = 0;
                }
                /* 发车后的宽限期：起步瞬间视觉还没稳定，先不判出界 */
                if ((uint16) (msTick - runStartMs) < OUT_START_GRACE_MS) {
                    outFrameCnt = 0;
                    outSeenMs = msTick;
                    outOfTrack = 0;
                }
            }
            if (raceState == RACE_RUNNING || raceState == RACE_BRAKING) {
                /* ================= 第一部分~第三部分的仲裁 =================
                 * 三个模块互斥，同一时刻只有一个在跑，优先级：
                 *     十字形环岛 XCross > 环岛 Round > 连续 S 弯 SCurve
                 * 占用期间其他模块的 detect 一律不调用，所以不会出现
                 * "在环岛里被 S 弯抢走"或"路口里被判成环岛"这类串扰。
                 * 每个模块自己判退出（条件消失 / 超时），这里只兜总时长。 */
                if (elemActive == ELEM_XCROSS) {
                    if (xcBusy()) xcRun();
                    else { xcExit(); elemActive = ELEM_NONE; }
                } else if (elemActive == ELEM_ROUND) {
                    if (rdBusy()) rdRun();
                    else { rdExit(); elemActive = ELEM_NONE; }
                } else if (elemActive == ELEM_SCURVE) {
                    if (scBusy()) scRun();
                    else { scExit(); elemActive = ELEM_NONE; }
                }

                if (elemActive == ELEM_NONE) {
                    if (xcDetect()) {
                        elemActive = ELEM_XCROSS;
                        xcEnter();
                        xcRun();
                    } else if (rdDetect()) {
                        elemActive = ELEM_ROUND;
                        rdEnter();
                        rdRun();
                    } else if (scDetect()) {
                        elemActive = ELEM_SCURVE;
                        scEnter();
                        scRun();
                    } else {
                        int8 beTurn = 0;
                        /* 普通循迹：三个元素输出量在这一支里统一复位 */
                        elemSteerBiasDeg = 0;
                        elemSpeedRatio = 100;
                        /* 连接处中线点数不足时用入弯前冻结的转角维持转向，
                         * 视觉一丢失就把方向打直，车会切进内侧或冲出去 */
                        if (ENABLE_BEND_ENTRY && bendEntryTurn(&beTurn))
                            setServoDuty(beTurn);
                        else
                            alongLine(centerLine, centerLineCount, getAimPos());
                    }
                }

                /* RUN_MODE 现在只用于两件事：串口 MODE 字段、以及 loadRoad() 里
                 * "只在 FORWARD 时重算 borderIn / centerLineDistance"的分支。
                 * 控制量全部由模块通过三个输出量给出，不再走 ALONG。 */
                if (elemActive == ELEM_XCROSS) RUN_MODE = RUN_X_CROSS;
                else if (elemActive == ELEM_ROUND) RUN_MODE = rdSide() ? RUN_ROUND_L : RUN_ROUND_R;
                else if (elemActive == ELEM_SCURVE) RUN_MODE = RUN_S_CURVE;
                else RUN_MODE = FORWARD;

                /* 串口诊断：STG = 元素内部阶段，CAND/RS = 环岛入口候选与积分 */
                if (elemActive == ELEM_XCROSS) elemStageDbg = xcStage();
                else if (elemActive == ELEM_ROUND) elemStageDbg = rdStage();
                else if (elemActive == ELEM_SCURVE) elemStageDbg = scStage();
                else elemStageDbg = 0;
                elemCandDbg = rdCandBits();
                roundScoreDbg = rdScore();

                /* 黑匣子：20 Hz 记一条，停车后按 B4 再上传 */
                if ((uint16) (msTick - logLastMs) >= 50) {
                    logLastMs = msTick;
                    elemLog[logW].t = (uint16) (msTick - logStartMs);
                    elemLog[logW].mode = (uint16) RUN_MODE;
                    elemLog[logW].stg = elemStageDbg;
                    elemLog[logW].cand = elemCandDbg;
                    elemLog[logW].rs = roundScoreDbg;
                    elemLog[logW].l = (uint8) (LRawCount > 255 ? 255 : LRawCount);
                    elemLog[logW].r = (uint8) (RRawCount > 255 ? 255 : RRawCount);
                    elemLog[logW].c = (uint8) (centerLineCount > 255 ? 255 : centerLineCount);
                    /* duty 压到 1/4 存，±395 → ±98，一字节够用 */
                    elemLog[logW].duty = (int8) (currentDuty / 4);
                    elemLog[logW].err = faultFlags;
                    elemLog[logW].spd = currentSpeed;
                    elemLog[logW].ds = setSpeed;
                    elemLog[logW].dark = darkCount;
                    elemLog[logW].huan = (uint8) (elemActive == ELEM_ROUND);
                    /* logW 是 uint16，不能强转 uint8：512 取模的结果是 0~511，
                     * 截成 8 位后写指针只在 0~255 循环，256~511 号槽位永远是
                     * 初始 0；而 logCount 仍会累加到 512，回放时就把半区空槽
                     * 一并吐出来（现场日志前 200 多条 LOG 0,...,H0 即由此产生）。 */
                    logW = (uint16) ((logW + 1) % ELEM_LOG_N);
                    if (logCount < ELEM_LOG_N) logCount++;
                }

                /* 总时长兜底：任何元素最多占用 ELEM_TOTAL_MS */
                if (elemActive != ELEM_NONE) {
                    if (!elemSessionOn) {
                        elemSessionOn = 1;
                        elemSessionMs = msTick;
                    } else if ((uint16) (msTick - elemSessionMs) > ELEM_TOTAL_MS) {
                        elementReset();
                    }
                } else {
                    elemSessionOn = 0;
                }
            } else {
                elemSessionOn = 0;
                elemStageDbg = 0;
                elemCandDbg = 0;
                roundScoreDbg = 0;
                /* 离开比赛状态：元素模块全部退出，并把 RUN_MODE 复位。
                 * 少了这一步，急停后串口 MODE 会一直停在最后那个元素值（例如 810），
                 * 看起来像"卡在环岛里"，实际只是显示没复位。 */
                if (elemActive != ELEM_NONE) {
                    scExit();
                    rdExit();
                    xcExit();
                    elemActive = ELEM_NONE;
                }
                RUN_MODE = FORWARD;
                elemSteerBiasDeg = 0;
                elemSpeedRatio = 100;
                /* 静态标定：停车时也实时算环岛候选位。
                 * 元素模块本身不运行（它们只在 RUNNING/BRAKING 下被调用），
                 * 但 rdCandBits() 是只读的，只反映本帧视觉。
                 * 推着车沿赛道逐点走，看 CAND 就知道"如果车在跑，这里会不会
                 * 被判成环岛入口"，以及缺的是哪一条：
                 *   右环岛 bit0 外侧>16点 / bit1 外侧末点行<=42 / bit2 内侧<7点 /
                 *   bit3 外侧够直；左环岛 bit4/5/6/7 镜像
                 *   → 0x0F / 0xF0 才是完整候选，bit3/7 为 0 更像普通急弯 */
                elemCandDbg = rdCandBits();
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
        /* 对侧点数少于 3 时不做配对：游标卡在 oc-1 附近，
         * 后续骨架点会持续误配到同一个点，不如直接按单侧外推。 */
        if (oc >= 3) {
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
    Point LStart, RStart;

    /* 先按视野窗口屏蔽四周的视场外区域，再找起点：远端行、近端车身、
     * 左右边缘的反光与底布会被爬线当成赛道，拟合出不存在的边界。 */
    cutViewRegion();
    /* 复位后请求的阈值重估放在这里：此刻主循环持有快照（frameBusy == 1），
     * DMA 不会再往里写，读整帧是安全的 */
    if (threshReload) {
        threshReload = 0;
        LThreshold = RThreshold = threshold = imageInitThreshold();
    }
    LStart = getLStart();
    RStart = getRStart();
    if (RUN_MODE == FORWARD) centerLineDistance = 18; //普通循迹恢复默认补线偏移
    LRoadCount = expRoadCount, RRoadCount = expRoadCount;
    getLBorder(LRoad, LStart, &LRoadCount);
    getRBorder(RRoad, RStart, &RRoadCount);
    LBorderCount = func_limit_ab(LRoadCount, 0, expRoadCount);
    memcpy(LBorder, LRoad, LBorderCount * sizeof(Point));
    RBorderCount = func_limit_ab(RRoadCount, 0, expRoadCount);
    memcpy(RBorder, RRoad, RBorderCount * sizeof(Point));
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
    LRawCount = LRoadCount;   // 拐点裁短前的原始点数，供环岛状态机与入口判据使用
    RRawCount = RRoadCount;
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
        /* 首点锚到视野窗口下沿，而不是画面底边（行 119）：近端 20 行已裁掉，
         * 锚到 119 会在首点与第一个真实点之间插进十个纯插值点，
         * 前瞻点落在这段合成线上，横向偏差被拉平。 */
        for (idx = 1; idx < centerLineCount; idx++)
            if (centerLine[idx].x < VIEW_NEAR_ROW - 2) { startPos = idx - 1; break; }
        centerLine[startPos].x = VIEW_NEAR_ROW;
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
