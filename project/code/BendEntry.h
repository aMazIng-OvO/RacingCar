//
// 弯道入口过渡区识别（独立模块）
//
// 解决的问题：直道与弯道的连接处，车头已经转向，画面里弯道外侧边界跑出视野、
// 内侧边界被 turn>=4 打断，两侧点数同时归零，出界保护随即在 15 ms 内停车。
//
// 本模块不修改 Image.c / Motor.c，只读取它们每帧产出的边界与中线，
// 维护一条独立的入弯状态机，向调用方提供两样东西：
//   1. 出界宽容等级——连接处不把"看不到边界"当成"冲出赛道"；
//   2. 转角保持——视觉彻底丢失时沿用入弯前的转向角，避免打直。
//
// 与竞速主程序的关系：ENABLE_BEND_ENTRY = 0 时整段逻辑不参与编译，
// 行为与接入前完全一致，可以直接对比两种配置的跑法。
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_BENDENTRY_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_BENDENTRY_H
#include "zf_common_headfile.h"

/* 总开关：1 = 启用入弯过渡区识别，0 = 完全旁路（编译期剔除） */
#define ENABLE_BEND_ENTRY       1

/* ===== 入弯判定 =====
 * 强度与 Motor.c 的 bend 同量纲：|远端偏差 - 前瞻偏差| + |转角|。
 * 直道约 0~5，缓弯 10~20，急弯 25 以上。 */
#define BE_BEND_MIN             10    // 强度超过该值开始累积"同向确认"
#define BE_BEND_EXIT            4     // 强度回落到该值以下视为离开弯道
#define BE_CONFIRM_FRAMES       2     // 连续同向帧数达到该值才进入 APPROACH；连续弯道需要快速重新确认

/* ===== 连接处容忍 =====
 * 进入 COMMITTED 后，画面里没有边界是正常现象，不能按"冲出赛道"处理。 */
#define BE_BLANK_MS             250   // 完全免判出界的时间窗（毫秒）
#define BE_MAX_LOST_MS          900   // 连接处最长容忍时间；大转弯幅度的弯道视觉丢失时间明显更长
#define BE_RECOVER_POINTS       15    // 两侧点数都超过该值才认为重新看到赛道

/* ===== 转角保持 =====
 * 中线点数少到无法解算时，用最近一次有效转角维持转向，按帧衰减。 */
#define BE_MIN_POINTS           8     // 中线点数低于该值时启用保持
#define BE_TURN_DECAY           1     // 每帧向 0 收敛的单位数（角度索引）；大弯需要保持更久

/* ===== 对外接口 ===== */

/* 上电与每次发车时清零。state 回到 STRAIGHT，保持角清零。 */
void bendEntryInit(void);

/* 每帧在 loadRoad() 之后调用一次，传入本帧中线与前瞻点。
 * 内部自行计算弯道强度与转角，不依赖 Motor.c 的静态量。 */
void bendEntryUpdate(Point centerLine[], int16 centerLineCount, int16 aimPos);

/* 出界宽容等级，供 main.c 的出界判定分支使用：
 *   0 = 正常判据（5 点全暗，或两侧无边界且至少 3 点暗）
 *   1 = 放宽（只有 5 点全暗才判）
 *   2 = 免判（连接处的时间窗内一律不判） */
uint8 bendEntryGuard(void);

/* 转角保持。返回 1 表示本帧应使用 *turn 而不是纯跟踪结果；
 * 返回 0 表示按正常链路走。 */
uint8 bendEntryTurn(int8 *turn);

/* 当前状态，0 STRAIGHT / 1 APPROACH / 2 COMMITTED / 3 RECOVER，供串口观察 */
uint8 bendEntryState(void);

#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_BENDENTRY_H
