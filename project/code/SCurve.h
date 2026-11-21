//
// 第一部分：连续 S 弯
//
// 为什么单独做一块：连续 S 弯的问题不是"打角不够"，而是"看太远"。
// 纯跟踪的前瞻点会落到下一个反向弯里，车头刚摆过去就立刻被拉回来，
// 表现为直道上左右小摆、S 弯里一路蹭着两侧边线过去。
//
// 输入：中线点列（centerLine / centerLineCount，重采样间距 2 像素）
// 识别：长窗（12 点 = 24 像素）横向增量的显著符号翻转 ≥ 1 次，且全程 dy 摆幅 ≥ 8 像素
//       —— 短基线测不到真实曲率（亚像素），只有长窗才能把弯道信号从噪声里分出来
// 动作：整个 S 弯内按住速度上限（只压速度，不动前瞻也不动打角）
//
// 与另外两部分的边界：本模块要求"两侧边界都在、中线足够长"，
// 一旦任一侧丢线（环岛/十字的特征），它自然判不成立，不会抢别人的触发。
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_SCURVE_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_SCURVE_H
#include "zf_common_headfile.h"

/* RUN_MODE / 串口 MODE 字段的取值，和 FORWARD(114) / RUN_X_CROSS(514) 同一量级不冲突 */
#define RUN_S_CURVE     314

uint8 scDetect(void);   // 纯谓词：本帧是否满足 S 弯条件（不改任何状态）
void  scEnter(void);    // 记下进入时刻
void  scRun(void);      // 本帧执行：收前瞻 + 压速度 + 正常循迹
uint8 scBusy(void);     // 1 = 还要继续占用；0 = 可以退出
void  scExit(void);     // 收尾：清掉前瞻裁剪与速度比例
void  scReset(void);    // 发车/急停强制复位
uint8 scStage(void);    // 0 = 空闲，1 = 正在处理

#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_SCURVE_H
