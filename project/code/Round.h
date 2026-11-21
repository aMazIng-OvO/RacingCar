//
// 第二部分：环岛（左右共用一套判定，靠 side 区分）
//
// 为什么单独做一块：环岛的入口在画面里是"外侧一条长边界一路伸到远端 +
// 内侧边界整条消失"。此时视觉中线在合流区指向"直穿"，纯跟踪算出的角度
// 贴不住环内（实拍视频就是这样直穿过去的）。环是定曲率，所以进入后按阶段
// 叠加一个固定的转向偏置，比指望视觉在宽白区解出曲率可靠得多。
//
// 输入：两侧边界的原始点数、裁短后点数、最远点行号、拐点序号
// 识别：外侧点数够长 + 外侧末点伸到远端 + 内侧点数够短，积分确认
// 动作：50% 限速 + 五阶段固定转向偏置 + 缺口锁存推进
//
// 与另外两部分的边界：本模块要求"一侧长、另一侧短"，
// S 弯要求"两侧都在"，十字要求"两侧都短"，三者互斥。
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_ROUND_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_ROUND_H
#include "zf_common_headfile.h"

#define RUN_ROUND_R     1919    // 右环岛（沿用原有 MODE 值，串口图例不变）
#define RUN_ROUND_L      810    // 左环岛

#define RD_SIDE_R          0    // 内侧在右
#define RD_SIDE_L          1    // 内侧在左

uint8 rdDetect(void);            // 纯谓词：本帧是否满足某一侧的入口条件
void  rdEnter(void);             // 确定方向并进入 READY
void  rdRun(void);               // 本帧执行
uint8 rdBusy(void);              // 1 = 还要继续占用
void  rdExit(void);              // 收尾：清偏置与限速
void  rdReset(void);             // 发车/急停强制复位
uint8 rdStage(void);             // 1~5 = READY/START/IN/END/EXIT，0 = 空闲
uint8 rdSide(void);              // 0 = 右环岛，1 = 左环岛
/* 只读诊断（串口 CAND）：右环岛看 bit0/1/2/3，左环岛看 bit4/5/6/7，
 * 0x0F / 0xF0 才是完整候选；bit3/7 为 0 表示外侧太弯，更像普通急弯 */
uint8 rdCandBits(void);
uint8 rdScore(void);             // 只读：两侧积分确认的较大值（串口 RS）

#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_ROUND_H
