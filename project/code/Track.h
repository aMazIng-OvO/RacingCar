#ifndef SEEKFREE_TRACK_H
#define SEEKFREE_TRACK_H

#include "zf_common_headfile.h"

/* ===== 赛道状态判定 =====
 * 判据来自参考实现的出界条件 !LRoadCount && !RRoadCount，
 * 取补集得到"任一侧边界点数非 0 即在赛道上"。
 * 输入就是 loadRoad() 的边界点数，不额外读像素。 */
void trackInit(void);            // 上电初始化
uint8 trackEvaluate(void);       // 每帧在 loadRoad() 之后调用，更新原始判据
void trackMonitor(void);         // 主循环每轮调用：对判据消抖并驱动蓝灯
uint8 trackIsOn(void);           // 1 = 当前判定在赛道上
void trackLedHold(uint16 ms);    // 按键确认反馈：蓝灯保持点亮 ms 毫秒（非阻塞）

#endif //SEEKFREE_TRACK_H
