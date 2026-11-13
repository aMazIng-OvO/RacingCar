//
// 赛道识别 + 蓝灯指示
//
// 判据：loadRoad() 每帧更新 LRoadCount / RRoadCount，任一侧边界点数超过阈值
//       且画面有明暗差（threshold != 128），即认为"车在赛道上"（带 50 ms 消抖）。
//
// 蓝灯：
//   发车被拒（不在赛道） → 慢闪 1 Hz，把车放回赛道后自动停闪
//   在赛道上             → 常规指示（故障常亮 / 正向快闪 / 反向常亮 / 静止熄灭）
//

#ifndef HFK_STC32G144K_TRACK_H
#define HFK_STC32G144K_TRACK_H

#include "zf_common_headfile.h"

void trackInit(void);            // 上电初始化（静态变量默认为 0，可省略）
void trackMonitor(void);         // 主循环 1 ms 调用一次：更新判据并驱动蓝灯
uint8 trackIsOn(void);           // 1 = 当前判定在赛道上
void trackSetNoTrackAlarm(uint8 on); // 1 = 打开"不在赛道"慢闪；0 = 关闭

#endif //HFK_STC32G144K_TRACK_H
