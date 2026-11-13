//
// 赛道识别 + 蓝灯指示
//
// 判据：loadRoad() 每帧更新 LRoadCount / RRoadCount，任一侧边界点数超过阈值
//       即认为"车在赛道上"（带 50 ms 消抖，避免单帧抖动来回切换）。
//
// 蓝灯：
//   在赛道上 → 常规指示（故障常亮 / 正向快闪 / 反向常亮 / 静止熄灭）
//   不在赛道 → 快闪 3 下 → 停 1 秒 → 再快闪 3 下 …… 循环
//

#ifndef HFK_STC32G144K_TRACK_H
#define HFK_STC32G144K_TRACK_H

#include "zf_common_headfile.h"

void trackInit(void);      // 上电初始化（可省略，静态变量默认已清零）
void trackMonitor(void);   // 主循环 1 ms 调用一次：更新判据并驱动蓝灯
uint8 trackIsOn(void);     // 查询当前是否判定为"在赛道上"（1 = 在）

#endif //HFK_STC32G144K_TRACK_H
