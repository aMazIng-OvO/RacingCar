//
// Created by 31663 on 2026/3/29.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_LROUND_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_LROUND_H

#include "zf_common_headfile.h"

void runLRound(int16 LCorner, int16 RCorner);
void lroundReset(void);  // 发车/急停时复位左环岛状态机

#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_LROUND_H
