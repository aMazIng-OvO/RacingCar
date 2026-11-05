//
// Created by 31663 on 2026/3/29.
//

#include "Cross.h"

#define CROSS_READY 1
#define CROSS_IN 2
extern uint8 switch2;
extern int16 centerLineCount, RUN_MODE;
extern Point centerLine[expRoadCount];

static int8 CROSS_MODE = CROSS_READY, canEnd = 0;
static int16 alongR, crossThreshold, minMaxH;

Point getCrossRStart() {
    Point pos = {80, 94};
    int16 min = 187, i;
    uint8 found = 0;
    if (RBorderCount >= 40 || !minMaxH) {
        for (i = 0; i < RBorderCount; i += 2) {
            if (RBorder[i].y <= min) min = RBorder[i].y;
            else {
                // 防止 uint8 下溢成 25x
                pos.x = RBorder[i].x > 4 ? RBorder[i].x - 4 : 0;
                pos.y = minMaxH = RBorder[i].y;
                found = 1;
                break;
            }
        }
        // 直道上 y 单调不增时循环一次都不会 break，必须给确定起点
        if (!found) {
            pos.x = 80;
            pos.y = (uint8) minMaxH;
        }
    } else {
        pos.x = 80;
        pos.y = (uint8) minMaxH;
    }
    while (pos.x >= 8 && (getImagePointFilterSelf(pos.x, pos.y) > crossThreshold || getImagePointFilterSelf(pos.x - 8, pos.y) > crossThreshold)) pos.x--;
    if (pos.x <= 7)
        pos.x = pos.y = 0;
    else
        pos.x++;
    return pos;
}

Point getCrossLStart() {
    Point pos = {80, 94};
    int16 max = 0, i;
    uint8 found = 0;
    if (LBorderCount >= 40 || !minMaxH) {
        for (i = 0; i < LBorderCount; i += 2) {
            if (LBorder[i].y >= max) max = LBorder[i].y;
            else {
                pos.x = LBorder[i].x > 4 ? LBorder[i].x - 4 : 0;
                pos.y = minMaxH = LBorder[i].y;
                found = 1;
                break;
            }
        }
        if (!found) {
            pos.x = 80;
            pos.y = (uint8) minMaxH;
        }
    } else {
        pos.x = 80;
        pos.y = (uint8) minMaxH;
    }
    while (pos.x >= 8 && (getImagePointFilterSelf(pos.x, pos.y) > crossThreshold || getImagePointFilterSelf(pos.x - 8, pos.y) > crossThreshold)) pos.x--;
    if (pos.x <= 7)
        pos.x = pos.y = 0;
    else
        pos.x++;
    return pos;
}

void runCross() {
    Point start;
#if (DEBUG_MODE)
    if (switch2) {
        ips200_show_int16(0, MT9V03X_H + 1, LBorderCount);
        ips200_show_int16(MT9V03X_W, MT9V03X_H + 1, RBorderCount);
        ips200_show_int16(MT9V03X_W / 2, MT9V03X_H + 1, centerLineCount);
    }
#endif
    if (CROSS_MODE == CROSS_READY) {
        //gpio_set_level(IO_P90, 1);
        if (centerLineCount && centerLineCount < 13) {
            crossThreshold = (threshold + LThreshold) >> 1;
            alongR = LBorderCount < RBorderCount;
            CROSS_MODE = CROSS_IN;
        }
        alongLine(centerLine, centerLineCount, getAimPos());
    } else if (CROSS_MODE == CROSS_IN) {
        Point border[expRoadCount], farCenterLine[expRoadCount];
        int16 roadCount = expRoadCount, farCenterLineCount, corner;
        if (!LBorderCount && !RBorderCount)
            canEnd = 1;
        banUpdateThreshold = 1;
        if (alongR) {
            start = getCrossRStart();
            getRBorder(border, start, &roadCount, 0);
        } else {
            start = getCrossLStart();
            getLBorder(border, start, &roadCount, 0);
        }
        getGodRoadLine(border, roadCount);
        resampleRoad(border, &roadCount, 0, 3);
        corner = findCorner(border, roadCount, 1);
        filterRoad(border, roadCount);
        farCenterLineCount = roadCount;
        if (farCenterLineCount > expRoadCount) farCenterLineCount = expRoadCount;
        if (farCenterLineCount < 2) {
            // 远边界太短：本帧保持上一拍控制，不写数组也不做补线
            alongLine(centerLine, centerLineCount, getAimPos());
            return;
        }
        if (alongR)
            getCenterLineByRBorder(border, farCenterLine, farCenterLineCount);
        else getCenterLineByLBorder(border, farCenterLine, farCenterLineCount);
        if (corner < 0 || corner >= farCenterLineCount) corner = 0;
        // 原来 func_min(corner+3, count) 会返回 count=100，写 farCenterLine[100] 越界
        if (corner && farCenterLine[corner].x > 50)
            corner = func_min(corner + 3, farCenterLineCount - 1);
        else
            corner = func_min(5, farCenterLineCount - 1);
        farCenterLine[corner].x = MT9V03X_H - 1;
        farCenterLine[corner].y = MT9V03X_W / 2 - 3;
        resampleRoad(farCenterLine, &farCenterLineCount, corner, 2);
        filterRoad(farCenterLine, farCenterLineCount);
        alongLine(farCenterLine, farCenterLineCount, getAimPos());
        if (centerLineCount > 20 && canEnd) {
            canEnd = minMaxH = 0;
            banUpdateThreshold = 0;   // 退出十字才解除阈值冻结
            CROSS_MODE = CROSS_READY;
            RUN_MODE = FORWARD;
            gpio_set_level(IO_P90, 0);
        }
    }
}

void crossReset(void) {
    CROSS_MODE = CROSS_READY;
    canEnd = 0;
    minMaxH = 0;
    alongR = 0;
    crossThreshold = 0;
    banUpdateThreshold = 0;
}