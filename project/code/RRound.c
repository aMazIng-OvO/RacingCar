//
// Created by 31663 on 2026/3/29.
//

#include "RRound.h"

#define ROUND_READY 1
#define ROUND_START 2
#define ROUND_IN 3
#define ROUND_END 4
#define ROUND_EXIT 5

static uint8 ROUND_MODE = ROUND_READY, canStart = 0, canIn = 0, canFinish = 0;
static int8 send_buff[32];
extern int16 centerLineCount, RUN_MODE, RRoadCount, LRoadCount, currentDuty;
extern uint8 ALONG, borderIn;
extern Point centerLine[expRoadCount], LRoad[expRoadCount];

void runRRound(int16 LCorner, int16 RCorner) {
    if (ROUND_MODE == ROUND_READY) {
        borderIn = 1;
        gpio_set_level(IO_P90, 1);
        alongLine(centerLine, centerLineCount, getAimPos());
        if (!RRoadCount)
            canStart = 1;
        else if (RRoadCount > 10 && !RCorner && canStart) {
            ROUND_MODE = ROUND_START;
            ALONG = ALONG_R;
            centerLineDistance = 11;
        }
    } else if (ROUND_MODE == ROUND_START) {
        gpio_set_level(IO_P90, 0);
        ALONG = ALONG_R;
        alongLine(centerLine, centerLineCount, getAimPos());
        if (!LRoadCount)
            canIn = 1;
        else if (LRoadCount > 20 && !LCorner && canIn) {
            centerLineDistance = 25;
            ROUND_MODE = ROUND_IN;
            ALONG = ALONG_L;
        }
    } else if (ROUND_MODE == ROUND_IN) {
        ALONG = ALONG_L;
        alongLine(centerLine, centerLineCount, getAimPos());
        if (LCorner > 3 && centerLineCount < 11 && centerLineCount) {
            centerLineDistance = 11;
            ROUND_MODE = ROUND_END;
            ALONG = ALONG_R;
        }
    } else if (ROUND_MODE == ROUND_END) {
        ALONG = ALONG_R;
        alongLine(centerLine, centerLineCount, 17);
        if (!LCorner && LRoadCount > 25 && LRoad[LRoadCount - 1].y > 55) {
            centerLineDistance = 18;
            ROUND_MODE = ROUND_EXIT;
            ALONG = ALONG_L;
        }
    } else if (ROUND_MODE == ROUND_EXIT) {
        ALONG = ALONG_L;
        alongLine(centerLine, centerLineCount, getAimPos());
        if (!RRoadCount)
            canFinish = 1;
        else if (RRoadCount > 10 && canFinish) {
            ROUND_MODE = ROUND_READY;
            handle = 0;
            RUN_MODE = FORWARD;
            canFinish = canStart = canIn = 0;
        }
    }
    // sprintf(send_buff, "duty: %d,%d\n", currentDuty, ROUND_MODE);
    // wireless_uart_send_buffer(send_buff, 32);
}