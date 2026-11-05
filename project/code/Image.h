//
// Created by 31663 on 2026/3/16.
//

#ifndef SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
#define SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
#include "zf_common_headfile.h"

#define block 5               //局域二值化边长
#define cap 4                 //去噪余量（黑白对比用6，蓝白对比低改用4，噪点多再调大）
#define expRoadCount 100      //边线期望长度

/* ---------------- 蓝底白赛道现场标定参数 ---------------- */
#define EDGE_DIFF 12          //边界梯度最小差（原15，蓝白对比低改小）
#define EDGE_RATIO 7          //亮度和/差值比上限，越小越严格
#define HIDE_ROW 90           //车身遮挡区起始行（该行以下中心区域强制为白）
#define HIDE_COL_L 48         //遮挡区左列
#define HIDE_COL_R 140        //遮挡区右列
#define CORNER_COS100 50      //拐点判定：cos阈值×100，越小越容易出拐点

void getLBorder(Point LRoad[], Point start, int16 *count, int8 delicate);
void getRBorder(Point RRoad[], Point start, int16 *count, int8 delicate);
void getCenterLineByLBorder(Point roadLine[], Point centerLine[], int16 count);
void getCenterLineByRBorder(Point roadLine[], Point centerLine[], int16 count);
void filterRoad(Point rodeLine[], int16 count);
void getGodRoadLine(Point roadLine[], int16 count);
void resampleRoad(Point roadLine[], int16 *count, int16 startPos, uint8 dis);
Point getLStart();
Point getRStart();
int16 findCorner(Point roadLine[], int16 count, int8 cross);
uint8 getImagePointFilterSelf(uint8 i, uint8 j);
uint8 imageInitThreshold(void);
extern uint8 threshold, LThreshold, RThreshold, centerLineDistance, banUpdateThreshold;
extern int16 LBorderCount, RBorderCount;
extern Point LBorder[expRoadCount], RBorder[expRoadCount];

/* 双缓冲：mt9v03x_image 由 DMA 直接写入（采集缓冲），mt9v03x_frame 是算法统一读取的稳定快照。
 * DMA 完成中断内搬运一帧并置 frameBusy；主循环消费完清 0，消费不过来则丢弃新帧。 */
extern uint8 mt9v03x_frame[MT9V03X_H][MT9V03X_W];
extern volatile uint8 frameBusy;
void mt9v03x_frame_snapshot(void);
#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
