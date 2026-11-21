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
/* 摄像头安装（2026-11 实测）：
 *   高度 24 cm；光轴与立杆夹角 63° → 相对水平俯角 27°；
 *   镜头距车头 15 cm，整车长 29 cm；FOV 120°，机内广角校正。
 * 机内校正已经输出近似针孔图像，直接使用校正后的原始像素坐标。
 * 按 188×120、120° 对角线 FOV 估算：f≈64.4 px，地平线约在行 27；
 * 行 99 约对应镜头前 14.7 cm（车头前缘），行 34 约对应 2.5 m。
 * 若 120° 指水平 FOV，地平线约行 32、行 34 约对应 7.5 m，窗口仍成立。 */

#define EDGE_DIFF 12          //边界梯度最小差（原15，蓝白对比低改小）
#define EDGE_RATIO 7          //亮度和/差值比上限，越小越严格
#define HIDE_COL_L 48         //车身遮挡区左列
#define HIDE_COL_R 140        //车身遮挡区右列
/* 车身在画面里出现的最上行。该行及以下的起点搜索只用两侧窄列
 * （左 3~HIDE_COL_L、右 HIDE_COL_R~186），绕开中央的车身暗块。 */
#define BODY_ROW 80
#define CORNER_COS100 50      //拐点判定：cos阈值×100，越小越容易出拐点

/* ===== 视野窗口：只保留中间一块，四周各裁掉一圈 =====
 * 摄像头 FOV 120° 且机内校正，视场明显大于赛道。远端是天空/场外底布，
 * 近端是车身与车头阴影，左右边缘还是场地。这些区域对循迹没有信息量，
 * 却会让爬线拟合出不存在的"边界"、让出界采样读到赛道外的暗区。
 * 24 cm 高、俯角 27° 时地平线约在行 27（按 120° 对角线 FOV）；
 * 旧窗口从行 10 开始，会把地平线以上的天空/背景一起喂给阈值和爬线。
 * 现取上 34 行、下 20 行、左右各 34 列 → 行 34~99（66 行）、列 34~153（120 列）。
 * 行 34 对应地面约 2.5 m（对角线 FOV）~7.5 m（水平 FOV），前瞻仍足够。
 * 行 99 对应镜头前约 14.7 cm，正好在车头前缘；再往下就是车身与地面阴影。
 * 调整原则：窗口必须仍包住赛道可能出现的全部位置，正常直道上两侧边线
 * 至少离窗口边缘 10 像素。留窄了会在弯道/环岛里把边线一起裁掉（点数骤降）。 */
#define VIEW_CUT_FAR    34    //远端（上）裁切：去掉地平线以上的天空/场外背景
#define VIEW_CUT_NEAR   20    //近端（下）裁掉的行数：车身与车头阴影
#define VIEW_CUT_LEFT   34    //左侧裁掉的列数
#define VIEW_CUT_RIGHT  34    //右侧裁掉的列数

#define VIEW_FAR_ROW   VIEW_CUT_FAR                        // = 34，保留起始行（含）
#define VIEW_NEAR_ROW  (MT9V03X_H - 1 - VIEW_CUT_NEAR)    // = 99，保留结束行（含）
#define VIEW_COL_L     VIEW_CUT_LEFT                      // = 34，保留起始列（含）
#define VIEW_COL_R     (MT9V03X_W - 1 - VIEW_CUT_RIGHT)   // = 153，保留结束列（含）

void getLBorder(Point LRoad[], Point start, int16 *count);
void getRBorder(Point RRoad[], Point start, int16 *count);
void filterRoad(Point rodeLine[], int16 count);
void resampleRoad(Point roadLine[], int16 *count, int16 startPos, uint8 dis);
Point getLStart();
Point getRStart();
int16 findCorner(Point roadLine[], int16 count, int8 cross);
uint8 imageInitThreshold(void);

/* 把快照的视野窗口以外区域（远端行、近端行、左右列）写成暗区，
 * 屏蔽赛道外的反光、背景与车身。必须在 getLStart() / getLBorder() 之前调用。 */
void cutViewRegion(void);
extern uint8 threshold, LThreshold, RThreshold, centerLineDistance;
extern int16 LBorderCount, RBorderCount;
extern Point LBorder[expRoadCount], RBorder[expRoadCount];

/* 双缓冲：mt9v03x_image 由 DMA 直接写入（采集缓冲），mt9v03x_frame 是算法统一读取的稳定快照。
 * DMA 完成中断内搬运一帧并置 frameBusy；主循环消费完清 0，消费不过来则丢弃新帧。 */
extern uint8 mt9v03x_frame[MT9V03X_H][MT9V03X_W];
extern volatile uint8 frameBusy;
void mt9v03x_frame_snapshot(void);
#endif //SEEKFREE_STC32G144K_100PIN_OPENSOURCE_LIBRARY_IMAGE_H
