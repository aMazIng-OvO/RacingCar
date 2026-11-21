//
// Created by 31663 on 2026/3/16.
//

#include "Image.h"

const static int8 data dirF[4][2] = {{-2, 0},{0, 2},{2, 0},{0, -2}},
    dirL[4][2] = {{-2, -2},{-2, 2},{2,  2},{2,  -2}},
    dirR[4][2] = {{-2, 2},{2,  2},{2,  -2},{-2, -2}};
uint8 LThreshold, RThreshold, threshold, centerLineDistance = 18;
const uint8 static data halfBlock = block / 2, data halfH = MT9V03X_H / 2, data halfW = MT9V03X_W / 2;
Point LBorder[expRoadCount], RBorder[expRoadCount];
int16 LBorderCount, RBorderCount;

/* 稳定优先：帧原子性由 DMA 完成中断保证，不允许算法读到半帧。
 * mt9v03x_image 是 DMA 采集缓冲，mt9v03x_frame 是算法唯一读取的快照。
 * 每帧 DMA 完成后在中断内整帧搬运；frameBusy 保证主循环处理期间快照不被覆盖，
 * 来不及消费时直接丢弃新帧（可预期的降级，而不是概率性混帧）。
 * TIM1 控制中断优先级 3、LCM_DMA 优先级 1（main.c 设置），控制中断可抢占本次搬运。 */
uint8 mt9v03x_frame[MT9V03X_H][MT9V03X_W];
volatile uint8 frameBusy = 0;
uint16 dropFrameCnt = 0;   // 主循环没消费完就被丢弃的帧数（串口 DROP 字段），帧率诊断用

void mt9v03x_frame_snapshot(void) {
    uint16 data i;
    uint16 *data dst;
    uint16 *data src;
    // 主循环还没消费完上一帧时跳过，防止快照被中途覆盖
    if (!frameBusy) {
        /* 22.5KB 整帧搬运是每帧最大的开销，而它跑在 DMA 完成中断里，直接决定实际帧率。
         * 逐字节 memcpy 在 96MHz 下约 4~5ms，逼近 5.88ms 的帧间隔，只能隔帧处理（实测约 96fps）；
         * 改成 16 位字搬运后循环次数减半，搬运结果与 memcpy 逐字节完全一致。 */
        dst = (uint16 *) &mt9v03x_frame[0][0];
        src = (uint16 *) &mt9v03x_image[0][0];
        /* 每次搬 4 个字（8 字节），把循环开销再摊薄 4 倍；
         * 22560 能被 8 整除，所以不会漏搬 */
        for (i = 0; i < (MT9V03X_IMAGE_SIZE >> 3); i++) {
            dst[0] = src[0];
            dst[1] = src[1];
            dst[2] = src[2];
            dst[3] = src[3];
            dst += 4;
            src += 4;
        }
        frameBusy = 1;
    } else {
        dropFrameCnt++;   // 处理慢于帧率时的可预期降级：计数并在串口暴露，而不是无声丢帧
    }
}

/* 视野窗口：把窗口以外（远端行、近端行、左右列）整块写成暗区，参数见 Image.h。
 * 透视后的远端每行只对应几毫米，反光、背景、赛道外的深色区域都在这里；
 * 近端是车身与车头阴影；左右边缘同样是场地。写成暗区后：
 *   爬线不会伸进去，起点搜索与爬线判据也同步用窗口收紧边界。 */
void cutViewRegion(void) {
    uint8 i, j;

    for (i = 0; i < MT9V03X_H; i++) {
        for (j = 0; j < VIEW_COL_L; j++)
            mt9v03x_frame[i][j] = 0;
        for (j = VIEW_COL_R + 1; j < MT9V03X_W; j++)
            mt9v03x_frame[i][j] = 0;
        if (i < VIEW_FAR_ROW || i > VIEW_NEAR_ROW)
            for (j = 0; j < MT9V03X_W; j++)
                mt9v03x_frame[i][j] = 0;
    }
}

uint8 getPointThreshold(uint8 x, uint8 y) {
    uint16 total = 0;
    uint8 i, j;
    int16 avg;
    //采样窗口必须完整落在图像内，否则 x - halfBlock 会按 uint8 回绕成 254 之类的越界下标
    if (x < halfBlock) x = halfBlock;
    if (x > MT9V03X_H - 1 - halfBlock) x = (uint8) (MT9V03X_H - 1 - halfBlock);
    if (y < halfBlock) y = halfBlock;
    if (y > MT9V03X_W - 1 - halfBlock) y = (uint8) (MT9V03X_W - 1 - halfBlock);
    for (i = x - halfBlock; i <= x + halfBlock; i++)
        for (j = y - halfBlock; j <= y + halfBlock; j++)
            total += mt9v03x_frame[i][j];
    avg = (int16) (total / (block * block)) - cap;
    if (avg < 0) avg = 0;
    return (uint8) avg;
}

//用视野窗口内的灰度最小/最大中点估计初始阈值，不依赖固定采样点
uint8 imageInitThreshold(void) {
    uint8 i, j, v, mn = 255, mx = 0;
    /* 取样范围必须落在窗口内并离边缘 4 像素：窗口外已被写成暗区，
     * 取进去只会把最小/最大值压到 0，对比度判据直接失效。
     * 原来固定取行 92~113，近端裁到行 99 以后会读到大片暗区。 */
    for (i = VIEW_FAR_ROW + 4; i <= VIEW_NEAR_ROW - 4; i += 2)
        for (j = VIEW_COL_L + 4; j <= VIEW_COL_R - 4; j += 2) {
            v = mt9v03x_frame[i][j];
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
    if (mx - mn < 20) return 128; //对比度异常，退回中间值
    return mn + ((mx - mn) >> 1);
}

static int8 check(uint8 x, uint8 y) {
    uint16 d = func_abs((int16) mt9v03x_frame[x][y - 2] - (int16) mt9v03x_frame[x][y + 2]), a =
            (uint16) mt9v03x_frame[x][y - 2] + (uint16) mt9v03x_frame[x][y + 2];
    if (d > EDGE_DIFF && a / d <= EDGE_RATIO) return 1;
    return 0;
}

static int16 getLIndexStart(int16 i) {
    int16 l, r, m;
    /* 左界从裁切框内侧起搜：框外的列已被写成暗区，参与二分只会把起点拉到框外 */
    l = VIEW_COL_L + halfBlock;
    r = i < BODY_ROW ? halfW : HIDE_COL_L;
    while (l < r) {
        m = (l + r) >> 1;
        if (mt9v03x_frame[i][m] < LThreshold && mt9v03x_frame[i][func_max(0, m - 8)] < LThreshold && mt9v03x_frame[i][func_max(0, m - 25)] < LThreshold) l = m + 1;
        else r = m;
    }
    return l;
}

Point getLStart() {
    int16 i, l;
    Point ans;
    /* 扫描范围就是视野窗口的行区间：从窗口下沿往上找第一处灰度跳变。
     * 窗口外整块是暗区，扫进去只会白费二分次数。 */
    for (i = VIEW_NEAR_ROW - halfBlock - 2; i >= VIEW_FAR_ROW + 3; i -= 3) {
        l = getLIndexStart(i);
        if (mt9v03x_frame[i][func_min(l + 2, MT9V03X_W - 1)] > LThreshold && check(i, l)) {
            ans.x = i;
            ans.y = l + 1;
            return ans;
        }
    }
    ans.x = 0;
    ans.y = 0;
    return ans;
}

static int16 getRIndexStart(int16 i) {
    int16 l, r, m;
    l = i < BODY_ROW ? halfW : HIDE_COL_R;
    /* 右界收到裁切框内侧，理由同 getLIndexStart */
    r = VIEW_COL_R - halfBlock;
    while (l < r) {
        m = (l + r + 1) >> 1;
        if (mt9v03x_frame[i][m] < RThreshold && mt9v03x_frame[i][func_min(MT9V03X_W - 1, m + 8)] < RThreshold && mt9v03x_frame[i][func_min(MT9V03X_W - 1, m + 25)] < RThreshold) r = m - 1;
        else l = m;
    }
    return r;
}

Point getRStart() {
    int16 i, r;
    Point ans;
    /* 与 getLStart 一致：扫描范围就是视野窗口的行区间 */
    for (i = VIEW_NEAR_ROW - halfBlock - 2; i >= VIEW_FAR_ROW + 3; i -= 3) {
        r = getRIndexStart(i);
        if (mt9v03x_frame[i][func_max(r - 2, 0)] > RThreshold && check(i, r)) {
            ans.x = i;
            ans.y = r - 1;
            return ans;
        }
    }
    ans.x = 0;
    ans.y = 0;
    return ans;
}

void getLBorder(Point LRoad[], Point start, int16 *count) {
    uint8 data
    x = start.x, data
    y = start.y, data
    turn = 0, data
    dir = 0, data
    up = 0, data
    localThreshold;
    int16 data
    step = 0, tempThreshold, limit;
    //起点无效（(0,0)）时必须先返回，不能再拿它去取局部阈值
    if (start.x == 0 && start.y == 0) {
        *count = 0;
        return;
    }
    // 写入点数硬夹在数组容量内：调用方误传负 count 时不会越界写 LRoad[]
    limit = func_limit_ab(*count, 0, expRoadCount);
    tempThreshold = localThreshold = getPointThreshold(x, y);
    /* 爬线活动范围限制在视野窗口内：窗口外是写成暗区的区域，
     * 走进去只会被当成"前方变暗"而反复转向，白白消耗 turn 配额 */
    /* turn 上限 8：原来 4 次就放弃，硬弯内侧边线是斜的、常需要连续几次转向，
     * 提前截断会让点数骤降，与"真实缺口"混在一起，是元素误判的主要噪声源。 */
    while (x > VIEW_FAR_ROW + 1 && x < VIEW_NEAR_ROW - 1 && y > VIEW_COL_L + 1 && y < VIEW_COL_R - 1 && step < limit &&
           turn < 8) {
        up++;
        if (up == 2) {
            localThreshold = getPointThreshold(x, y);
            up = 0;
        }
        if (mt9v03x_frame[dirF[dir][0] + x][dirF[dir][1] + y] < localThreshold) {
            dir = (dir + 1) % 4;
            turn++;
        } else if (mt9v03x_frame[dirL[dir][0] + x][dirL[dir][1] + y] < localThreshold) {
            x += dirF[dir][0];
            y += dirF[dir][1];
            LRoad[step].x = x;
            LRoad[step].y = y;
            step++;
            turn = 0;
        } else {
            x += dirL[dir][0];
            y += dirL[dir][1];
            LRoad[step].x = x;
            LRoad[step].y = y;
            step++;
            turn = 0;
            dir = (dir + 3) % 4;
        }
        if (x == start.x && y == start.y && !turn) break;
    }
    *count = step;
    /* 阈值更新不允许被"爬线成功"单方面锁死：光照突变 → 画面整体偏暗 → 爬线失败
     * （step 很小）→ 阈值不再更新 → 下一帧更失败——正反馈死锁，停死后按键也无法恢复。
     * 爬线失败时用全帧 min/max 中点做一次 1/4 权重的弱拉回，让阈值缓慢跟随环境；
     * 成功路径（step > 25）仍走原来的算术平均。 */
    if (step > 25) {
        /* 阈值平滑：取「本次局部阈值」与「上一帧阈值」的算术平均。
         * 先把两个操作数提升到 int16 再相加，避免 int8 中间结果溢出。
         * 括号只是把 C 的优先级（+ 高于 >>）写明白，语义与参考实现一致。 */
        LThreshold = (uint8) (((int16) tempThreshold + (int16) LThreshold) >> 1);
        threshold = (uint8) (((int16) LThreshold + (int16) RThreshold) >> 1);
    } else if (step < 5) {
        /* 只在"几乎一个点都没爬出来"时才兜底：这才是阈值死锁的症状。
         * 原来条件写成 step <= 25，但急弯 / 环岛 / 十字里单侧可见点数本来就只有
         * 十几到二十几个，会每帧都命中，把局部自适应阈值一路拉向全帧中点。 */
        uint8 re = imageInitThreshold();
        LThreshold = (uint8) (((int16) re + 3 * (int16) LThreshold) >> 2);
        RThreshold = (uint8) (((int16) re + 3 * (int16) RThreshold) >> 2);
        threshold = (uint8) (((int16) LThreshold + (int16) RThreshold) >> 1);
    }
}

void getRBorder(Point RRoad[], Point start, int16 *count) {
    uint8 data
    x = start.x, data
    y = start.y, data
    turn = 0, data
    dir = 0, data
    up = 0, data
    localThreshold;
    int16 data
    step = 0, tempThreshold, limit;
    //起点无效（(0,0)）时必须先返回，不能再拿它去取局部阈值
    if (start.x == 0 && start.y == 0) {
        *count = 0;
        return;
    }
    limit = func_limit_ab(*count, 0, expRoadCount);
    tempThreshold = localThreshold = getPointThreshold(x, y);
    /* 与 getLBorder 对称：活动范围限制在窗口内，turn 上限同步放宽到 8 */
    while (x > VIEW_FAR_ROW + 1 && x < VIEW_NEAR_ROW - 1 && y > VIEW_COL_L + 1 && y < VIEW_COL_R - 1 && step < limit &&
           turn < 8) {
        up++;
        if (up == 2) {
            localThreshold = getPointThreshold(x, y);
            up = 0;
        }
        if (mt9v03x_frame[dirF[dir][0] + x][dirF[dir][1] + y] < localThreshold) {
            dir = (dir + 3) % 4;
            turn++;
        } else if (mt9v03x_frame[dirR[dir][0] + x][dirR[dir][1] + y] < localThreshold) {
            x += dirF[dir][0];
            y += dirF[dir][1];
            RRoad[step].x = x;
            RRoad[step].y = y;
            step++;
            turn = 0;
        } else {
            x += dirR[dir][0];
            y += dirR[dir][1];
            RRoad[step].x = x;
            RRoad[step].y = y;
            step++;
            turn = 0;
            dir = (dir + 1) % 4;
        }
        if (x == start.x && y == start.y && !turn) break;
    }
    *count = step;
    /* 与 getLBorder 一致：成功走算术平均；失败时不再重复做全帧重估——
     * 同帧内 getLBorder 已经按 1/4 权重拉过一次，这里只把合成阈值刷新一下，
     * 避免失败帧每侧各扫 2296 点（2 次/帧 → 1 次/帧）。 */
    if (step > 25) {
        /* 阈值平滑：与 getLBorder 一致，先提升到 int16 再取算术平均 */
        RThreshold = (uint8) (((int16) tempThreshold + (int16) RThreshold) >> 1);
    }
    threshold = (uint8) (((int16) LThreshold + (int16) RThreshold) >> 1);
}

void filterRoad(Point rodeLine[], int16 count) {
    int16 data
    i, data sx, data sy;
    for (i = 1; i < count && i < expRoadCount; i++) {
        // 先加后移，避免 (a>>1)+(b>>1) 在两奇数时系统性少 1 个像素
        sx = (int16) rodeLine[i - 1].x + (int16) rodeLine[i].x + 1;
        sy = (int16) rodeLine[i - 1].y + (int16) rodeLine[i].y + 1;
        rodeLine[i].x = func_limit_ab(sx >> 1, 0, MT9V03X_H - 1);
        rodeLine[i].y = func_limit_ab(sy >> 1, 0, MT9V03X_W - 1);
    }
}

void resampleRoad(Point roadLine[], int16 *count, int16 startPos, uint8 dis) {
    int16 data remain, data step, data i, data dx, data dy, data len, data x0, data y0, data cx, data cy;
    int32 data tx, data ty;
    Point output[expRoadCount];
    remain = 0;
    step = 0;
    // 入口夹取：负 startPos、超容量 count 都不能进入下面的索引循环
    if (startPos < 0 || *count <= 0 || *count > expRoadCount || *count <= startPos + 1 || dis == 0) {
        *count = 0;
        return;
    }
    for (i = startPos; i < *count - 1; i++) {
        x0 = roadLine[i].x;
        y0 = roadLine[i].y;
        dx = roadLine[i + 1].x - x0;
        dy = roadLine[i + 1].y - y0;
        len = (int16) sqrt_int((int32) dx * dx + (int32) dy * dy);
        if (!len)
            continue;
        while (step < expRoadCount) {
            if (remain >= len) {
                remain -= len;
                break;
            }
            tx = (int32) x0 * len + (int32) dx * remain;
            ty = (int32) y0 * len + (int32) dy * remain;
            cx = (int16) (tx / len);
            cy = (int16) (ty / len);
            output[step].x = func_limit_ab(cx, 0, MT9V03X_H - 1);
            output[step].y = func_limit_ab(cy, 0, MT9V03X_W - 1);
            step++;
            remain += dis;
        }
    }
    if (step < expRoadCount)
        output[step] = roadLine[*count - 1];
    else
        output[--step] = roadLine[*count - 1];
    step++;
    memcpy(roadLine, output, step * sizeof(Point));
    *count = step;
}

int16 findCorner(Point roadLine[], int16 count, int8 cross) {
    /* 局部变量不加 volatile：这里没有中断共享，加 volatile 会强制 C251 每步都回到
     * 内存访存，而这个函数是逐点开方的访存密集区。保留 data 段修饰即可。 */
    int16 data i;
    int16 data p = 0, data c = 0;
    int32 data m = 0;   // 100*abs_dot 远超 int8，原来被截断导致"最尖拐点"分支永不生效
    /* 拐点只在窗口内搜索：点列走到窗口远端行就该停，继续往上是暗区 */
    for (i = 3; i < count - 2 && roadLine[i].x > VIEW_FAR_ROW; i++) {
        int32 i_5 = func_limit_ab(i - 5, 0, count - 1), i0 = func_limit_ab(i, 0, count - 1),
        i5 = func_limit_ab(i + (cross ? 5 : 2), 0, count - 1), dx1 = roadLine[i0].x - roadLine[i_5].x,
        dy1 = roadLine[i0].y - roadLine[i_5].y, dx2 = roadLine[i5].x - roadLine[i0].x,
        dy2 = roadLine[i5].y - roadLine[i0].y, len1_sq = dx1 * dx1 + dy1 * dy1,
        len2_sq = dx2 * dx2 + dy2 * dy2, len1, len2, dot, abs_dot;
        if (!len1_sq || !len2_sq) continue;
        len1 = sqrt_int(len1_sq);
        len2 = sqrt_int(len2_sq);
        dot = dx1 * dx2 + dy1 * dy2;
        abs_dot = (dot >= 0) ? dot : -dot;
        if (100 * abs_dot <= CORNER_COS100 * len1 * len2) {
            if (!p) {
                p = i;
                m = 100 * abs_dot;
                c = 3;
            } else if (m > 100 * abs_dot) {
                m = 100 * abs_dot;
                p = i;
            }
        }
        if (c && c-- == 1) return p;
    }
    return 0;
}


