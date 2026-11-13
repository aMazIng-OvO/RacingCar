# 图像处理链路与参考实现的对齐说明

> 参考实现：`backup/version1.3/project/code/Image.c`、`Image.h`（2671 行 / 43 行）
> 当前实现：`version1.0/project/code/Image.c`、`Image.h`
> 一致性手段：整文件替换，再用哈希与函数体比对复核

---

## 一、参考实现的来源

| 项 | 内容 |
|---|---|
| 路径 | `backup/version1.3/project/code/Image.c`、`Image.h` |
| 规模 | `Image.c` 2671 行（含 `reflection[120][188][2]` 常量表），`Image.h` 43 行 |
| 参考 SHA256（`Image.c`） | `337F28C872A86F323BEB808F6E438DFD814B079BEFD9F326A30FA4FEA9BC1BFA` |
| 参考 SHA256（`Image.h`） | `BD53F91F3CDE4B726D5524621D476655625C8D261B6AA26BC9EEF74CA8DC365A` |
| 当前 SHA256（`Image.c`） | `11BD628AF6FC397A96A92C2EDD5CD7B4950762F3779BE7E9BD8F227DE6933B52` |
| 当前 SHA256（`Image.h`） | `1B66D8E2290C241662756DEBA330F3064BB834B4752295C331F8EAEA8F5AA844` |
| 上游来源 | 浙江工商大学 哈夫克自动智能车项目，`HFK_STC32G144K-master/project/code/Image.c`，经本项目 `version1.3` 修订 |
| 同版本其他参考文件 | `backup/version1.3/project/user/main.c`（`loadRoad()` 与 `elementReset()`）、`project/code/Cross.c`（补线调用） |

`reflection` 标定表与参考实现相同。当前实现与参考的差异只有一处现场标定项 `BODY_ROW`，详见第八节。

## 1.1 车身入镜的现场约束

参考实现假定车身只出现在行 91 以下：`getLIndexStart()` / `getRIndexStart()` 以 `HIDE_ROW = 90` 为界，行 90 以下改用两侧窄列（左 3~48、右 140~186）绕开车身，行 70~89 仍搜全宽。

实车有部分车身出现在行 80~89，那里的中央暗块会被宽列二分当成赛道边界，产生假起点。参考实现里 `getImagePointFilterSelf()` 虽然会把 `i > HIDE_ROW && 48 <= j <= 140` 强制为 255，但它只被 `Cross.c` 的补线调用，起点搜索、`check()`、`getPointThreshold()` 与爬线全部直接读 `mt9v03x_frame` 原始像素，遮蔽对主链路不起作用。

处理方式沿用参考实现自己的思路：新增 `BODY_ROW = 80`，把起点搜索切换到窄列的界线上移，`HIDE_ROW` 保持 90 给补线遮挡使用。行 80~116 用窄列，行 70~79 仍用宽列兜底，保证车身遮住近处边界时还能在更远处找到真实起点。

---

## 二、核心处理步骤

每帧一次，在主循环的 `if (frameBusy)` 分支内调用 `loadRoad()`。链路共 8 步，顺序固定。

| 步骤 | 函数 | 输入 | 输出 | 判据 |
|---|---|---|---|---|
| 1 | `getLStart()` / `getRStart()` | 快照、`LThreshold` / `RThreshold` | 起点 `Point` | 从行 116 向行 70 按 3 行步长扫描，二分找第一处符合 `check()` 的灰度跳变 |
| 2 | `getLBorder()` / `getRBorder()` | 起点、点数上限 | 边界点数组、点数 | 八邻域爬行，步长 2 像素，`turn >= 4` 退出 |
| 3 | 拷贝 | 上一步结果 | `LBorder[]` / `RBorder[]`（原始坐标副本） | 供元素模块使用 |
| 4 | `getGodRoadLine()` | 边界点 | 俯视坐标边界点 | 查 `reflection[x][y]`，越界点跳过 |
| 5 | `filterRoad()` | 俯视边界点 | 平滑后的边界点 | 因果滤波 `(前一点 + 当前点 + 1) >> 1` |
| 6 | `resampleRoad(dis = 3)` | 平滑边界点 | 等弧长重采样点 | 按弧长累加，上限 `expRoadCount` |
| 7 | `findCorner()` | 重采样边界点 | 拐点序号 | 夹角余弦 `<= 0.50`，连续 3 点确认，只在前 40 像素以远搜索 |
| 8 | `getCenterLineByLBorder()` 或 `getCenterLineByRBorder()` | 单侧边界、`ALONG` | 中线 | 沿边界法向平移 `centerLineDistance` |

中线生成后再做一次 `filterRoad()` 与 `resampleRoad(dis = 2)`，见 `loadRoad()`。

### 2.1 `loadRoad()` 的调用序列

```c
Point LStart = getLStart(), RStart = getRStart();
LRoadCount = expRoadCount, RRoadCount = expRoadCount;
getLBorder(LRoad, LStart, &LRoadCount);
getRBorder(RRoad, RStart, &RRoadCount);
LBorderCount = func_limit_ab(LRoadCount, 0, expRoadCount);
memcpy(LBorder, LRoad, LBorderCount * sizeof(Point));
RBorderCount = func_limit_ab(RRoadCount, 0, expRoadCount);
memcpy(RBorder, RRoad, RBorderCount * sizeof(Point));
getGodRoadLine(LRoad, LRoadCount);
getGodRoadLine(RRoad, RRoadCount);
filterRoad(LRoad, LRoadCount);
filterRoad(RRoad, RRoadCount);
resampleRoad(LRoad, &LRoadCount, 0, 3);
resampleRoad(RRoad, &RRoadCount, 0, 3);
LCorner = findCorner(LRoad, LRoadCount, 0);
RCorner = findCorner(RRoad, RRoadCount, 0);
/* borderIn 判定、拐点截断、单侧中线、ALONG 点数投票 */
```

要点：

- 两侧点数在调用爬线前先置 `expRoadCount`，爬线函数以它作为写入上限。
- `ALONG` 在函数末尾由两侧点数无条件重算：`LRoadCount > RRoadCount ? ALONG_L : ALONG_R`。
- 中线只取一侧边界外推，不做双侧融合。

---

## 三、关键参数与阈值

以下值来自 `Image.h` 与 `Image.c`，与参考实现完全相同。

| 符号 | 值 | 位置 | 作用 |
|---|---|---|---|
| `block` | 5 | `Image.h` | 局部阈值窗口边长 |
| `cap` | 4 | `Image.h` | 局部阈值去噪余量 |
| `expRoadCount` | 100 | `Image.h` | 边界点数组容量 |
| `EDGE_DIFF` | 12 | `Image.h` | 边界灰度跳变下限 |
| `EDGE_RATIO` | 7 | `Image.h` | `(亮+暗)/(亮-暗)` 上限 |
| `HIDE_ROW` | 90 | `Image.h` | 车身遮挡区起始行 |
| `HIDE_COL_L` / `HIDE_COL_R` | 48 / 140 | `Image.h` | 遮挡区列范围 |
| `CORNER_COS100` | 50 | `Image.h` | 拐点夹角余弦阈值 × 100 |
| 初始阈值对比度门槛 | 20 | `imageInitThreshold()` | 低于此值返回兜底值 128 |
| 初始阈值采样带 | 行 92~113，步长 2 | `imageInitThreshold()` | 灰度最小最大值的中点 |
| 起点扫描范围 | 行 116 → 70，步长 3 | `getLStart()` / `getRStart()` | 近到远 |
| `BODY_ROW` | 80 | `Image.h` | 该行及以上改用全宽搜索，以下只用两侧窄列；参考实现的界是 `HIDE_ROW = 90` |
| 爬线步长 | 2 像素 | `dirF` / `dirL` / `dirR` | 八邻域 |
| 爬线转向上限 | 4 次 | `getLBorder()` | 连续转向即退出 |
| 阈值滑动触发点 | 点数 > 25 | `getLBorder()` / `getRBorder()` | 半权重跟进 |
| `centerLineDistance` | 18，拐角处 9 | `main.c` / `Image.c` | 中线法向偏移 |
| 边界重采样间距 | 3 | `loadRoad()` | 等弧长 |
| 中线重采样间距 | 2 | `loadRoad()` | 等弧长 |

### 3.1 阈值更新公式

```c
LThreshold = (int16) tempThreshold + (int16) LThreshold >> 1;
threshold  = (int16) LThreshold + (int16) RThreshold >> 1;
```

C 的移位优先级低于加法，实际计算的是 `(tempThreshold + LThreshold) >> 1`。这一点与参考实现一致，**不要额外加括号**，否则结果不同。

---

## 四、输入输出数据格式与精度

### 4.1 输入

| 数据 | 类型 | 范围 | 说明 |
|---|---|---|---|
| `mt9v03x_frame[120][188]` | `uint8` | 0~255 | 灰度快照，DMA 完成中断内搬入 |
| `threshold` / `LThreshold` / `RThreshold` | `uint8` | 0~255 | 全局与单侧阈值，128 表示对比度不足 |
| `banUpdateThreshold` | `uint8` | 0 / 1 | 为 1 时冻结阈值滑动 |
| `centerLineDistance` | `uint8` | 9 / 18 | 中线法向偏移 |

### 4.2 输出

| 数据 | 类型 | 精度 | 说明 |
|---|---|---|---|
| `Point.x` | `uint8` | 0~119 | 图像行号，去畸变后为俯视行号 |
| `Point.y` | `uint8` | 0~187 | 图像列号，去畸变后为俯视列号 |
| `LRoadCount` / `RRoadCount` | `int16` | 0~100 | 边界点数，0 表示该侧无效 |
| `LBorderCount` / `RBorderCount` | `int16` | 0~100 | 去畸变之前的原始边界点数 |
| `centerLineCount` | `int16` | 0~100 | 中线点数 |
| `LCorner` / `RCorner` | `int16` | 0~100 | 拐点序号，0 表示没有拐点 |

### 4.3 精度约束

- 爬线以 2 像素为一步，边界点的位置精度是 2 像素。
- 去畸变查表为整像素映射，不插值。
- `resampleRoad` 在段内做整数线性插值，`tx / len` 的整数除法会向下取整，最大误差 1 像素。
- 所有中间量用 `int16` 存储，平方运算前必须提升到 `int32`（`dy²` 可达 34969）。

---

## 五、异常与边界情况的处理

| 情况 | 处理方式 | 代码位置 |
|---|---|---|
| 起点搜索失败 | `getLStart()` 返回 `(0,0)`，`getLBorder()` 立即把点数置 0 返回 | `getLStart()` / `getLBorder()` |
| 快照读取越界 | `getImagePointFilterSelf()` 对 `i >= 120` 或 `j >= 188` 返回 0 | `getImagePointFilterSelf()` |
| 车身遮挡区 | `i > 90 && 48 <= j <= 140` 时返回 255（强制为白），只作用于 `Cross.c` 补线 | `getImagePointFilterSelf()` |
| 车身出现在行 80~89 | `getLIndexStart()` / `getRIndexStart()` 以 `BODY_ROW = 80` 为界改用两侧窄列，跳过车身暗块；行 70~79 仍用宽列兜底 | `Image.c` / `Image.h` |
| 局部阈值窗口越界 | 先把窗口中心夹到 `[halfBlock, 119-halfBlock]` 与 `[halfBlock, 187-halfBlock]` | `getPointThreshold()` |
| 画面没有明暗差 | `imageInitThreshold()` 返回 128 | `imageInitThreshold()` |
| 去畸变坐标越界 | 该点保持原值不发生映射，继续处理下一个点 | `getGodRoadLine()` |
| 重采样入口参数异常 | 负 `startPos`、`count > 100`、`count <= startPos + 1`、`dis == 0` 时把点数置 0 返回 | `resampleRoad()` |
| 重采样输出超出容量 | 结果点数达到 `expRoadCount` 时最后一点覆盖写入并收束 | `resampleRoad()` |
| 边界点写入超出容量 | `limit = func_limit_ab(*count, 0, expRoadCount)` 先夹取 | `getLBorder()` / `getRBorder()` |
| 爬线回到起点 | `x == start.x && y == start.y && !turn` 时跳出循环 | `getLBorder()` / `getRBorder()` |
| 边界点数不足以构造中线 | `centerLineCount < 2` 时把中线点数置 0，`alongLine()` 直接返回、控制量保持 | `loadRoad()` / `alongLine()` |
| 两侧边界全丢 | 出界判定成立，`FAULT_OUT` 置位并停车 | `main.c` / `Motor.c` |

---

## 六、是否在赛道上的识别

参考实现没有独立的赛道判定函数：`main.c` 的 `updateStatusLed()` 注释写明"不含赛道检测"，`B2` 按键直接调用 `raceStart()`，不设门槛。参考实现里唯一与赛道状态相关的量是出界保护的条件：

```c
if ((!LRoadCount && !RRoadCount) || (5 个采样点全暗)) { 停车 }
```

当前实现在 `Track.c` 里取这条条件的补集作为"在赛道上"的判据：

```c
uint8 trackEvaluate(void) {
    rawOn = (uint8) ((LRoadCount > 0) || (RRoadCount > 0));
    return rawOn;
}
```

| `LRoadCount` | `RRoadCount` | 参考出界条件 | 当前赛道判定 |
|---|---|---|---|
| 0 | 0 | 成立 | 不在赛道 |
| 0 | 非 0 | 不成立 | 在赛道 |
| 非 0 | 0 | 不成立 | 在赛道 |
| 非 0 | 非 0 | 不成立 | 在赛道 |

判定的输入就是 `loadRoad()` 产出的边界点数，不额外读像素，所以相同输入必得相同结论。赛道状态只用于串口输出与蓝灯以外的指示，**不阻断发车**，与参考实现的行为一致。

---

## 七、测试用例与预期结果

### 7.1 对齐核对（`tests/reference_alignment_check.py`）

| 用例 | 输入 | 预期结果 | 实际结果 |
|---|---|---|---|
| T1 `Image.c` 差异限定 | 两份 `Image.c` 的行级差异 | 只有 2 行，都是 `i < HIDE_ROW` → `i < BODY_ROW` | 删除 2 行、新增 2 行，符合预期 |
| T2 `Image.h` 差异限定 | 两份 `Image.h` 的行级差异 | 只新增 `BODY_ROW` 及其注释 | 删除 0 行、新增 5 行，符合预期 |
| T3 调用序列一致 | 两份 `main.c` 的 `loadRoad()` | 去注释与空白后逐行相同 | 25 行全部相同 |
| T4 关键参数一致 | 9 个宏 | 取值相同 | `block=5 cap=4 expRoadCount=100 EDGE_DIFF=12 EDGE_RATIO=7 HIDE_ROW=90 HIDE_COL_L=48 HIDE_COL_R=140 CORNER_COS100=50` |
| T5 赛道判据互补 | 8 组点数组合 | 两者互为补集 | 8/8 通过 |
| T6 上电初值一致 | `elementReset()` | 两侧点数都置 `expRoadCount` | 相同 |

### 7.1 车身入镜核对（`tests/body_row_check.py`）

模拟 `getLIndexStart()` 的二分与 `check()`，图像为 188 × 120，赛道白 200 在列 30~158，底布 60，车身暗块 40，阈值 130。

| 场景 | 参考 `HIDE_ROW = 90` | 当前 `BODY_ROW = 80` | 期望 |
|---|---|---|---|
| 无车身 | 起点 `(116, 31)` | `(116, 31)` | 一致 |
| 车身只在中央 55~125，行 70~116 | `(116, 31)` | `(116, 31)` | 一致 |
| 车身遮住左边界 20~60，行 80~116 | `(89, 62)`，车身右缘被当成边界 | `(77, 31)`，真实赛道边界 | 当前正确 |
| 车身延伸到列 10~100，行 80~116 | `(77, 31)` | `(77, 31)` | 一致 |

逐行分析（车身 20~60，真实边界 30）：

| 行号 | 参考列 / 起点有效 | 当前列 / 起点有效 |
|---|---|---|
| 116 / 100 / 92 | 48 / 否 | 48 / 否 |
| 89 | 61 / 是（假起点） | 48 / 否（跳过） |
| 80 | 61 / 是（假起点） | 48 / 否（跳过） |
| 77 / 74 / 71 | 30 / 是 | 30 / 是 |

窄列搜索在直道上仍可用：无车身时参考与当前的起点都是 `(116, 31)`。

### 7.2 赛道判定真值表

| 输入 `(LRoadCount, RRoadCount)` | 参考判为出界 | 当前判为在赛道 | 期望 |
|---|---|---|---|
| (0, 0) | 是 | 否 | 通过 |
| (0, 1) | 否 | 是 | 通过 |
| (0, 40) | 否 | 是 | 通过 |
| (0, 100) | 否 | 是 | 通过 |
| (1, 0) | 否 | 是 | 通过 |
| (40, 0) | 否 | 是 | 通过 |
| (100, 0) | 否 | 是 | 通过 |
| (40, 40) | 否 | 是 | 通过 |

### 7.3 编译

```
0 Error(s), 1 Warning(s)        仅逐飞库 zf_device_mt9v03x.c(138) C183
data=102.5  edata+hdata=8192  xdata=51380  const=48722  code=78525
```

参考实现的 `README.md` 记录的条件编译告警是"3 条：`zf_device_mt9v03x.c(138)` 与 `Image.c(147/176)` 两条 `C173`"，并说明把 `step` 改为 `int16` 后两条 `C173` 应消失。当前只剩 1 条，与这条记录吻合。

### 7.4 相同输入产生相同输出的依据

`Image.c` 与参考的差异经 T1 验证严格限定为两行 `HIDE_ROW` → `BODY_ROW`，`Image.h` 经 T2 验证只新增 `BODY_ROW` 定义。`loadRoad()` 的调用序列经 T3 验证与参考一致。因此：

- 车身未进入行 70~89 的画面（或车身只在中央 48~140 且不遮挡两侧边界）时，同一帧 `mt9v03x_frame` 与同一组阈值进入后，`LRoad[]` / `RRoad[]` / `LRoadCount` / `RRoadCount` / `centerLine[]` / `LCorner` / `RCorner` 与参考实现完全相同。
- 车身进入行 70~89 时，当前实现跳过被车身污染的行，从更远处找真实起点；这是 `BODY_ROW` 带来的唯一行为差异，核对见 7.1 节。
- 赛道判定直接由 `LRoadCount` / `RRoadCount` 得出，同一组点数必得同一结论。

---

## 八、与参考实现保持一致的范围

| 文件 | 处理 |
|---|---|
| `project/code/Image.c` | 整文件来自参考实现，仅两处 `i < HIDE_ROW` 改为 `i < BODY_ROW` |
| `project/code/Image.h` | 整文件来自参考实现，仅新增 `BODY_ROW = 80` 及其注释 |
| `project/user/main.c` 的 `loadRoad()` | 恢复为参考实现的调用序列 |
| `project/user/main.c` 的 `elementReset()` | `LRoadCount` / `RRoadCount` 置 `expRoadCount` |
| `project/code/Cross.c` | 补线调用恢复为 `getLBorder()` / `getRBorder()`，点数门槛恢复为 40 |
| `project/code/Track.c` | 判定改为参考出界条件的补集，蓝灯位掩码恢复为四个故障位 |
| `project/user/main.c` 的 B2 分支 | 取消赛道门槛，与参考一致直接发车 |

以下部分不在图像链路范围内，保留当前实现：电机与舵机的控制参数（`Motor.c` / `Motor.h`）、出界判定所在的执行上下文（主循环判定、控制中断消抖）、按键计时与调试串口输出。
