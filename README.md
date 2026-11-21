# Racing Car — STC32G144K 摄像头循迹车

基于 STC32G144K246 与逐飞 MT9V03X 摄像头的竞速智能车工程，Keil C251（MCS-251）工具链，裸机前后台 + 5 ms 控制中断。

当前分支 `version-1.4`，基线提交 `57222a3`（`version-1.3` 末次提交）。

---

## 代码来源声明

**自 `version-1.4` 起，本工程 `project/` 目录下的应用层代码全部由本队自行编写与维护。**

具体范围：

| 模块 | 文件 | 来源 |
|---|---|---|
| 图像链路 | `project/code/Image.c/.h` | 本队自研 |
| 控制链路 | `project/code/Motor.c/.h` | 本队自研 |
| 元素状态机 | `SCurve.c` / `Round.c` / `XCross.c` / `BendEntry.c` | 本队自研 |
| 发车与安全 | `Stop.c` / `Track.c` | 本队自研 |
| 调度主循环 | `project/user/main.c` | 本队自研 |
| 离线校验 | `tests/*.py` | 本队自研 |

应用层不含任何外部现成赛车的算法实现片段：图像处理（局部阈值爬线、等弧长重采样、余弦拐点、双侧中线融合）、纯跟踪与速度规划、元素状态机、异常检测，均为本队按本项目硬件与赛道标定的实现，设计细节见源码注释。

（文件级的上游派生历史——中断入口、工程文件组织——已在 `NOTICE` 中逐条列明，不涉及上述算法实现。）

**必须同时说明的第三方部分**（见 `NOTICE` 与 `LICENSE`，GPL-3.0 要求保留，不得删除）：

| 组件 | 位置 | 来源 | 许可 |
|---|---|---|---|
| `zf_` 驱动库 / 设备库 | `libraries/` | 成都逐飞科技 STC32G144K 开源库 V3.2.5 | GPL-3.0，保留原文件头 |
| `sqrt_int()` / `atan2_int()` | `libraries/zf_common/zf_common_function.c` | 上游不提供，由本队追加（已在 NOTICE 列明） | GPL-3.0 |
| 中断入口与向量表 | `project/user/isr.c` / `isr_extended_*.c` / `isr_extended_vectors.A51` | 结构沿用逐飞中断模板，内容按 C251 向量号 0~31 限制重写 | GPL-3.0 |
| 工程文件与启动代码 | `project/mdk/seekfree.uvproj`、`libraries/zf_common/START251.A51` | 逐飞模板，仅增删文件列表 | 原样引用 |
| 芯片寄存器头 | `libraries/zf_common/stc32g144k246.h` | STC 官方 | 原样引用 |

本仓库整体仍以 **GNU General Public License v3.0** 分发。再分发须保留 `LICENSE` / `NOTICE` 全文、版权声明，并标明修改内容。

---

## 版本演进

| 版本 | 分支 | 主要内容 |
|---|---|---|
| 1.0 | `version1.1` 之前 | 从 C-model 模板起步：双缓冲采集、扩展中断向量处理、发车 / 里程 / 定点停车 |
| 1.1 | `version1.1` | 实车验证的稳定基线 |
| 1.2 | `version-1.2` / `version-1.2.1` | 出界保护与赛道识别完善 |
| 1.3 | `version-1.3` | 弯道入口识别、自适应转向增益、轨迹偏置标定 |
| **1.4** | `version-1.4` | 元素架构重构为 `XCross`/`Round`/`SCurve` 三模块互斥仲裁；环岛入口改为「严格路径 + 几何路径」双判据；前瞻取法可选、出界判据修正、发车起步斜坡、黑匣子压缩到 64 字节包（见第 12 节） |

---

## 1. 硬件

| 项目 | 型号 / 参数 |
|---|---|
| MCU | STC32G144K246，96 MHz，256 KB Flash / 128 KB XRAM |
| 摄像头 | 逐飞 MT9V03X，188 × 120 灰度，LCM_DMA 采集 |
| 舵机 | 200 Hz PWM，中值 `steerMid = 2975`（`Motor.c:10`），机械修正 `STEER_TRIM = -58` |
| 电机 | 双直流减速电机，17 kHz PWM，AB 相编码器正交解码 |
| 调试 | USB CDC 虚拟串口，`printf` 输出 |
| 安装 | 摄像头高 24 cm，光轴与立杆夹角 63°（俯角 27°），距车头 15 cm；FOV 120°，机内广角校正 |
| 车体 | 全长 29 cm，两轮间距 18.2 cm，轮半径 3.1 cm |

### 1.1 引脚

| 功能 | 引脚 | 外设 | 方向 |
|---|---|---|---|
| 右轮 PWM | P76 | PWMB CH3 | 输出 |
| 右轮 DIR | P77 | GPIO | 输出 |
| 左轮 PWM | P74 | PWMB CH1 | 输出 |
| 左轮 DIR | P75 | GPIO | 输出 |
| 右编码器 A / B | P60 / P62 | PWMA CH1P / CH2P | 输入 |
| 左编码器 A / B | P40 / P42 | PWMC CH1P / CH2P | 输入 |
| 舵机信号 | PA4 | PWME CH3P | 输出 |
| 状态灯 | P52 | GPIO | 输出，低电平点亮 |
| 拨码 SW1 / SW2 | PB0 / PB1 | GPIO 上拉 | 输入 |
| 按键 B2 / B3 / B4 | PB2 / PB3 / PB4 | GPIO 上拉 | 输入 |
| 摄像头 D0~D7 | P20~P27 | GPIO | 输入 |
| 摄像头 SCL / SDA | PB6 / PB7 | 软件 IIC | 双向 |
| FIFO VSY | P33 | INT1 | 输入 |
| FIFO 控制 | P37 / P41 / P73 / P72 / P43 | GPIO | 输出 |
| DMA 占用 | P45 / P36 | — | 不可复用 |
| USB | P30 / P31 | USB 全速设备 | 双向 |

## 2. 目录

```
project/
  code/
    Image.c/.h        快照、视野窗口、起点搜索、爬线、重采样、拐点
    Motor.c/.h        纯跟踪、舵机换算、差速、双轮 PID、故障检测
    SCurve.c/.h       连续 S 弯状态机（RUN_S_CURVE=314）
    Round.c/.h        环岛状态机（左右共用 side 区分，5 阶段 + 入口双判据）
    XCross.c/.h       十字形环岛（X 路口）状态机（RUN_X_CROSS=514）
    BendEntry.c/.h    弯道入口过渡区（出界宽容 + 转角保持）
    Stop.c/.h         发车、里程标定、定点停车
    Track.c/.h        赛道存在性判定与状态灯
  user/
    main.c            上电流程、按键、主循环、元素仲裁、中线融合、黑匣子
    isr.c             向量号 0~31 的中断入口
    isr_extended_0.c  P0~P9、DMA UART、DMA_LCM、TM11
    isr_extended_1.c  其余扩展中断
    isr_extended_vectors.A51  扩展中断跳转表
  mdk/                uVision 工程与编译产物
libraries/            逐飞库（仅保留当前使用的 MT9V03X、USB 与基础驱动）
tests/                Python 离线校验脚本（9 个）
```

**元素模块说明**：`main.c` 只负责"派发谁执行"与"总时长兜底"，不持有任何元素阈值。三个元素模块互斥运行，优先级 `XCross > Round > SCurve`，同一时刻只有一个激活。旧版 `Cross.c` / `LRound.c` / `RRound.c` 已从源码树移除（原文件从未列入 `seekfree.uvproj`、不参与编译）。

## 3. 构建与烧录

```powershell
$uv   = 'D:\Keil_v5\UV4\UV4.exe'
$proj = (Join-Path (Get-Location) 'project\mdk\seekfree.uvproj')   # 在本工程根目录执行
# -r 全量重建 / -b 增量构建；-j0 关闭并行；日志写到临时目录避免污染工作区
Start-Process $uv -ArgumentList '-r', "`"$proj`"", '-j0', '-o', "`"$env:TEMP\build.log`"" -Wait -PassThru -WindowStyle Hidden
```

原开发环境路径为 `f:\项目文件夹\Racing Car C-model\main\project\mdk\seekfree.uvproj`；本备份在其所在目录用上面的相对定位编译即可。工程路径含空格，**命令行里必须给路径加引号**，否则 `UV4` 直接返回 15（读不到工程）。退出码：0=无错无警告、1=仅警告、≥2=有错；日志含中文，用 `Get-Content -Encoding Default` 读。

产物：`project/mdk/out_file/SEEKFREE.hex`，同时生成 `SEEKFREE.map` 与 `.lnp`。

本备份基线结果（清理后，`project/mdk/out_file/SEEKFREE.build_log.htm`，2026-11-21）：

```
Program Size: data=77.2  edata+hdata=8192  xdata=59262  const=2737  code=72154
".\out_file\SEEKFREE" - 0 Error(s), 1 Warning(s)
```

唯一警告是逐飞库自带的 `zf_device_mt9v03x.c(138): C183 dead assignment eliminated`，属第三方代码，不做修改。清理时已删除未使用的 IPS200 显示驱动、IMU / 无线 / TOF / BLE / 逐飞助手模块、ADC / EEPROM / SPI 驱动、重复 SPI-DMA 文件、45 KB 反射查表以及零调用应用函数；`code` 较清理前减少 14127 字节，`xdata` 减少 2088 字节，`const` 减少 991 字节。

## 4. 上电与操作

| 步骤 | 现象 |
|---|---|
| 上电 | 状态灯闪 3 下；默认不输出周期调试行（`ENABLE_LIVE_TELEMETRY=0`），置 1 后 USB CDC 才输出 `ODO=` |
| 摄像头初始化 | 失败时状态灯 500 ms 慢闪并停住（最多重试 3 s） |
| 舵机自检 | `steerMid + STEER_TRIM` → 一侧 30° → 另一侧 30° → 回中，约 1.5 s（`SERVO_TEST_SWING=495`、`SERVO_TEST_STEP=33`，三段精确落在端点） |
| 选档 | 读 PB0 / PB1，从 8 档里取 1 / 3 / 5 / 7 档 |
| 待发车 | 状态灯按编码器状态闪烁 |

| 按键 | 功能 |
|---|---|
| B2 | 发车（`RACE_IDLE` / `RACE_STOPPED` 时可发） |
| B3 | 任意状态紧急停车 |
| B4 短按 | 上传黑匣子全部记录，末尾输出 `ENDDDDD`；行驶中按下则停车后上传 |
| B4 长按 2 s | 待发车状态下把当前里程记为一圈 |

`CALIB_REQUIRED = 0`：不标定也能发车，此时 `raceStart()` 把 `lapPulse` 设为 `0x7FFFFFFF`，不会因里程自动刹停，只能靠 B3 手动停。

黑匣子默认不主动上传；只有 B4 短按才会发送全部记录。需要恢复周期性 `ODO=` 调试行时，把 `main.c` 的 `ENABLE_LIVE_TELEMETRY` 改为 `1`。

## 5. 图像链路

```
DMA 写 mt9v03x_image
  → DMA 完成中断 mt9v03x_frame_snapshot() 搬 22.5 KB 到 mt9v03x_frame，置 frameBusy
  → 主循环 frameBusy == 1
      cutViewRegion()               视野窗口：只保留行 34~99、列 34~153，四周置暗
      getLStart / getRStart         行 95 → 37，列 36 → 151，车身行以下用窄列
      getLBorder / getRBorder       四邻域爬线，局部阈值每 2 步重算，turn < 8
      filterRoad                    因果平均 (前 + 今 + 1) >> 1
      resampleRoad(dis = 3)         等弧长重采样
      findCorner                    夹角余弦找拐点，裁短点数
      buildCenterLine               双侧配对取中点 / 单侧按实测半宽外推
      filterRoad + resampleRoad(2)  中线平滑与等距化，首点锚到窗口下沿
```

设计细节以 `Image.c` / `Image.h` 的源码注释为准。

## 6. 控制链路

```
5 ms 控制中断 pitHandler
  编码器读数 → faultScan → 出界消抖
  → 差速分配 → 左右增量式 PID → pwm_set_duty
  → steerWrite(steerMid + STEER_TRIM + steerPIDCalculate(&steerPid, duty))
```

| 环节 | 实现 |
|---|---|
| 前瞻点 | `getAimPos()` = `c3 + currentSpeed / 40`，夹 `c3 ~ c3 + SPEED_LIMIT/40` |
| 取点方式 | `PP_ARC_MODE = 0`：按下标 `pos`，并夹 `count-1-12`；`= 1`：按弧长累计，线短时沿末端切线外推 |
| 纯跟踪 | `d = atan2_int(2·L·dy, dx² + dy²)`，`L = PP_LOOKAHEAD_PX = 28`，`dx = 99 - 点行 + 28`（下限 30），`dy = 94 - 点列` |
| 远端前馈 | `dy = dyAim + (dyFar - dyAim) / BEND_FEED_DIV(=3)`，远端点在前瞻点之后 `BEND_FAR_AHEAD=12` 个点距（约 24 像素） |
| 舵机换算 | `currentDuty = d × c1 / 20`，夹 ±395；`d` 先夹 ±30 |
| 舵机环 | `steerPid` Kp 100 / Ki 0 / Kd 100，输出夹 ±395 |
| 曲率限速 | `r = 20000 / tan(abs(d))`，`v = √(r·c4)`，`target = min(v, c5)` |
| 弯道减速 | `bend = abs(dyFar - dyAim) + abs(d)`，比例 `100 - (bend-4)×3` 夹到 `BEND_SLOW_MIN=40`~100，`target` 再被 `BEND_SPEED_CAP=170` 硬顶，`bendHold` 记住本弯最严值 |
| 弯道增益 | 减速状态下角度放大 `BEND_STEER_PCT=110` ~ `BEND_STEER_MAX=185`%（强度 `BEND_GAIN_BEND=15` 起每单位 +`BEND_GAIN_RATE=6` 个百分点） |
| 差速 | `diff = setSpeed × 8 / R`，夹 ±`setSpeed/2`；`R = 20000 / tan(abs(d))` 带符号 |
| 电机环 | 增量式，Kp 1000 / Ki 50 / Kd 0，输出 ±10000；刹车段负向限幅收窄到 -1500 |

## 7. 元素识别

三个元素模块在 `main.c` 里**互斥仲裁**，同一时刻只有一个激活，优先级 `XCross > Round > SCurve`。占用期间其余模块的 `detect()` 不调用，避免串扰；`main.c` 不持有任何元素阈值，只做派发与总时长兜底（`ELEM_TOTAL_MS=20000`）。

| 元素 | MODE | 源文件 | 状态机 / 触发 |
|---|---|---|---|
| 十字形环岛（X 路口） | `RUN_X_CROSS=514` | `XCross.c` | `xcDetect()` 入口：`XC_MAX_PTS=6` / `XC_WEAK_PTS=10` / `XC_MAX_CL=10`，连续 `XC_CONFIRM=3` 帧；速度比例 `XC_SPEED_RATIO=60`%；阶段超时 `XC_MAX_MS=800` |
| 环岛（左右共用 side） | `RUN_ROUND_L=810` / `RUN_ROUND_R=1919` | `Round.c` | 5 阶段 `READY→START→IN→END→EXIT`；入口两条判据见下；积分 `RD_SCORE_NEED=6`；速度比例 `RD_SPEED_RATIO=50`%；阶段超时 `RD_STAGE_MS=8000`、`RD_READY_MS=1800` |
| 连续 S 弯 | `RUN_S_CURVE=314` | `SCurve.c` | 斜率窗口 `SC_WINDOW=12`、斜率下限 `SC_SLOPE_MIN=3`、峰谷差 `SC_P2P_MIN=8`，释放 `SC_RELEASE=5`；速度比例 `SC_SPEED_RATIO=70`%；时长 200~3000 ms |
| 弯道入口过渡 | （非元素，普通循迹分支） | `BendEntry.c` | `ENABLE_BEND_ENTRY=1`，4 状态 `STRAIGHT→APPROACH→COMMITTED→RECOVER`；`BE_CONFIRM_FRAMES=2`，`BE_BLANK_MS=250`，`BE_MAX_LOST_MS=900`，`BE_MAX_APPROACH_MS=400` |

**环岛入口双判据**（`Round.c` 的 `rdCand()`，满足其一即可，`side=0` 右环 / `side=1` 左环）：

1. **严格路径**：外侧原始点数 `> RD_OUTER_MIN=20`、外侧末点行 `<= RD_FAR_ROW=42`、内侧裁短点数 `< RD_INNER_MAX=7`、外侧弯曲度 `<= RD_OUTER_AVG_MAX=4`、无反向打角（`RD_ENTRY_OPP_MAX=10`）。
2. **几何路径** `rdGeomCand()`：放宽点数/直度（`RD_GEOM_*`），但必须同时满足宽度相对标准宽度扩张 `RD_WIDTH_EXPAND_PX=8`（`rdWidthExpand()`）且内侧边界有局部极值 `rdInnerTurn()`（`RD_TURN_PROM=5`）或存在拐点；反向打角上限放宽到 `RD_GEOM_OPP_MAX=14`。

两条判据都用 `LRawCount` / `RRawCount`（拐点裁短前的原始点数）作外侧长度，避免外侧边界自带拐点时被裁短而漏判。标准宽度基准 `rdBaseWidth` 只在宽度稳定时以 3/4 旧值 + 1/4 新值缓慢跟随，检测到扩张时冻结。

元素开关：`BendEntry.c` 的 `ENABLE_BEND_ENTRY=1`；三个元素模块无独立 `ENABLE_*` 开关（旧文档所写的 `ENABLE_CROSS` / `ENABLE_ROUND` 在当前源码中不存在），由 `xcDetect()` / `rdDetect()` / `scDetect()` 自身判据决定是否进入。

## 8. 安全保护

| 保护 | 触发条件 | 动作 |
|---|---|---|
| 出界 | 5 个采样点全暗 **且** 已 200 ms 没看到赛道，连续 3 帧成立后再经 3 拍消抖 | 停车，`ERR` 置 0x0400 |
| 编码器掉线 | 目标 > 60、输出 > 2000、该轮 300 ms 计数 < 3 | 停车，`ERR` 置 0x0002 / 0x0004 |
| PID 饱和 | 输出 ≥ 96% 上限且速度 < 目标一半，持续 1 s | 停车，`ERR` 置 0x0100 |
| 正反馈 | 正向输出 > 1500 且编码器 < -5，持续 500 ms | 停车，`ERR` 置 0x0200 |
| 帧看门狗 | 1 s 无新帧 | `raceStopNow()` |
| 元素超时 | 单个元素阶段超过 8 s（`RD_STAGE_MS`），或整个元素超过 20 s（`ELEM_TOTAL_MS`） | 强制回 `FORWARD` 并复位 |
| 环岛误判兜底 | `READY` 停留 1800 ms（`RD_READY_MS`）仍等不到缺口 | 回 `FORWARD`，解除限速 |
| 环岛全丢线 | 两侧边界与中线同时为 0 持续 1000 ms（`RD_LOST_MS`） | 强制退出环岛 |
| 发车宽限 | 发车后 300 ms（`OUT_START_GRACE_MS`） | 不判出界 |

出界判据要求三条信号同时成立，且都要持续（`main.c` 的 `OUT_*` 宏）：

1. 5 个采样点全部偏暗（`outSamplePt`，每点取 3×3 块均值）；
2. 距最近一次“看到赛道”已超过 `OUT_BLIND_MS = 200` ms；
   “看到赛道”用**拐点裁短前**的 `LRawCount` / `RRawCount` 判定——用裁短值会在十字/环岛期间恒不成立；
3. 上述结论连续 `OUT_CONFIRM_FRAMES = 3` 帧，之后控制中断再做 3 拍（15 ms）消抖。

采样点全部放在车身顶边 `BODY_ROW = 80` 以上（`OUT_ROW_NEAR=76`，近端行 74~76、远端行 66）并收进赛道内侧（`outSamplePt` 五点列号为 94 / 74 / 114 / 80 / 108，左右收到 ±20）：
摄像头视场远大于赛道，画面边缘与远端本来就是场地外暗区，早期版本把 3 个点放在行 84~88
（车身遮挡区）、2 个点放在列 47 / 141（常在线外），等于长期白送暗点，这是直道偶发停车的主因。

单侧还有边界时不停车：画面里仍有可循迹的线，交给纯跟踪把车拉回。

## 9. 参数表

`speedSet[8][4]`，列为 `{c1, c3, c4, c5}`：

| 档 | c1 | c3 | c4 | c5 |
|---|---|---|---|---|
| 1 | 260 | 26 | 2400 | 400 |
| 2 | 200 | 22 | 2000 | 300 |
| 3 | 210 | 23 | 2000 | 320 |
| 4 | 220 | 23 | 2000 | 340 |
| 5 | 230 | 24 | 2000 | 360 |
| 6 | 240 | 24 | 2000 | 380 |
| 7 | 250 | 25 | 2000 | 400 |
| 8 | 260 | 25 | 2000 | 400 |

拨码组合 `(SW2, SW1)` 映射到 1 / 3 / 5 / 7 档：`00 → 1 档`、`01 → 3 档`、`10 → 5 档`、`11 → 7 档`（0 为拨码断开）。

| 宏 | 值 | 说明 |
|---|---|---|
| `STEER_DUTY_LIMIT` | 395 | 舵机相对中值的最大偏移（1 duty = 0.5 µs） |
| `STEER_DUTY_DIV` | 20 | `duty = d × c1 / 20` |
| `STEER_TRIM` | -58 | 舵机机械中值修正，停车、自检、初始化三处统一叠加 |
| `TRACK_BIAS_PX` | 0 | 纯跟踪瞄准点横向偏置 |
| `MOTOR_DIFF_TRIM` | 0 | 左右轮目标速度差补偿 |
| `PP_LOOKAHEAD_PX` | 28 | 纯跟踪等效轴距（转角分母里的 L），唯一调参入口 |
| `PP_ARC_MODE` | 0 | 前瞻取法：0 按下标，1 按弧长（见第 12 节） |
| `PP_ARC_EXT_MAX` | 24 | 弧长模式下远端点沿切线外推的上限 |
| `PP_DX_MIN` | 30 | 纯跟踪纵向距离下限，防止贴脸点打满舵 |
| `STEER_REF_ROW` | `VIEW_NEAR_ROW`(99) | 纵向距离的参考行；改回 120 会拉长前瞻、转角变柔 |
| `SPEED_LIMIT` | 400 | 目标速度上限（脉冲 / 5 ms） |
| `MOTOR_DUTY_LIMIT` | 10000 | 双轮 PID 输出上限（满量程） |
| `MOTOR_BRAKE_DUTY` | 1500 | 刹车段允许的最大反向占空比 |
| `BEND_FAR_AHEAD` / `BEND_FEED_DIV` | 12 / 3 | 远端趋势采样点偏移 / 前馈分母（定义于 `Motor.c`） |
| `BEND_SPEED_CAP` | 170 | 减速状态的硬顶 |
| `BEND_SLOW_START` / `BEND_SLOW_GAIN` / `BEND_SLOW_MIN` | 4 / 3 / 40 | 弯道减速曲线 |
| `BEND_STEER_PCT` / `BEND_STEER_MAX` | 110 / 185 | 弯道转向增益区间 |
| `BEND_GAIN_BEND` / `BEND_GAIN_RATE` | 15 / 6 | 增益起算强度 / 每单位百分点 |
| `BORDER_IN_SPEED_RATIO` | 40 | 元素内速度比例 |
| `SPEED_RAMP_MS` / `SPEED_RAMP_STEP` | 1200 / 2 | 发车起步斜坡：每 1200 ms 提速一档，最多 2 档 |
| `SERVO_TEST_SWING` / `SERVO_TEST_STEP` | 495 / 33 | 自检摆幅与步长（495 = 33×15，三段都精确落在端点） |
| `MOTOR_DIR_FORWARD` | 0 | 低电平前进 |
| `MOTOR_KICK_TICKS` | 0 | 起跑助推关闭 |
| `MOTOR_OPEN_LOOP` / `LOW_SPEED_TEST` | 0 / 0 | 闭环、比赛模式 |
| `EDGE_DIFF` / `EDGE_RATIO` | 12 / 7 | 边界梯度双条件 |
| `BODY_ROW` / `HIDE_COL_L` / `HIDE_COL_R` | 80 / 48 / 140 | 车身遮挡行与起点搜索窄列范围 |
| `XC_MAX_PTS` / `XC_WEAK_PTS` / `XC_MAX_CL` | 6 / 10 / 10 | 十字形环岛入口点数门槛（定义于 `XCross.c`） |
| `XC_SPEED_RATIO` / `XC_MAX_MS` | 60 / 800 | 十字形环岛速度比例 / 阶段超时（定义于 `XCross.c`） |
| `SC_WINDOW` / `SC_SLOPE_MIN` / `SC_P2P_MIN` | 12 / 3 / 8 | S 弯斜率窗口 / 斜率下限 / 峰谷差（定义于 `SCurve.c`） |
| `SC_SPEED_RATIO` / `SC_MAX_MS` | 70 / 3000 | S 弯速度比例 / 超时（定义于 `SCurve.c`） |
| `RD_OUTER_MIN` / `RD_FAR_ROW` / `RD_INNER_MAX` | 20 / 42 / 7 | 环岛严格路径：外侧点数下限、末点行上限、内侧点数上限（定义于 `Round.c`） |
| `RD_GEOM_OUTER_MIN` / `RD_GEOM_FAR_ROW` / `RD_GEOM_INNER_MAX` | 16 / 46 / 10 | 环岛几何路径门槛（定义于 `Round.c`） |
| `RD_WIDTH_EXPAND_PX` / `RD_TURN_PROM` | 8 / 5 | 环岛几何路径：宽度扩张量 / 内侧局部极值的凸起度 |
| `RD_SCORE_NEED` / `RD_READY_MS` | 6 / 1800 | 环岛入口积分阈值 / READY 误判兜底（定义于 `Round.c`） |
| `RD_SPEED_RATIO` / `RD_STAGE_MS` / `RD_LOST_MS` | 50 / 8000 / 1000 | 环岛速度比例 / 单阶段超时 / 全丢线强制退出（定义于 `Round.c`） |
| `VIEW_CUT_FAR` / `VIEW_CUT_NEAR` | 34 / 20 | 视野窗口上、下裁掉的行数 |
| `VIEW_CUT_LEFT` / `VIEW_CUT_RIGHT` | 34 / 34 | 视野窗口左、右裁掉的列数 |
| `VIEW_FAR_ROW` / `VIEW_NEAR_ROW` | 34 / 99 | 由上面四项推导 |
| `VIEW_COL_L` / `VIEW_COL_R` | 34 / 153 | 由上面四项推导 |
| `OUT_BLIND_MS` / `OUT_CONFIRM_FRAMES` | 200 / 3 | 出界盲区时间与确认帧数 |
| `OUT_START_GRACE_MS` | 300 | 发车宽限 |
| `OUT_ROW_NEAR` | 76 | 出界采样点近端行（`BODY_ROW - 4`） |
| `ELEM_TOTAL_MS` | 20000 | 元素全会话超时（`main.c`）；单阶段超时由各元素模块自管，环岛为 `RD_STAGE_MS=8000` |
| `BE_BEND_MIN` / `BE_BEND_EXIT` | 10 / 4 | 入弯强度进入 / 退出阈值 |
| `BE_BLANK_MS` / `BE_MAX_LOST_MS` | 250 / 900 | 连接处完全免判时长 / 最长容忍 |
| `BE_MAX_APPROACH_MS` | 400 | 预备入弯的超时 |
| `PULSE_PER_METER` | 34600 | 每米脉冲数，必须推车 1 m 复核 |
| `LAP_DISTANCE_CM` / `STOP_OFFSET_CM` / `BRAKE_DISTANCE_CM` | 1200 / 100 / 200 | 圈长 / 停车点 / 提前刹车距离 |

## 10. 串口协议

默认不输出周期性数据。停车后按 B4 短按，才上传黑匣子全部记录：

```
LOG t,MODE,STG,CAND,RS,L,R,C,DUTY,SPD,DS,DARK,H0
LOG t,MODE,STG,CAND,RS,L,R,C,DUTY,SPD,DS,DARK,H1
...
ENDDDDD
```

格式串为 `"LOG %u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%u,H%u\r\n"`。`t` 为相对发车时刻的毫秒，`DUTY` 为舵机偏移（已从 1/4 压缩还原）。最后一行固定为 `ENDDDDD`，之后不再发送任何内容。

| 字段 | 含义 |
|---|---|
| `MODE` | 114 循迹、314 S 弯、514 十字形环岛、810 左环岛、1919 右环岛 |
| `STG` | 元素内部阶段，环岛 1~5 |
| `CAND` | 环岛候选位：bit0~3 右环、bit4~7 左环（`rdCandBits()`） |
| `RS` | 环岛积分，达到 `RD_SCORE_NEED=6` 触发 |
| `L` / `R` | 左右边界原始点数（拐点裁短前，截到 255） |
| `C` | 中线点数（截到 255） |
| `SPD` / `DS` | 实际 / 目标速度 |
| `DARK` | 出界采样点暗点数 0~5 |
| `H0` / `H1` | 末字段前缀 `H` + 数值：`H1` = 环岛状态机激活（`elemActive==ELEM_ROUND`），`H0` = 未激活 |

整行控制在 64 字节以内：USB CDC 端点包只有 64 字节，超长会被拆包，调试助手按包显示时会丢掉前半行（含 `MODE`/`STG`/`CAND`/`RS`）。因此黑匣子**不含 `ERR` 字段**；`faultFlags` 仍记录在 RAM 结构体 `elemLog_t.err` 中，需要时用 `ENABLE_LIVE_TELEMETRY` 的 `ERR` 字段查看。

`ENABLE_LIVE_TELEMETRY = 1`（默认 0）时改为周期性调试行（待发车每 250 ms、跑车中每 1 s），格式串为：

```
ODO=%ld L=%d R=%d C=%d AIM=%d THR=%d DARK=%d DROP=%u MODE=%d STG=%d CAND=%d RS=%d AL=%d STA=%d ERR=%d SPD=%d DS=%d DUTY=%d
```

| 字段 | 含义 |
|---|---|
| `ODO` | 累计里程脉冲 |
| `L` / `R` | 左右边界点数（拐点裁短前） |
| `C` | 中线点数（融合后） |
| `AIM` | 本帧前瞻点所在行（对比两种取法用） |
| `THR` | 当前全局阈值 |
| `DARK` | 最近一帧的暗点数（0~5），直道正常行驶应为 0 |
| `DROP` | 被丢弃的帧数，异常增多说明 CPU/XRAM 到瓶颈 |
| `MODE` | 114 循迹、314 S 弯、514 十字形环岛、810 左环岛、1919 右环岛 |
| `STG` / `CAND` / `RS` | 元素内部阶段 / 环岛候选位 / 环岛积分 |
| `AL` | 88 左侧外推、99 右侧外推 |
| `STA` | `trackIsOn()`，赛道是否可跑 |
| `ERR` | 故障位，见下表 |
| `SPD` / `DS` | 实际速度 / 目标速度 |
| `DUTY` | 舵机偏移 |

| 位 | 值 | 含义 |
|---|---|---|
| `FAULT_STOPPED` | 0x0001 | 因异常被强制紧急停车 |
| `FAULT_ENC_L_DOWN` | 0x0002 | 左编码器无反馈 |
| `FAULT_ENC_R_DOWN` | 0x0004 | 右编码器无反馈 |
| `FAULT_PID_SAT` | 0x0100 | 输出饱和且速度上不去 |
| `FAULT_PID_REV` | 0x0200 | 正输出配反向计数 |
| `FAULT_OUT` | 0x0400 | 出界保护触发 |

## 11. 离线校验

`tests/` 下每个脚本独立可跑（纯 Python，无第三方依赖），数值实现与固件逐行对应：

| 脚本 | 校验内容 |
|---|---|
| `numeric_check.py` | `atan2_int` / `sqrt_int` / `tani` 表值、曲率限速曲线、舵机换算与饱和点 |
| `aim_arc_check.py` | 前瞻两种取法（`PP_ARC_MODE`）的前瞻距离、`dx`、转角、duty 对比 |
| `bend_slowdown_check.py` | 弯道减速曲线的分档与保持逻辑 |
| `bend_entry_check.py` | 入弯状态机迁移 |
| `bend_recognition_check.py` | 中线融合与弯道识别 |
| `out_of_track_check.py` | 出界判据的暗点 / 盲区 / 确认帧组合 |
| `body_row_check.py` | `STEER_REF_ROW` 与车身遮挡行的几何 |
| `track_bias_check.py` | 横向偏置对瞄准点的影响 |
| `roundabout_geom_check.py` | 环岛几何路径判据（`rdWidthExpand` / `rdInnerTurn` / `rdGeomCand`）的离线复现 |

**当前实测状态**：9 个脚本中 8 个通过；`aim_arc_check.py` 在 GBK 控制台下因输出 `²` 字符抛 `UnicodeEncodeError`（脚本编码问题，非逻辑错误）。部分脚本常量仍停留在早期标定（`body_row_check.py` 的俯角、`bend_slowdown_check.py` 的 `BEND_SPEED_CAP=170` 等已与当前源码同步，但注释中的旧值需留意）。

## 12. version-1.4 变更

1. **环岛入口判据**：去掉“内侧必须带拐点”的前提。`LRoadCount` 在 `loadRoad()` 里
   会被裁成 `LCorner`（取值只有 0 或 ≥3），再要求 `LCorner` 非零，就把“内侧线整条消失”
   这个主要入口工况排除掉了，实际只剩“拐点恰落在第 3 点”能进。
2. **出界判据**：“看到赛道”改用裁短前的 `LRawCount` / `RRawCount`，避免十字与环岛期间
   `LRoadCount` 恒 ≤3 导致盲区计时一直成立。
3. **前瞻取法可选**：新增 `PP_ARC_MODE`。旧取法为给远端点留 12 个点，前瞻距离被永久
   扣掉 24 像素，中线越短缩得越狠，同一横向偏差解出的打角可差 5 倍以上；弧长模式按
   累计弧长取点、线短时沿末端切线外推，默认仍为 0（保持现车标定）。
   新增 `PP_LOOKAHEAD_PX` 作为等效轴距的唯一调参入口——原来 `motorInit` 第 7 参数恒传 0，
   注释里的调参建议没有入口。
4. **舵机自检**：新增 `SERVO_TEST_STEP = 33`（495 = 33×15）。原步长 25 除不尽 495，
   三段摆动分别停在 +475 / -480 / -20，收尾离中位还差 20 duty。
5. **数值清理**：`tani` 表缩到 31 项（`atan2_int` 上限 ±30，原 31/32 两档永远取不到）；
   删除 `alongLine` / `bendEntryCalc` 中数学上不可达的 `ax <= 0` 分支；`getAimPos` 上界
   改为 `SPEED_LIMIT/40`（原 `c3+20` 需 800 脉冲/5 ms 才顶得到，恒不生效）。
6. **十字补线起点**：兜底改用本次扫描的 `min` / `max`。原来用 `minMaxH`，单调边界下它
   恒为 0，起点会落到画面最左侧的裁切暗区，爬线在暗区里爬出几十个假点。
7. **可观测性**：串口新增 `AIM` 字段；新增 `tests/aim_arc_check.py`；`numeric_check.py`
   同步 `STEER_REF_ROW = 99` 与 31 项 `tani` 表。
8. **交付**：`out_file/SEEKFREE.hex` 与 `.map` 随版本入库，Keil 全量重建 0 Error。

## 13. 当前状态

已在实车确认：

- 编码器方向与电机方向匹配，`MOTOR_DIR_FORWARD = 0` 对应前进；
- 双缓冲 16 位字搬运，有效帧率约 96 fps；
- 直道行走与起步正常；`DARK` 在直道正常行驶时为 0；
- `version-1.2` 为已实车验证的稳定基线。

仍需现场验证：

- 弯道以推头冲出为主，偶发能过；
- 三个元素（S 弯 / 环岛 / 十字形环岛）的入口阈值都只是初标（`RD_OUTER_MIN` / `RD_GEOM_*` / `XC_*` / `SC_*`，定义在各元素 `.c` 文件中），需要用串口 `MODE` / `STG` / `CAND` / `RS` 或黑匣子观察实际迁移；
- 定点停车依赖 `lapPulse` 实推标定，`PULSE_PER_METER = 34600` 需推车 1 m 复核 `ODO`；
- `PP_ARC_MODE = 1` 尚未上车验证，切换后同一弯道打角会变小，需要重标 `c1` 或 `PP_LOOKAHEAD_PX`；
- 发车起步斜坡（`SPEED_RAMP_MS=1200`、`SPEED_RAMP_STEP=2`）需实车确认。

逐项改动审查见 `REVIEW.md`；描述当前实现时以本 `README.md` 与源码为准。

## 14. 许可与致谢

- 本项目整体以 **GNU General Public License v3.0** 分发，全文见 `LICENSE`。
- 第三方组件与相对上游的修改清单见 `NOTICE`。
- 感谢成都逐飞科技提供 STC32G144K 开源库与硬件支持。
- 感谢浙江工商大学哈夫克自动智能车项目提供的原始工程结构（见 `NOTICE`）。
