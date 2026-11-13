# Racing Car C-model

> **作者：aMaz1ng**
> 本项目基于 **浙江工商大学 哈夫克自动智能车项目** 进行二次开发（非官方分支，仅供学习与竞赛参考）。
> 底层驱动来自 [逐飞科技 STC32G144K 开源库](https://seekfree.taobao.com/)（GPL-3.0）。

> **版本状态：version1.2 —— 已实车验证可用**
> 本版在 **version1.1** 基础上完成精简（删除爬线中永不执行的 `delicate` 分支与配套 1 像素方向表、
> 清理未参与运算的变量、8 组整定参数改为 `speedSet[8][4]` 表驱动），并已**上车实测通过**：
> 摄像头初始化、边界与中线提取、舵机打角、左右电机闭环、双编码器里程均工作正常，可正常发车与停车。
>
> 实测确认的硬件配置（换板时需同步修改）：
>
> | 项目 | 实测值 | 代码位置 |
> |---|---|---|
> | 三个独立按键 | **PB2 / PB3 / PB4**（原工程为 P02/P01/P32） | `main.c` 三处 `gpio_init` 与三处 `gpio_get_level` |
> | 舵机信号 | **PA4**，200 Hz，中值 2975 | `Motor.c` `steerMid` / `pwm_init(PWME_CH3P_PA4, ...)` |
> | 电机 PWM/DIR | **P76/P77（右）**、**P74/P75（左）**，17 kHz | `Motor.c` 顶部宏 |
> | 编码器 | **P60/P62（右）**、**P40/P42（左）** | `Motor.c` `encoder_quad_init` |

---

## 1. 项目简介

面向 **全国大学生智能汽车竞赛（摄像头组/竞速组）** 的 STC32G144K246 平台完整竞速程序。

- **主控**：STC32G144K246（C251 内核，Keil MDK FOR C251，主频 96 MHz）
- **传感器**：逐飞总钻风 MT9V03X 灰度摄像头（188 × 120，DMA + FIFO 采集）
- **显示**：IPS200 彩色屏
- **执行**：舵机（PWME_CH3P_PA4）+ 双直流电机（PWMB 驱动）+ 双路编码器（PWMA / PWMC）

核心思路：**灰度图爬边线 → 透视去畸变 → 中线外推 → 纯跟踪（Pure Pursuit）解算转向角 → 按转弯半径限速 → 双电机增量式 PID**。

## 2. 功能清单

| 模块 | 说明 | 代码位置 |
|---|---|---|
| 图像巡线 | 八邻域/四邻域爬线，局部动态阈值 + 全局阈值自更新 | `project/code/Image.c` |
| 透视去畸变 | 45 KB 离线标定查找表，把斜视画面映射到"上帝视角" | `reflection[120][188][2]` |
| 中线外推 | 依左右边界外推中线，拐角处切换补线偏移 | `loadRoad()` / `getCenterLine*` |
| 纯跟踪控制 | 由前瞻点解算转弯半径，前馈打角 | `Motor.c` `alongLine()` |
| 速度控制 | 弯道 `v=√(R·c4)` 限速 + 双电机增量式 PID + 差速 | `Motor.c` `pitHandler()` |
| 十字 | 元素状态机，进出判据 + 强制打点 | `project/code/Cross.c` |
| 左右环岛 | 各 5 个阶段的状态机，左右对称 | `LRound.c` / `RRound.c` |
| 里程停车 | 编码器里程积分，一圈自动刹车停车 | `project/code/Stop.c` |
| 出界保护 | 5 点采样全黑或两侧边界全丢 → 立即停车 | `Motor.c` `pitHandler()` |
| 图像双缓冲 | 采集缓冲与算法快照分离，消除 170 fps 下的撕裂 | `Image.c` / `isr_extended_0.c` |

> 说明：当前版本**不含**斑马线与障碍块（避障）处理，赛道元素为「循迹 + 十字 + 左右环岛 + 定点停车」。2026-11-05 的核心功能审查与修复同样**没有引入任何避障逻辑**。

## 3. 目录结构

```
version1.0/
├── README.md
├── LICENSE                          # GPL-3.0 全文
├── NOTICE                           # 上游来源与修改说明
├── .gitignore                       # 排除 out_file 等编译产物
├── libraries/                       # 逐飞开源库（GPL-3.0，请勿随意删减）
│   ├── LICENSE                      # GPL-3.0 全文（库文件头引用此文件）
│   ├── doc/                         # 版本说明与 GPL 许可声明
│   ├── zf_common/                   # 时钟、中断、数学、字体、typedef
│   ├── zf_components/               # seekfree_assistant 等组件
│   ├── zf_device/                   # 摄像头、IPS200、无线串口、IMU 等设备驱动
│   └── zf_driver/                   # GPIO/EXTI/PWM/PIT/UART/SPI/ADC/编码器
└── project/
    ├── code/                        # 应用层算法
    │   ├── Image.c/.h               # 图像处理、爬线、去畸变、双缓冲快照
    │   ├── Motor.c/.h               # 电机/舵机/编码器、PID、纯跟踪、出界保护
    │   ├── Stop.c/.h                # 里程计、发车/一圈标定/停车状态机
    │   ├── Cross.c/.h               # 十字
    │   ├── LRound.c/.h              # 左环岛
    │   └── RRound.c/.h              # 右环岛
    ├── user/                        # 程序入口与中断
    │   ├── main.c/.h                # 主流程、参数档位、发车主循环
    │   ├── isr.c/.h                 # 向量号 0~31 的基础中断
    │   ├── isr_extended_0.c         # P0~P9、DMA UART1~4、DMA_LCM、TM11
    │   ├── isr_extended_1.c         # PA/PB、DMA UART5~8、TM5~TM18
    │   └── isr_extended_vectors.A51 # 扩展中断的真实向量跳转表
    └── mdk/                         # Keil 工程
        ├── seekfree.uvproj          # 目标：stc32g144k246
        └── out_file/                # 编译产物（不入版本库）
```

## 4. 编译与烧录

1. 使用 **Keil MDK FOR C251**（已在 C251 V5.60.0 验证）打开 `project/mdk/seekfree.uvproj`。
2. 目标选择 `stc32g144k246`，`Rebuild`。
3. 产物：`project/mdk/out_file/SEEKFREE.hex`，用 STC-ISP 下载。

**修复前**的历史编译输出（本机 2026-11-05 未安装 Keil C251，无法重新编译，仅供参考）：

```
0 Error(s), 3 Warning(s)
Program Size: data=107.6  edata+hdata=8192  xdata=56294  const=50701  code=97140
```

3 条告警：`zf_device_mt9v03x.c(138) C183`（逐飞库内无效赋值）、`Image.c(147/176) C173`（`getLBorder` 里 `uint16 step` 与 `int16 *count` 混比）。本次审查已把 `step` 改为 `int16` 并加容量夹取，后两条 C173 应消失。**修改后必须在装有 Keil C251 的机器上 Rebuild 一次**，确认 0 Error 并重新生成 hex。

另外：工程里 `<DisableWarningNumbers>57,16,7,25</DisableWarningNumbers>` 屏蔽了 4 类链接器告警、链接器参数带 `REMOVEUNUSED,NOOVERLAY`（关闭调用树分析）；`<Cpu> CLOCK(35000000)` 是仿真时钟，与实际 `clock_init(SYSTEM_CLOCK_96M)` 不一致（只影响 uVision 仿真/波特率计算，不影响烧录后运行）。

清理中间产物可直接运行 `project/mdk/MDK删除临时文件.bat`。

## 5. 两个必须了解的实现细节

### 5.1 C251 只支持 0~31 号中断

芯片的扩展中断（`P0~PB`、`DMA UART`、`DMA_LCM`、`TM5~TM18`）真实向量号范围是 **37~126**，本工程实现了其中 37~99 区间的 **26 个**；C251 的 `interrupt N` 属性无法直接使用（`ERROR C168: value not in range 0 ... 31`）。因此：

- `isr.c`：只放向量号 ≤ 31 的基础中断，由编译器自动生成向量表；
- `isr_extended_0.c` / `isr_extended_1.c`：文件头加 `#pragma noiv`，函数用 **0~31 的局部编号**声明；
- `isr_extended_vectors.A51`：在真实向量地址（`CSEG AT 012BH` … `031BH`）上放 `EJMP` 跳转。

> 改动中断逻辑时注意：扩展中断的 handler 在 `isr_extended_*.c` 里，**不要**写回 `isr.c`；这两个文件里的编号只是占位，真实入口由 A51 决定。26 个 `EJMP` 目标符号、地址、C 函数名已逐条核对无错配；其余向量没有默认陷阱，**禁止使能未实现的扩展中断**。

### 5.2 图像双缓冲（稳定优先）

```
摄像头 ──DMA──► mt9v03x_image（采集缓冲，库内固定地址）
                      │  DMA 完成中断（LCM_DMA，优先级 1）
                      │  mt9v03x_dma_handler() → mt9v03x_frame_snapshot()
                      │  memcpy 22.5 KB，并置 frameBusy = 1
                      ▼
              mt9v03x_frame（稳定快照，所有算法只读这里）
                      │  主循环以 frameBusy 为门控
                      ▼
          loadRoad() → 元素状态机 → 控制量 → frameBusy = 0
```

- 算法层**一律读 `mt9v03x_frame`**，不要直接读 `mt9v03x_image`（那是 DMA 正在写的缓冲）。
- **搬运在 DMA 完成中断内完成，保证帧原子性**：`frameBusy` 为 1 时 ISR 直接跳过搬运（丢弃来不及处理的新帧），不会出现半帧混入。这是可预期的"丢帧降级"，而不是概率性的错误输入。
- **控制中断的确定性靠优先级保证，不靠把拷贝挪出 ISR**：库默认 `INT1`(VSYNC)=3、`LCM_DMA`=3，`TIM1` 未设置=0（最低）。`main.c` 在摄像头初始化后调用 `interrupt_set_priority(TIMER1_IRQn, 3)` 与 `interrupt_set_priority(LCM_DMA_IRQn, 1)`，5 ms 的电机 PID 中断可抢占 22.5 KB 拷贝。
- **按键扫描与 `raceTask()` 在帧门控之外**，摄像头停帧时急停/停车判定依然生效；主循环另有约 1 s 的帧看门狗，比赛状态长时间收不到新帧会调用 `raceStopNow()` 安全停车。
- 该方案替代了原来的 `outputImage` 调试缓冲（22.5 KB）。若需要恢复"二值化图/边界叠加"预览，必须把新增缓冲放到 `far` 区，否则会触发 `L107 ADDRESS SPACE OVERFLOW`（16 位 XDATA 只有 64 KB）。

## 6. 操作方式

| 操作 | 功能 |
|---|---|
| 拨码 SW1/SW2（PB0/PB1） | 选择 4 组 × 2 个共 8 组预置参数（按键 TOP / BOTTOM 对应同组内两套） |
| TOP 键（**PB2**，板上丝印 B2） | 发车（完赛或急停后也可重新发车，会复位元素状态机与 PID 历史）；选档阶段选奇数档 |
| BOTTOM 键（**PB3**，板上丝印 B3） | 推车跑完一圈到停车点后按下，标定一圈里程（门槛为默认圈长的一半，防止误按）；选档阶段选偶数档。**未标定前按 TOP 不会发车**，屏幕提示 `CALIB LAP FIRST` |
| IMAGE 键（**PB4**，板上丝印 B4） | 比赛中立即停车（不受相机帧率影响） |

> 按键引脚为实车实测值：原工程（浙江工商大学那套板）是 P02/P01/P32，本车实测为 **PB2/PB3/PB4**（用扫描固件逐个确认）。PB0/PB1 是拨码开关、PB6/PB7 是摄像头 SCCB，互不冲突。若换板，只需改 `main.c` 里 `gpio_init(IO_PB2/3/4, ...)` 与三处 `gpio_get_level()`。

上电后会先用一帧原始灰度图确认摄像头正常，再显示待发车信息（`ODO / LAP / THR`）。

### 6.1 视觉单测模式（`VISION_ONLY`）

`project/user/main.c` 顶部的开关：

```c
#define VISION_ONLY  1   // 1 = 只测图像；0 = 正常竞速（上车前必须改回 0）
```

置 1 时，摄像头初始化成功后会直接进入预览循环，**不会调用 `motorInit()`**，因此不启动 5 ms 控制中断、不初始化 PWM/编码器、不驱动电机与舵机——只需要「主板 + 摄像头 + 屏幕」即可调试视觉，适合装车前单独验证图像链路。

预览界面内容：

| 位置 | 内容 |
|---|---|
| 0~119 行 | 实时原始灰度图 |
| 画面内 | 蓝点 = 左边界、红点 = 右边界、绿点 = 中线（`loadRoad()` 的真实输出） |
| 计数区 | `L` / `R` 边界点数、`C` 中线点数、`THR` 全局阈值、`LC` / `RC` 拐点、`FRAME` 已处理帧数 |

调参对应关系：`L`/`R` 长期为 0 → 降 `EDGE_DIFF`、放宽 `EDGE_RATIO`；边界跑偏 → 重设 `HIDE_ROW/HIDE_COL_L/HIDE_COL_R`；`LC`/`RC` 恒为 0 → 降 `CORNER_COS100`；中线整体偏一侧 → 调 `centerLineDistance`。

> 预览刷新率受 IPS200 的 SPI 带宽限制（约 10~20 fps），以 `FRAME` 递增为准。**装车后务必把 `VISION_ONLY` 改回 0**，否则程序会停在预览循环、无法进入选档与发车。

## 7. 主要可调参数

| 参数 | 位置 | 说明 |
|---|---|---|
| 舵机中值 / 舵机频率 | `Motor.c` `steerMid` / `pwm_init(..., 200, ...)` | 200 Hz 对应中值 2975；改 50 Hz 时中值需改为 750 |
| 等效轴距 | `Motor.c` `DEVICE_H`（默认 18，可由 `motorInit()` 第 5 个参数覆盖） | 纯跟踪用，影响整体打角幅度 |
| 转向/速度 PID | `Motor.c` `PIDInit(...)` | 转向 `Kp=c1, Kd=50`；速度 `3000/50/50` |
| 差速系数 | `Motor.c` `setSpeed * 8 / R` | 弯道内轮打滑就调小 |
| 8 组预置参数 | `main.c` 的 `speedSet[8][4]` 表 | 每行依次为：舵机增益 c1、前瞻点序号 c3、弯道限速系数 c4、直道限速 c5 |
| 元素开关 | `main.c` `ENABLE_CROSS` / `ENABLE_ROUND` | 先关掉跑通纯循迹再逐个打开 |
| 元素阈值 | `main.c` `ROUND_ROW / ROUND_NEAR_X / ROUND_FAR_MAX / CROSS_MAX` | 按画面点数标定 |
| 图像阈值与边缘 | `Image.h` `block/cap/EDGE_DIFF/EDGE_RATIO/HIDE_ROW/CORNER_COS100` | 蓝底白赛道现场标定 |
| 里程标定 | `Stop.h` `PULSE_PER_METER / LAP_DISTANCE_CM / STOP_OFFSET_CM / BRAKE_DISTANCE_CM` | 必须实测，`PULSE_PER_METER` 用推车 1 m 标；`BRAKE_DISTANCE_CM` 稳定优先取 200（原 150） |
| 速度天花板 | `Motor.h` `SAFE_SPEED_CEILING`（默认 400） | 稳定优先：8 档 `c5` 中超过该值的会被压到 400；需要提速只改这一个宏 |
| 帧看门狗 | `main.c` `noFrameTicks > 1000`（1 ms 节拍，约 1 s） | 比赛状态连续无新帧则调用 `raceStopNow()` 安全停车 |

> 注意：十字、环岛、停车等判据中的点数阈值都是针对特定赛道量化出来的，**换场地基本都需要重新标定**。

### 7.1 视觉标定记录（2026-11-07 · 台架实测）

台架条件：白纸赛道（A4 拼贴）+ 棕色硬纸板背景；摄像头高度 **12~15 cm**、俯视 **30°~45°**、镜头压在赛道中线上。

**合格判据（`VISION_ONLY=1` 屏幕上直接读）**

| 读数 | 合格范围 | 不合格时怎么处理 |
|---|---|---|
| `THR` | **140~160** | 读到 `128` = 采样区没有明暗差（镜头对着纯色/太暗），**先调安装角度，不要动参数** |
| `L` / `R` | **60~90**，两者差 < 20 | < 20：视野被白纸占满（摄像头太低/太近），或 `EDGE_DIFF` 过大 |
| `C` | **60~100** | 为 0：两侧边界都没出来；远小于 L/R：被拐点提前截断 |
| `LC` / `RC` | 直道 0，分叉处跳成非 0 | 恒为 0：边界太短（< 15 点）或 `CORNER_COS100` 偏严（可降到 40） |
| `FPS` | 空跑 ~171、满载 150~170 | 明显低于 100：先设 `VISION_PREVIEW=0` 排除刷屏干扰 |

**实测结果**

| 场景 | 读数 | 结论 |
|---|---|---|
| 直道（标定成功） | `L=86  R=72  C=100  THR=151` | 边界贴合两侧、中线居中 |
| T 形分叉 | `LC=15  RC=28` | 两侧边界被拐点截断、`C` 缩短，与真赛道的十字/分叉同构 |
| 镜头对着桌面 | `THR=128  L=R=C=0` | 采样区无明暗差，图像不可用 |
| 摄像头太低 | `THR=152  L=37  R=40` | 白纸占满画面，两侧背景被挤出视野 |

**三个必须记住的坑**

1. **`THR=128` 是"对比度不足"的兜底值**，出现它就说明画面不能用；不要试图用 `EDGE_DIFF` 去救。
2. **桌面裸板调试时 `HIDE_ROW` 要临时改 119**（等于不遮挡）。否则第 90 行以下、第 48~140 列会被当成车身，近处真实赛道被排除，起点搜索直接失败（表现为 `L=R=C=0`）。**装车后必须改回 90。**
3. **只有一侧能看到背景时**（另一侧赛道撑满画面），算法会把"赛道外沿"当成"内沿"，中线被推到赛道外。这时必须调安装，调参数无效。
>
> **2026-11-05 修复后必须重新标定速度**：本项目自定义的整数开方函数 `sqrt_int()`（二次开发时追加在 `libraries/zf_common/zf_common_function.c`，逐飞上游库并不提供该函数）原来对 `x > 1048576` 的输入返回偏小值，而弯道限速 `√(R·c4)` 的乘积可达 5.3×10⁶。修复后**弯道**限速值变大（如 5,292,000 从 2047 变为 2300）；直道仍由各档 `c5` 封顶，现再由 `SAFE_SPEED_CEILING` 统一兜住上限。`c4`、各档 `c5` 以及 PID 参数仍需按实车重新调。

### 7.2 异常状态检测（`faultFlags`）

`Motor.h` 只保留 **PID 异常** 与 **紧急停车** 两类，运行时自动置位，屏幕在状态切换处显示 `ERR:<十进制>`。

| 位 | 值 | 含义 | 判定位置 | 是否停车 |
|---|---|---|---|---|
| `FAULT_PID_SAT` | 0x0100 | PID 输出打满 ≥98% **且**速度不到目标一半，持续 1 s（堵转 / 欠压 / 轮子卡住） | `Motor.c` 5 ms 中断 | **是** |
| `FAULT_PID_REV` | 0x0200 | PID 输出与编码器计数方向相反持续 250 ms（速度环正反馈，飞车前兆） | 同上 | **是** |
| `FAULT_STOPPED` | 0x0001 | 因上述异常被强制紧急停车（同时驱动蜂鸣器响 150 ms） | 上述异常 | — |

**使用约定**

- 发车（TOP）时调用 `faultReset()` 清零；**停车时保留**，方便赛后回看是哪种异常导致的停。
- 有 **1 秒宽限期**（发车后 PID 起转、齿轮间隙需要时间），避免起步误判。
- 出现异常时，除紧急停车外还会驱动蜂鸣器响 **150 ms**（`IO_P90`）。蜂鸣器电平由 `Motor.h` 的 `BEEP_ON_LEVEL / BEEP_OFF_LEVEL` 决定，若报警无声或长鸣，对调这两个宏即可。
- 检测逻辑只读 `pidOutL/R`、`encoderL/R`、`setSpeed`、`currentSpeed`，**不改变任何控制量**；唯一的动作是异常时置 `stop` 并切到 `RACE_STOPPED`。
- 屏幕 `ERR` 只在状态或故障位变化时刷新一次，不占运行时间。
- 帧看门狗（约 1 s 无新帧）仍会紧急停车，但不再置故障位；出界保护同理（5 点全暗连续 3 拍），`ERR` 保持 0。

## 8. 已知限制

1. **参数强耦合于单条赛道**：元素判据以像素点数为单位，换赛道/换摄像头安装高度需重标。
2. **整帧拷贝仍在 DMA 完成中断内，靠优先级抢占、而非搬移来保控制周期**：`mt9v03x_frame_snapshot()` 每帧 `memcpy` 22.5 KB，估算 1~3 ms。TIM1 已提到优先级 3、`LCM_DMA` 降到 1，控制中断可以抢占拷贝；但 **C251 的高优先级中断能否抢占正在执行的 ISR，尚未在实机/仿真中验证**，若硬件行为与预期不符，5 ms 控制周期仍会被推迟 1~3 ms，必须用 IO 翻转实测。拷贝被抢占而拉长时，`frameBusy` 会自然丢弃下一帧（降级而非出错），有效帧率会低于 170 fps，需实测掉帧率与循迹稳定性。彻底方案仍是 DMA 双缓冲乒乓：需逐飞 `.LIB` 支持运行时切换目标地址，且要再腾出 22.5 KB XRAM，而当前 XDATA 余量约 14 KB 仍不足（裁剪 8 个未引用驱动后从 9.2 KB 提升到 14 KB）。
3. **去畸变查找表不可现场调整**：`reflection[120][188][2]` 是离线标定产物，摄像头位置或分辨率变化后必须重新生成。
4. **仓库状态**：已纳入 Git 管理（`main` 分支），`.gitignore` 排除 `project/mdk/out_file/` 等编译产物；`LICENSE`、`NOTICE`、`libraries/LICENSE`、`libraries/doc/GPL3_permission_statement.txt` 已按 GPL-3.0 要求补齐。
5. **逐飞库内仍有与 `version.txt` 声明不符的残留问题**（当前配置未触发，故未擅自改第三方 GPL 库）：`zf_driver_encoder.c` 正交编码器清零时"关计数器→写 0→开计数器"两行被注释（V3.1.1/3.1.2 声称已修）、`TIM6/8/10_ENCODER` 清零恢复写成 `|= (1<<3)`（应为 bit7）、`encoder_n & 0x0F + PWMF_ENCODER + 1` 缺括号；`zf_common_interrupt.c` 的 `IP/IPH` 分支用 `|=` 只能调高不能调低、`P8~PB` 分支移位溢出（V3.2.4 声称已修）。**已联网核对 Gitee 上游 `master`：最新版本就是 V3.2.5（2026-06-27），这几个文件与本地逐字一致**，说明是上游现存问题、没有更新版本可升级。本工程只用 PWMA/PWMC 正交编码器，且库内调用 `interrupt_set_priority(INT1_IRQn, 3)`（置位方向，可用）。2026-11-05 应用层在 `main.c` 新增两次调用：`TIMER1_IRQn, 3` 走 `IP/IPH` 置位方向（0→3，可用），`LCM_DMA_IRQn, 1` 走 `DMA_LCM_CFG` 的先清后置分支（可双向设置），**均不受上述 `|=` 缺陷影响**；若以后启用 TIM 编码器或动态调整其它优先级，需等上游修复或自行打补丁。
6. **摄像头 DMA 是预编译库**：`zf_device_mt9v03x_dma.LIB` 无源码，`version.txt` 中的"修复摄像头花屏""采集优化到 3 ms"无法从源码核实。
7. **元素状态机无超时兜底**：十字/环岛各阶段靠边界计数判据退出，若现场阈值不合适可能长时间停在某阶段；`raceStart()` 会强制复位，但比赛途中没有自动超时回 `FORWARD`。调参时优先用 `ENABLE_CROSS/ENABLE_ROUND` 逐个验证。
8. **摄像头 DMA 内部参数与上游不一致（未决项）**：本地 `zf_device_mt9v03x.c` 调 `mt9v03x_dma_init(..., 0, 3, 3)`，上游 V3.2.5 是 `..., 2, 3, 3)`，而头文件示例写 `4`。第三个参数 `value` 被官方注释为"内部参数，用户无需关心"，实现只在 `.LIB` 里、无源码，无法判断 0/2/4 的语义，因此**未擅自改回 2**。若实车出现花屏、丢帧或图像错位，优先对比试上游值 `2`（同时确认本地 `.LIB` 与上游版本一致）。

## 9. 致谢与许可

- 原始项目：**浙江工商大学 哈夫克自动智能车项目**，本仓库在其基础上完成算法重构、双缓冲改造与工程化整理。
- 底层驱动：[逐飞科技 STC32G144K 开源库](https://seekfree.taobao.com/)，遵循 **GPL-3.0**。
- 二次开发：**aMaz1ng**
- 核心功能修复：PR #1（**TBssjq**，`257224f`），见第 10 节。

本仓库整体以 **GPL-3.0** 分发。`libraries/` 下被修改的源文件有两处：`zf_common/zf_common_function.c` 追加了本项目自定义的 `sqrt_int()` / `atan2_int()` 整数实现（逐飞上游库并不提供），2026-11-05 由 PR #1 修正其 16 位截断与起始位问题，并由 GBK 转存为 UTF-8；`zf_common/zf_common_headfile.h` 第 111~116 行追加了 `Cross.h` / `Image.h` / `LRound.h` / `RRound.h` / `Motor.h` / `Stop.h` 六个应用层头文件引用。按 GPL-3.0 第 5(a) 条，修改内容、日期与责任方记录于此及 `NOTICE`。

| 文件 | 内容 |
|---|---|
| `LICENSE` | GPL-3.0 许可证全文 |
| `libraries/LICENSE` | 同上（每个逐飞库文件头引用此文件） |
| `libraries/doc/GPL3_permission_statement.txt` | 逐飞库要求的英文许可声明 |
| `NOTICE` | 上游来源、修改范围与再分发义务 |

再分发请保留全部版权声明与上述文件，并注明所做修改。

## 10. 核心功能审查与修复记录（2026-11-05）

范围：只审查/修复**核心功能**（图像巡线、透视去畸变、中线外推、纯跟踪、速度环、十字/环岛、里程停车、双缓冲、中断向量表），**不加入避障或任何新赛道元素**。仓库内可参考的开发文档只有本 README 与 `libraries/doc/version.txt`（逐飞库 V3.2.5 版本说明），已逐条核对；另核对了芯片头文件的中断向量地址、`zf_driver_*` 的引脚分配与 `zf_common_function.c` 的数学函数实现，并通过互联网核对了 STC 官方资料与逐飞 Gitee 上游仓库（见 10.1）。

| 类别 | 问题 | 位置 |
|---|---|---|
| 越界写 | 十字补线 `corner = func_min(corner+3, count)` 可返回 100，写 `farCenterLine[100]` | `Cross.c` |
| 越界读 | 十字补线起点 `Point pos` 未初始化（直道 y 单调时必现）+ `getImagePointFilterSelf` 无边界检查 | `Cross.c` / `Image.c` |
| 越界/脏数据 | `resampleRoad` 负 startPos 或超容量 count、`reflection` 表索引、`centerLine[i+1]` 与首点残留、`func_limit_ab(cx,0,199)` 上限错误 | `Image.c` / `main.c` |
| 16 位溢出 | 三处 `dx*dx+dy*dy`；`atan2_int` 内部 `(int)` 强转；`sqrt_int` 起始位只有 `1L<<20`；PID `int16 base` 先溢出再限幅；编码器取反/倍频/两轮求和 | `Image.c` / `Motor.c` / `zf_common_function.c` |
| 类型 | `findCorner` 的 `int8 m` 存不下 `100*abs_dot`（"最尖拐点"分支永不生效）；`getLBorder` 的 `uint16 step` 混比（C173） | `Image.c` |
| 状态机 | 出界保护只置 `stop` 不同步 `raceState`（无法再发车）；发车不清元素/PID 状态；`RRound` 清掉刹车锁 `handle` | `Motor.c` / `RRound.c` / `Stop.c` / `main.c` |
| 里程停车 | 低速判据在刹车段起点误停；`setSpeed=200` 覆盖制动目标；`odometer` 32 位撕裂读；一圈标定门槛仅 200 脉冲 | `Motor.c` / `Stop.c` |
| 逻辑 | `findBorderIn` 跨左右分支残留导致 `borderIn` 误判；`banUpdateThreshold` 被同帧爬线函数清掉；补线首点沿用上一帧；滤波 `(a>>1)+(b>>1)` 少 1 像素 | `main.c` / `Image.c` |
| 可靠性 | 按键与 `raceTask()` 被 `frameBusy` 门控（相机停帧时无法急停）；开机首帧是选档期间的旧图 | `main.c` |

已完成的验证：

- `sqrt_int` 修复用等价算法做数值验证：x ≤ 1048576 时新旧结果完全一致（1/100/10000/1000000 → 1/10/100/1000）；x = 5,292,000 时旧值 2047、新值 2300（真值 2300.43）。
- 中断向量表 26 个 `EJMP` 目标符号、真实向量地址、C handler 逐条核对无错配；`#pragma noiv` 与 HUGE 模式符号修饰正确。
- 引脚在 `USER_USB_CDC=1` 下无冲突（P37/P36 由摄像头 FIFO/DMA 占用，UART1 未启用；IPS200 的 MISO/CS 未初始化，P82 作 BLK 安全）。
- 提取修复后的 `sqrt_int`/`atan2_int` 用 gcc `-Wall -Wextra` 真编译并运行，0 warning，数值符合预期。

### 10.1 联网核对的官方资料（2026-11-05）

| 来源 | 核对结论 |
|---|---|
| STC 官方 `STC32G144K246` 芯片页 / 数据手册下载页 | 246K Flash、49+ 中断源、4 级中断优先级、100~120 MHz、6 组高级 PWM 支持正交解码；本工程 96 MHz 是库的合法档位，不是降频 bug |
| STC 官方《关于中断号大于 31 在 Keil 中编译出错的处理》+ 国芯论坛 STC32G 学习贴 | C251 只支持中断号 0~31，扩展中断需汇编跳转或官方中断向量扩展工具 → 本工程 `isr_extended_vectors.A51` 的 `EJMP` 方案与官方思路一致；优先级 0 最低 / 3 最高，高可打断低、同级按序响应 |
| 逐飞 Gitee 上游 `seekfree/STC32G144K246_100Pin_Library`（master） | `version.txt` 最新即 V3.2.5（2026-06-27），与本地一致；`zf_driver_encoder.c`、`zf_common_interrupt.c`、`zf_device_mt9v03x.c` 与本地逐字一致，确认第 8 节第 2/5 条是上游现存行为、无新版可升级 |
| 逐飞上游 `zf_common_function.c/.h` | **不含** `sqrt_int`/`atan2_int` → 这两个函数是本项目二次开发追加的自定义实现，修改它们不改变第三方库行为 |
| 国芯论坛 / DeepWiki 的 STC32G 中断系统说明 | `IP(0xB8)/IPH(0xB7)/IP2(0xB5)/IP2H(0xB6)` 两个位组合成 0~3 级，0 最低 3 最高（网上有 AI 生成回答称"0 最高"且把该芯片误称为 Ai8052U，不可采信） |

联网核对**没有推翻已做的修复**，但新增一个未决项：`mt9v03x_dma_init` 第三个参数本地 `0` vs 上游 `2`（见第 8 节第 8 条）。

**未完成的验证**：Keil C251 实编译（本机无该工具链）、实车标定与路试、DMA 内部参数 0/2 的实画面对比。烧录前请按第 4 节重新 Rebuild，并按第 7 节提示重新标定速度参数。

## 11. 整合记录：PR #1 与本地并行修复（2026-11-05）

PR #1 与另一条并行开发分支基于同一份评估报告各自做了修复，整合时逐文件取舍如下：

| 文件 | 采用 | 说明 |
|---|---|---|
| `Cross.c/.h`、`Motor.c/.h`、`Stop.c/.h`、`LRound.c/.h`、`RRound.c/.h`、`Image.c`、`main.c` | PR #1 | 覆盖越界、16 位溢出、状态机/PID 复位、里程撕裂读、按键移出帧门控，比本地版完整 |
| `zf_common_function.c` | PR #1 | `sqrt_int` 起始位 `1L<<20`→`1L<<30`、`atan2_int` 去掉 `(int)` 截断 |
| `Image.h`、`isr_extended_0.c` | 本地 | 删除 `frameBusy` 与 ISR 内的整帧搬运 |
| `main.c`（双缓冲部分） | 本地 | 改用 `mt9v03x_finish_flag` 触发、主循环搬运；同时保留 PR 的按键 / `raceTask()` 门外执行、`system_delay_ms(2)` 去抖、`elementReset()` 与 `loadRoad()` 边界修复 |
| `main.c` 全局变量 | 本地 | `LRoadCount` / `RRoadCount` / `borderIn` 统一 `volatile`（`Motor.c` / `LRound.c` / `RRound.c` 的 extern 同步修改），PR #1 未覆盖此项 |
| `main.c` 摄像头初始化 | 本地 | `mt9v03x_init()` 无限重试改为最多 10 次 + 屏幕提示 |
| `main.c` 文件头 | 修正 | 还原被误写为 `G5NU` 的 `GNU General Public License` 字样 |
| `README.md` | 合并 | 保留第 10 节审查记录，补齐许可证清单、修正第 8.4 条过时的 Git 描述，并按实际方案重写第 5.2 / 8.2 节 |

**双缓冲取舍**：采用「搬运移出 ISR」而非 PR #1 保留的「ISR 内搬运 + `frameBusy` 丢帧」。理由是 TIM1 控制中断优先级为最低 0、无法抢占最高优先级 3 的 DMA_LCM，把 22.5 KB 拷贝留在 ISR 里会确定性地推迟电机控制；移到主循环后控制中断可随时抢占，代价是概率性的混帧窗口（见 8.2）。两者都需实车/示波器验证，此处选择优先保证控制周期的确定性。

**注意**：以上整合仅完成静态审读与一致性检查，未在 Keil C251 上编译，也未实车验证；`sqrt_int` 修复后必须按第 7 节重新标定速度参数。

> **后续变更**：本节「搬运移出 ISR」的双缓冲取舍已于同日被第 12 节的稳定优先调整取代 —— 恢复 ISR 内搬运 + `frameBusy` 保证帧原子性，并通过中断优先级调整同时保住控制周期。

## 12. 稳定优先调整（2026-11-05）

明确的取舍原则：**优先保证稳定运行，其次才是速度**。据此重新审视第 11 节的双缓冲方案并做如下调整。

| 项 | 调整前 | 调整后 | 稳定收益 |
|---|---|---|---|
| 帧搬运位置 | 主循环，用 `mt9v03x_finish_flag` 触发 | DMA 完成中断内，以 `frameBusy` 门控 | 消除半帧混入；来不及处理时明确丢帧，而不是读到错误赛道 |
| 控制中断 | TIM1 默认最低 0，无法抢占 DMA | `TIMER1_IRQn = 3`、`LCM_DMA_IRQn = 1` | 5 ms PID 可抢占 22.5 KB 拷贝，帧原子性与控制确定性不再二选一 |
| 速度 | 8 档 `c5` 最高 520；修复 `sqrt_int` 后弯道限速变大 | 新增 `SAFE_SPEED_CEILING 400` | 只压 1/7/8 三档（520/450/500→400），其余 5 档不动；提速只改一个宏 |
| 刹车 | 提前量 150 cm | 提前量 200 cm | 多留 50 cm 制动余量，降低冲出停车区概率 |
| 掉帧 | 无兜底，保持上一拍打角继续跑 | 约 1 s 无新帧 → `raceStopNow()` | 摄像头 / DMA 异常时主动安全停车 |

用户同日对 `Stop.c` 的改动一并保留：刹车脉冲按 `lapPulse × BRAKE_DISTANCE_CM / (LAP_DISTANCE_CM + STOP_OFFSET_CM)` 比例换算，并带参数填错时的 `lapPulse >> 3` 兜底。按当前默认值，`lapPulse = 13000`、`brakePulse = 2000`，刹车段从约 11.0 m 开始（起跑线 12 m）。

**为什么翻转第 11 节的决定**：原方案为换取控制周期确定性而容忍概率性混帧，且混帧窗口恰好落在起点搜索（第 118 行）与阈值采样（92~113 行）所在的关键区——错误输入比可测的时序抖动更危险。而 `LCM_DMA_IRQn` 的优先级寄存器 `DMA_LCM_CFG` 可双向设置、`TIMER1_IRQn` 的 `IP/IPH` 可置位到 3，硬件上允许同时拿到帧原子性与控制抢占，因此原本的"二选一"并不成立。

**验证状态**：以上均为静态审读 + 一致性检查，**未在 Keil C251 实编译、未实车验证**。烧录前必须：① Rebuild；② 用 IO 翻转确认 C251 高优先级中断确实能抢占 DMA ISR，并实测 5 ms 控制周期抖动与有效帧率；③ 按第 7 节重新标定速度与刹车参数。若现场出现频繁丢帧，优先调低 `SAFE_SPEED_CEILING` 或削减帧内计算量，**不要回退帧原子性**。
