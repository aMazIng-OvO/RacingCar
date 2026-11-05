# Racing Car C-model

> **作者：aMaz1ng**
> 本项目基于 **浙江工商大学 哈夫克自动智能车项目** 进行二次开发（非官方分支，仅供学习与竞赛参考）。
> 底层驱动来自 [逐飞科技 STC32G144K 开源库](https://seekfree.taobao.com/)（GPL-3.0）。

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

> 说明：当前版本**不含**斑马线与障碍块处理，赛道元素为「循迹 + 十字 + 左右环岛 + 定点停车」。

## 3. 目录结构

```
version1.0/
├── README.md
├── .gitignore                       # 排除 out_file 等编译产物
├── libraries/                       # 逐飞开源库（GPL-3.0，请勿随意删减）
│   ├── doc/                         # 版本说明与 GPL 协议
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

当前编译状态：

```
0 Error(s), 3 Warning(s)
Program Size: data=107.6  edata+hdata=8192  xdata=56294  const=50701  code=97140
```

3 条告警均为历史遗留，不影响目标生成：
`zf_device_mt9v03x.c(138) C183`（逐飞库内无效赋值）、`Image.c(147/176) C173`（`int16` 与 `uint8` 比较）。

清理中间产物可直接运行 `project/mdk/MDK删除临时文件.bat`。

## 5. 两个必须了解的实现细节

### 5.1 C251 只支持 0~31 号中断

芯片的扩展中断（`P0~PB`、`DMA UART`、`DMA_LCM`、`TM5~TM18`）向量号是 37~99，C251 的 `interrupt N` 属性无法直接使用（`ERROR C168: value not in range 0 ... 31`）。因此：

- `isr.c`：只放向量号 ≤ 31 的基础中断，由编译器自动生成向量表；
- `isr_extended_0.c` / `isr_extended_1.c`：文件头加 `#pragma noiv`，函数用 **0~31 的局部编号**声明；
- `isr_extended_vectors.A51`：在真实向量地址（`CSEG AT 012BH` … `031BH`）上放 `EJMP` 跳转。

> 改动中断逻辑时注意：扩展中断的 handler 在 `isr_extended_*.c` 里，**不要**写回 `isr.c`；这两个文件里的编号只是占位，真实入口由 A51 决定。

### 5.2 图像双缓冲

```
摄像头 ──DMA──► mt9v03x_image（采集缓冲，库内固定地址）
                      │  DMA 完成中断
                      ▼
              mt9v03x_frame_snapshot()   ← frameBusy==0 时才搬运
                      ▼
              mt9v03x_frame（稳定快照，所有算法只读这里）
                      │  主循环处理完
                      ▼
                  frameBusy = 0
```

- 算法层**一律读 `mt9v03x_frame`**，不要直接读 `mt9v03x_image`（那是 DMA 正在写的缓冲）。
- 主循环的触发条件是 `frameBusy`，处理完置 0；`mt9v03x_finish_flag` 只在开机等第一帧时使用。
- 该方案替代了原来的 `outputImage` 调试缓冲（22.5 KB）。若需要恢复"二值化图/边界叠加"预览，必须把新增缓冲放到 `far` 区，否则会触发 `L107 ADDRESS SPACE OVERFLOW`（16 位 XDATA 只有 64 KB）。

## 6. 操作方式

| 操作 | 功能 |
|---|---|
| 拨码 SW1/SW2 | 选择 4 组 × 2 个共 8 组预置参数（按键 TOP / BOTTOM 对应同组内两套） |
| TOP 键 | 发车 |
| BOTTOM 键 | 跑完一圈到停车点后按下，标定一圈里程 |
| IMAGE 键 | 比赛中立即停车 |

上电后会先用一帧原始灰度图确认摄像头正常，再显示待发车信息（`ODO / LAP / THR`）。

## 7. 主要可调参数

| 参数 | 位置 | 说明 |
|---|---|---|
| 舵机中值 / 舵机频率 | `Motor.c` `steerMid` / `pwm_init(..., 200, ...)` | 200 Hz 对应中值 2975；改 50 Hz 时中值需改为 750 |
| 等效轴距 | `Motor.c` `DEVICE_H`（默认 18，可由第 7 个参数覆盖） | 纯跟踪用，影响整体打角幅度 |
| 转向/速度 PID | `Motor.c` `PIDInit(...)` | 转向 `Kp=c1, Kd=50`；速度 `3000/50/50` |
| 差速系数 | `Motor.c` `setSpeed * 8 / R` | 弯道内轮打滑就调小 |
| 8 组预置参数 | `main.c` 各档 `motorInit(...)` | 依次为：舵机增益、未用、前瞻点序号、弯道系数、直道限速、未用、等效轴距 |
| 元素开关 | `main.c` `ENABLE_CROSS` / `ENABLE_ROUND` | 先关掉跑通纯循迹再逐个打开 |
| 元素阈值 | `main.c` `ROUND_ROW / ROUND_NEAR_X / ROUND_FAR_MAX / CROSS_MAX` | 按画面点数标定 |
| 图像阈值与边缘 | `Image.h` `block/cap/EDGE_DIFF/EDGE_RATIO/HIDE_ROW/CORNER_COS100` | 蓝底白赛道现场标定 |
| 里程标定 | `Stop.h` `PULSE_PER_METER / LAP_DISTANCE_CM / STOP_OFFSET_CM / BRAKE_DISTANCE_CM` | 必须实测，`PULSE_PER_METER` 用推车 1 m 标 |

> 注意：十字、环岛、停车等判据中的点数阈值都是针对特定赛道量化出来的，**换场地基本都需要重新标定**。

## 8. 已知限制

1. **参数强耦合于单条赛道**：元素判据以像素点数为单位，换赛道/换摄像头安装高度需重标。
2. **快照搬运在中断中完成**：`mt9v03x_frame_snapshot()` 在 DMA 完成中断（优先级 3）里 `memcpy` 22.5 KB，估算 1~3 ms，期间 5 ms 电机控制中断会被推迟。若实车出现电机抖动，可把该调用移到主循环 `loadRoad()` 之前（代价是极端情况下快照底部少量行混入下一帧）。
3. **去畸变查找表不可现场调整**：`reflection[120][188][2]` 是离线标定产物，摄像头位置或分辨率变化后必须重新生成。
4. **无版本控制历史**：仓库未纳入 Git 管理，建议先 `git init`（`.gitignore` 已就绪）。

## 9. 致谢与许可

- 原始项目：**浙江工商大学 哈夫克自动智能车项目**，本仓库在其基础上完成算法重构、双缓冲改造与工程化整理。
- 底层驱动：[逐飞科技 STC32G144K 开源库](https://seekfree.taobao.com/)，遵循 **GPL-3.0**，二次分发请保留原有版权声明与 LICENSE。
- 二次开发：**aMaz1ng**
