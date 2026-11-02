# 阶段1 计划书：视觉模型训练（手部关键点蒸馏 → INT8 → 上板）

> 前置：`docs/阶段0-视觉接板-POSE1-实施方案.md`（阶段0 已跑通，2026-11-01）
> 终态：`docs/参考-板端控制链路速查-rk7100_20261031.md`（板端协议与版本标记）
> 存档基线：`archive/20261101_stage0_v2/`

---

## 1. 目标与边界

### 1.1 本阶段要做什么

把 PC 上的 MediaPipe 手部检测**蒸馏成一个可上板的轻量模型**，输出形态**完全复刻 MediaPipe 的 21 个关键点**，从而在板上替换掉 MediaPipe 而**不改动下游任何一行代码**。

三步递进：

| 步 | 产物 | 运行位置 |
|---|---|---|
| 1 | `sw/vision_nn.py` + ONNX 模型 | PC（ONNX Runtime），与 MediaPipe A/B 对比 |
| 2 | INT8 量化后的 xmodel | PL（Vitis AI DPU）或纯 PS |
| 3 | 板端端到端：摄像头 → 模型 → Geometry → POSE1 → 逆解核 | 板子独立跑 |

### 1.2 本阶段明确不做

- ❌ 不改逆解核（`xsolver`/`lm_solver`）——它已验证与固件等价，属数学而非模型。
- ❌ 不改板端固件（`arm_control.c` / `pose_uart.inc` 等）。
- ❌ 不做端到端"手部图像 → 关节角"的黑箱模型（见 §3 决策 A）。
- ❌ 不动 `sw/pose_map.py` 的映射/标定逻辑。
- ❌ 第一期不做摄像头进 PL（MIPI/VDMA）——先用 DPU 从 DDR 取图。

---

## 2. 为什么要训练（三个动机，按紧迫度）

1. **端到端延迟**：PC 侧 `landmarker detect_for_video` 约 **18.9 ms/帧**，是当前 29.5 fps 里最大的单项（读帧 7.9 / 检测 18.9 / 左面板 1.2~2.1 / IK 0.15~2.15 / 渲染 2.3~2.7 / 显示 2.2~2.5 ms）。换小网可以把这一项压到 5 ms 以内。
2. **板上没有 MediaPipe**：MediaPipe 是桌面级框架（TFLite + C++ 图调度），Zynq7100 裸机（FreeRTOS/裸跑）跑不动，必须换。
3. **可控性**：自训模型可以针对"单手握拳/张掌、距离 90~130 px、固定机位"这个窄域做优化，精度可超过通用模型的该场景表现，且能直接输出 handedness 与存在概率。

---

## 3. 关键决策：输出形态选 A（复刻 21 关键点）

| | **方案 A（推荐）** | 方案 B |
|---|---|---|
| 输出 | 21×2 归一化坐标 + presence + handedness | 直接回归 5 通道（u/v/手长比/theta/伸指数） |
| 下游改动 | **零**（`Geometry.compute` 与 `PoseMapper` 原样用） | 全部重写：标定、死区、slew、AutoZero 全部失效 |
| 可解释性 | 可在画面上画骨架，现场排障靠肉眼 | 黑箱，出问题无法定位 |
| 离散量 | 伸指数由几何算出，天然稳定 | `n_ext` 是整数，回归难、易抖 |
| 数据复用 | 复用 MediaPipe 这个"教师"，无需人工标注 | 同样可蒸馏，但标签是 5 通道，难查错 |
| 模型规模 | 稍大（需要空间精度） | 更小 |

**结论：走 A。** 理由与阶段0 的架构认知一致——视觉侧只是"遥控器"，输出的应是可解释的手部几何，而非关节量。

---

## 4. 数据管线（本阶段最大的风险，占工期 60%）

### 4.1 现状盘点

| 资产 | 规格 | 能否直接用于训练 |
|---|---|---|
| `results/w3_pose/calib_rec.mp4` | 1280×720 / 30 fps / 7687 帧 / 4.3 min | ⚠️ 有画面，**无逐帧关键点标签** |
| `results/w3_pose/demo_walk.mp4` | 640×480 / 160 帧 | 太短，仅作 sanity check |
| `results/w3_pose/pose_run_*.jsonl`（16 个，38 MB） | 几何量、POS E1、gesture | ❌ **不含 21 点原始坐标**，且无帧索引 |
| `results/w3_pose/arm_run_*.jsonl`（20+ 个） | 真机逐帧 POSE1 / raw / size_ratio | ❌ 同上，无画面 |

**结论：现有数据无法直接训练。必须建立"视频 + 同步标签"的采集与标注流水线。**

### 4.2 采集规范（机位固定，与真机一致）

| 项 | 要求 | 理由 |
|---|---|---|
| 分辨率 | **640×480**（与 `--width/--height` 一致） | 训练/推理同分布，避免缩放偏差 |
| 帧率 | 30 fps | 与现网一致 |
| 手长 | **90~130 px**（`MIN_HAND_PX=0.08` 是 51 px 下限） | 与真机操作距离一致，太小会误检 |
| 机位 | 正前方略俯 15°，手在画面中下部 | 与现在 USB 相机装法一致 |
| 光照 | 顶光 + 侧光各录一组；避免逆光 | 泛化 |
| 背景 | 桌面（现状）、白墙、深色布 各一组 | 防过拟合背景 |
| 手 | 左手 / 右手 都要 | handedness 分支 |
| 服饰 | 短袖 / 长袖 / 戴手套 各一组 | 袖口影响 wrist 关键点 |
| 动作 | 平移（左右/上下/前后）、绕腕旋转 ±90°、握拳-张掌循环、快速抖动 | 覆盖 5 通道全量程 |
| 单段时长 | 60~120 s，文件不要过大 | 便于标注与划分 |

**规模目标**：

- M1（PC 基线）：**1.5 万帧**（现有 7687 + 再录 ~30 min）
- M2（正式训练）：**4 万帧**（约 2 小时素材），覆盖 ≥8 种光照/背景/服饰组合

### 4.3 自动标注流水线（MediaPipe 当教师）

关键点：**用 IMAGE 模式逐帧独立推理**（而非 VIDEO 模式），避免跟踪漂移把错误标签带进数据集；然后用时序一致性过滤。

```
video.mp4
  → 逐帧 imdecode
  → mediapipe HandLandmarker(IMAGE, num_hands=1)
  → 21 点 (x,y) ∈ [0,1] + handedness + score
  → 过滤：score ≥ 0.8 且 hand_px ≥ 0.08 且 与前后帧最大位移 ≤ 0.15（剔除跳变）
  → 存 <video>_labels.npz  (N×23 float32 + N 布尔 mask)
```

- 预计吞吐：PC 单进程 IMAGE 模式约 60~80 fps → 2 小时素材约 25~35 分钟标完，可接受。
- 丢弃率预期 10~20%（低置信/小目标/跳变帧），**保留丢弃清单**用于反查教师缺陷。

### 4.4 数据划分

- **按视频划分**，不按帧划分（相邻帧高度相关，按帧分会造成严重泄漏）。
- 建议 8 : 1 : 1（训练 / 验证 / 测试），并且**测试集必须包含一个完全没见过的背景**，用来暴露过拟合。

### 4.5 增强

| 增强 | 参数 | 覆盖的现实情况 |
|---|---|---|
| 亮度/伽马 | ±25% | 开灯/关灯 |
| 色彩抖动 | HSV ±10 | 肤色差 |
| 随机平移缩放 | ±10% / 0.85~1.15 | 手距变化（对应 `size_ratio` 通道） |
| 轻微旋转 | ±15° | 手腕姿态 |
| 运动模糊 | kernel 3~5 | 快速挥动 |
| 左右翻转 | 概率 0.5，**同时翻转 handedness 标签** | 左右手泛化 |

> ⚠️ 翻转必须改 handedness 标签，否则会让模型学会错误的手别；这是本项目容易踩的坑。

---

## 5. 模型设计

### 5.1 结构

```
输入 256×256×3 (uint8 → float/127.5-1)
  └ MobileNetV3-Small 前 4 段（width mult 1.0，去掉最后 stage）
      └ 全局池化 + 2 层 FC
          ├─ 21×2 关键点（归一化到 [0,1]，相对输入图）
          ├─ presence（1 logit）
          └─ handedness（2 logits）
```

| 指标 | 目标 |
|---|---|
| 参数量 | ≤ 1.5 M |
| MACs @256² | ≤ 300 M |
| PC 单帧（ONNX Runtime，CPU，4 线程） | ≤ 5 ms |
| 板上单帧（DPU INT8 @125 MHz） | ≤ 10 ms（对应 4.17M cycles/帧预算的 1/4 左右） |

> 这个规模对 8 GB 显存是小菜（见 §6.4），因此 **有余量往上试**：若 M1 精度不够，可先抬输入到 320²，再考虑换 MobileNetV3-Large 或小的混合 backbone，只有当板上资源（§7.2）卡住时才必须回退。

### 5.2 输入策略：全图 vs ROI

- **M1 先做全图 256×256**（最简单，一步到位）。
- 若 M1 的 NME 不达标（见 §6.3），再升级为 **两阶段**：粗定位手部 ROI（64×64 分类/回归）→ ROI 放大到 256² 精回归。两阶段可以在 DPU 上串起来，但会翻倍延迟，只在必要时做。

### 5.3 关键点表示

- **直接回归坐标**（不做 heatmap）：INT8 量化友好、后处理零成本、可用 wing loss 解决"大误差"问题。
- heatmap 精度更高但需要 soft-argmax 后处理、量化时 argmax 对量化噪声敏感，本期不用。

---

## 6. 训练

### 6.1 损失

```
L = 5.0 · Wing(Δkpt, w=10, ε=2)          # 21 点，仅 mask 为真的帧
  + 1.0 · BCE(presence)
  + 0.5 · CE(handedness)
  + 0.1 · 骨长一致性正则（可选，抑制关键点相对漂移）
```

`Wing loss` 对小误差线性、对大误差对数，正好适合关键点。

### 6.2 训练配置

| 项 | 值 |
|---|---|
| 优化器 | AdamW，lr 1e-3，wd 1e-4 |
| 调度 | cosine，warmup 5 epoch |
| batch | 64 |
| epoch | 100（早停 patience 15） |
| 精度 | AMP（fp16） |
| 硬件 | **本地 RTX 5060 Laptop GPU 8 GB**（实测 8151 MiB / 驱动 610.74，见 §6.4） |
| 吞吐（估） | 约 **200~500 张/s**（256², batch 128, AMP fp16）**待 M0 实测标定** |
| 单轮耗时（4 万帧） | 约 **2~5 min** → 100 epoch ≈ 3~8 h；建议先跑 40 epoch 看曲线 |

### 6.3 验证指标（三层，缺一不可）

| 层 | 指标 | 门限 |
|---|---|---|
| 关键点 | NME（按手掌尺度归一化的平均点误差） | **≤ 0.020**（≈ 手长的 2%） |
| 关键点 | PCK@0.05 / PCK@0.10 | ≥ 90% / ≥ 98% |
| **任务层（最重要）** | 与 MediaPipe 在同段视频上的 5 通道偏差：u/v/手长比/theta/伸指数 | **u,v ≤ 0.02；手长比 ≤ 5%；theta ≤ 3°；伸指数完全一致率 ≥ 95%** |
| 稳定性 | 单帧抖动（相邻帧输出的标准差） | 不高于教师模型的 1.5 倍 |

> 只看 NME 是不够的：模型可能点都"准"，但 `n_ext`（伸指数）在阈值附近反复跳变，导致夹爪乱开合。**任务层指标才是验收依据。**

### 6.4 本机训练环境：RTX 5060 8 GB 的具体预算

> **实测硬件**（2026-11-01 只读查询 `nvidia-smi`）：
> `NVIDIA GeForce RTX 5060 Laptop GPU` / **8151 MiB** / 驱动 **610.74**
> → 驱动足够新，**直接支持 cu128 轮子**，不需要升级驱动；注意是 **Laptop 版**（功耗受限、长时间满载会降频），长训练要盯 `nvidia-smi -q -d CLOCK`。

#### 6.4.1 独立环境（不与运行时混用）

训练必须**另建 conda 环境**，不能装进 `hand-pi`（`hand-pi` 是现网运行时，mediapipe 1.1.1 + opencv 5.0 已验证稳定，动它就等于把已验证的真机链路置于风险中）。

```bash
# 环境名必须以 -pi 结尾（项目规矩）
D:/Anaconda3/_conda.exe create -n train-pi python=3.10 -y
D:/Anaconda3/envs/train-pi/python.exe -m pip install   torch torchvision --index-url https://download.pytorch.org/whl/cu128
D:/Anaconda3/envs/train-pi/python.exe -m pip install   numpy opencv-python onnx onnxruntime-gpu tqdm tensorboard
```

| 环境 | 用途 | 关键包 |
|---|---|---|
| `hand-pi` | **运行时**（pose2arm / 仿真 / 自动打标） | mediapipe 1.1.1、opencv-contrib 5.0.0、numpy 2.2.6、pyserial |
| `train-pi` | **训练与导出**（本阶段新增） | torch ≥ 2.7 (cu128)、onnxruntime-gpu |

#### 6.4.2 ⚠️ 5060 是 Blackwell（sm_120），PyTorch 有版本硬门槛

RTX 50 系列计算能力是 **sm_120**，老版本 PyTorch 的预编译核心里没有它，会报：

```
CUDA error: no kernel image is available for execution on the device
```

**装完必须先验证**（一行，装错立刻能发现）：

```bash
D:/Anaconda3/envs/train-pi/python.exe -c  "import torch;print(torch.__version__, torch.version.cuda, torch.cuda.get_device_capability())"
# 期望看到: 2.7.x 12.8 (12, 0)
```

- `torch.__version__` 至少 **2.7**，`torch.version.cuda` 必须 **12.8 及以上**；
- 驱动需 **570 以上**（`nvidia-smi` 可见 CUDA Version ≥ 12.8）；
- 若版本不符，换 `--index-url .../whl/cu128` 或更新的 nightly，**不要用 conda 默认频道的老 torch**。

#### 6.4.3 8 GB 显存预算表

| 项 | 占用 | 说明 |
|---|---|---|
| 模型权重 | ~6 MB | MobileNetV3-Small 1.2 M 参数 × fp32 |
| 优化器状态（AdamW） | ~12 MB | 2× 权重 |
| 激活值（batch 64 @256²，AMP） | ~0.5~0.8 GB | 最大项，随 batch 线性增长 |
| cuDNN workspace | ~0.3~0.5 GB | 卷积算法选择 |
| **合计** | **≈ 1.2~1.5 GB** | 8 GB **余量很大** |

**余量可以这么用**（按优先级）：

1. batch 64 → **128**（更稳的 BN 统计，收敛更快）；
2. 输入 256² → **320²**（小目标关键点精度提升明显，对应真机手长 90~130 px 的场景）；
3. backbone → MobileNetV3-Large；
4. 开 `torch.compile`（Blackwell 支持良好，通常 +20~40% 吞吐）。

**显存不够时的降级顺序（反过来用）**：batch 128→64→32 → 输入 320²→256²→224² → backbone Large→Small。

#### 6.4.4 吞吐与工期实测预估

按 MobileNetV3-Small、256²、batch 128、AMP fp16，**吞吐取 200~500 张/s（保守区间，M0 阶段实测标定后回填）**：

| 规模 | 单轮（估） | 40 epoch | 100 epoch |
|---|---|---|---|
| 7687 帧（sanity 模型，§12） | ~0.3~0.6 min | **~20~40 min** | ~1~2 h |
| 1.5 万帧（M1） | ~0.5~1.2 min | ~0.5~1 h | ~1.5~2.5 h |
| 4 万帧（M2 正式） | ~2~5 min | ~1.5~3.5 h | **3~8 h** |

> 计算侧不是瓶颈：MobileNetV3-Small 前反向仅约 0.25 GFLOPs/张，瓶颈在**数据加载**（640×480 JPEG 解码约 1000 张/s/核）。因此 §6.4.5 的 DataLoader 配置比调模型更重要。
>
> 关键结论：**sanity 模型半小时内可出结果**，"先验证路线再投素材"的策略成本极低；即使正式训练翻车，最多损失一两天。

#### 6.4.5 训练期间的注意事项

0. **移动版会降频**：5060 Laptop 长时间满载（>30 min）通常掉到 70~85% 频率，长训练的实际耗时按上表上限估；同一台机器不要边训练边跑真机，表现会互相拖累。
1. **不要与真机链路同时跑**：`pose2arm.py` 抢 GPU 无意义（它不用 GPU），但训练满载会让整机卡顿、误操作板子的风险上升。训练就专心训练。
2. **数据加载别成瓶颈**：4 万帧 × 640×480×3 ≈ 37 GB（解码后），**不能全塞内存**。用 `npz` 存已解码的 256² 或 JPEG 质量 95 的原图，`DataLoader(num_workers=6, pin_memory=True, persistent_workers=True)`，并把 dataset 用 `np.load(..., mmap_mode='r')`。
3. **必须开 checkpoint + resume**：17~33 h 的长任务，按 epoch 存 `last.pt` / `best.pt`，断点续训。
4. **先小后大**：先用 10% 数据跑 3 epoch，确认 loss 在降、验证集指标在动，再开全量长跑。
5. **固定随机种子 + 记录配置**：每个实验存 `config.yaml` + git 哈希（本项目非 git 仓库，用 `archive/` 快照号代替）。

---

## 7. 量化与上板

### 7.1 量化

1. **PTQ 先行**：500 帧校准集（覆盖不同光照），per-channel 对称量化，权重 int8、激活 int8。
2. 掉点门限：量化后 NME 相对浮点 **恶化 ≤ 10%**，且 5 通道任务层指标不回退超 20%。
3. 不达标再上 **QAT**（先量化感知训练 20 epoch，重点监控第一层与最后一层）。

### 7.2 路线选择

| 路线 | 优点 | 代价 | 结论 |
|---|---|---|---|
| **Vitis AI DPU** | 官方支持 Zynq 7000 + DPU IP，量化/编译工具链完整，PS 侧调用简单 | 需要 DPU IP 授权/资源，PL 资源占用较大（与 IK 核共存需评估） | **首选** |
| hls4ml | 全流水、延迟确定（可用流水线换吞吐） | 工具流重，模型结构受限（首层 conv 大 kernel 难做） | 备选 |
| 手写 HLS | 最省资源、最可控 | 开发量最大 | 只在资源实在不够时 |

**资源评估**：XC7Z100 有 2020 DSP / 277K LUT，IK 核只占一小部分（见 `boot/utilization` 报告）。DPU 小核（B1152/B1600）通常占 ~1000 LUT 级别 DSP 的一部分，**大概率装得下，但必须在 M3 用 Vivado 实测**，这是本阶段唯一可能推翻方案的技术风险。

### 7.3 摄像头通路

- 第一期：**摄像头仍在 PS 侧**（USB 或 VDMA 从 DDR 取图）→ DPU → 结果回 PS。不动 MIPI。
- 第二期（可选）：MIPI CSI → VDMA → DPU 硬通路，彻底去掉 PS 的拷贝开销。

---

## 8. 里程碑与验收门限

| 里程碑 | 内容 | 交付物 | 验收门限 | 估时 |
|---|---|---|---|---|
| **M0** | 采集与标注流水线 | `sw/collect_data.py`、`sw/label_auto.py` | 录 10 min 素材 → 自动标注 → 可视化抽查 100 帧无错标 | 1~2 天 |
| **M1** | PC 基线模型 | `models/hand_kpt_v1.onnx`、`sw/vision_nn.py` | NME ≤ 0.025；`--model-kind nn` 能在 `pose2arm.py` 里替换 MediaPipe 跑通，5 通道偏差达标（可放宽 2×） | **1~2 天**（单轮 <1 min，主要卡在写 `vision_nn.py` 与接口对齐） |
| **M2** | 数据扩量 + 正式训练 | 4 万帧数据集、v2 模型 | NME ≤ 0.020；任务层指标全达标；未见背景上不掉点 > 20% | **2~3 天**（其中录制 1 天，训练 3~8 h 可夜间跑） |
| **M3** | 量化 | `.xmodel` / INT8 ONNX | 掉点 ≤ 10%；DPU 侧单帧 ≤ 10 ms（仿真器） | 3~4 天 |
| **M4** | 上板 | 板端推理程序 | DPU + IK 核共存综合布线成功；板上实测 ≤ 15 ms/帧；端到端 ≥ 25 fps | 5~8 天 |
| **M5** | A/B 替换 | 真机对比报告 | 与 MediaPipe 版对比：跟随精度不下降，延迟下降 ≥ 30% | 2~3 天 |

**总计约 3~4 周**。训练用本地 5060 后，M1+M2 从 7~9 天压到 3~5 天；**工期重心已经从"训练"转移到 M3/M4 的量化与 PL 集成**（那块才是真正的不确定性，若资源不够需回退到 hls4ml 或把输入降到 192²）。

---

## 9. 风险与对策

| # | 风险 | 影响 | 对策 |
|---|---|---|---|
| 1 | 数据量不足 / 分布单一（只有一台相机、一个背景） | 模型只对当前机位有效，换个房间就崩 | 采集规范强制多背景多光照；测试集留一个未见背景 |
| 2 | 教师标签噪声（MediaPipe 本身在快速挥动时抖） | 学生学到坏习惯 | IMAGE 模式独立推理 + 时序一致性过滤 + 丢弃清单审查 |
| 3 | INT8 量化掉点（关键点回归对量化敏感） | 精度不达标 | PTQ→QAT；必要时关键点输出层保留 int16 或最后反量化在 PS 侧 |
| 4 | **PL 资源不够（DPU + IK 核共存）** | 方案推翻 | M3 阶段先用 Vivado 做资源预估；备选 hls4ml 或降分辨率 192² |
| 5 | 伸指数（夹爪）在阈值附近抖 | 夹爪反复开合 | 任务层指标纳入验收；必要时在 PS 侧加 3 帧投票（现有 `vote=3` 已有机制） |
| 6 | 板上推理与 POSE1 发送的时序竞争 | 看门狗冻结 | 复用现有 `keepalive` 自适应喂狗线程（已验证有效） |
| 7 | 摄像头通路带宽 | 掉帧 | 先用 640×480@30 YUYV（约 18 MB/s，USB2 够用） |

---

## 10. 与现有代码的接缝（保证可回退）

**新增文件，不改现有文件**（除一处开关）：

```
sw/vision_nn.py         # 新：VisionNN 类，接口与 mediapipe 的 detect_for_video 对齐
sw/collect_data.py      # 新：录制 + 同步存帧索引
sw/label_auto.py        # 新：MediaPipe IMAGE 模式批量打标
tools/train_kpt.py      # 新：训练脚本
tools/export_quant.py   # 新：ONNX → INT8 → xmodel
models/                 # 新：权重与导出产物
```

`vision_nn.py` 对外接口必须**返回与 mediapipe 相同结构的对象**：

```python
class VisionNN:
    def detect_for_video(self, bgr, ts_ms):
        # 返回对象需具备:
        #   .hand_landmarks[0] -> list[21] of objects with .x .y   (归一化 0~1)
        #   .handedness[0][0]  -> object with .category_name (str) .score (float)
        return res
```

`pose2arm.py` 只加一个开关：

```
--model-kind {mp,nn}   默认 mp（MediaPipe，现网行为不变）
--nn-model models/hand_kpt_v2.onnx
```

这样任何时候 `--model-kind mp` 都能回到今天已验证的状态（见 `archive/20261101_stage0_v2/`），**A/B 对比也只需换一个参数**。

---

## 11. 需要拍板的决策点

1. **采集谁来录？** 建议助手写好 `collect_data.py`（自动按规范分段、画面打提示"请做 XX 动作"），由你按提示录，共约 2 小时素材。
2. ~~**训练在哪跑？**~~ ✅ **已定（2026-11-01）：本地 RTX 5060 Laptop 8 GB（实测 8151 MiB / 驱动 610.74）**，另建 `train-pi` 环境，细节见 §6.4。前置检查只有一条：PyTorch 必须是 **cu128 及以上**（Blackwell sm_120 硬门槛，装错会报 `no kernel image is available`）。
3. **上板路线先按 Vitis AI DPU 走？** 需要确认 Vivado/Vitis AI 版本（板端工程是 Vivado 2024.2），以及 DPU IP 是否可用。
4. **第一期是否接受"摄像头仍走 PS"？** 建议接受，能把 M4 的风险压到最低。

---

## 12. 立即可以开始的三件事（无需拍板）

1. 写 `sw/label_auto.py`：把现有 `calib_rec.mp4`（7687 帧）自动打标，得到第一批可用数据集——**这一步今天就能做完**，用来验证整条流水线。
2. 用这 7687 帧训一个 **sanity 模型**（M0.5），确认"蒸馏路线可行、INT8 掉点可接受"，再决定要不要录 2 小时素材。**在 5060 上 40 轮只要 20~40 分钟**，几乎是零成本试错。
3. 在 `pose2arm.py` 里加 `--model-kind nn` 开关骨架，接口留空，先保证不影响现网。

> 建议顺序：**先做 1+2（1 天内出结论），再决定是否投入 2 小时录制**。这样最坏情况只浪费一天，而不是一周。
