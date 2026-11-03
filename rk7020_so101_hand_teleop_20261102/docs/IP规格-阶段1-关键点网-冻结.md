# IP 规格冻结 —— 阶段1 手部关键点网（v1.0，2026-11-03）

> **状态**：功能与定点规格冻结；**定点语义已用两条独立实现验证通过**
> （numpy 参考 vs onnxruntime int8：平均 0.32 LSB；C++ 功能模型 vs onnxruntime int8：平均 **0.87 LSB**）
> **冻结范围**：输入输出契约、量化参数、层表、MAC/存储预算。**不含** HLS 综合结果（尚未综合）。
> **产物**：`fpga/ip/kpt_net/`（功能模型 + 自动生成的层表/权重）

## 1. 模型来源

| 项 | 值 |
|---|---|
| 训练产物 | `results/stage1/self_run5_192`（部署机位 + 全姿态，留出块 NME 0.0238、theta 1.29°、ext 98.4%） |
| 量化产物（上板用） | `results/stage1/self_run5_192/int8/best_hm_int8.onnx`（QDQ int8，0.54 MB） |
| 精度代价 | fp32 0.0231 → int8 0.0238（+3%），详见 `results/stage1/self_run5_192/README.md` |
| 结构 | MobileNetV2 w0.75 cut14 + stride16 热图头；纯 ReLU6；全静态 shape |
| 规模 | **112.9M MACs**、0.34M 参数、int8 权重 **323.5 KB** |

## 2. 输入输出契约（部署侧必须一致）

### 输入
| 项 | 规格 |
|---|---|
| 张量 | 1×3×192×192，**BGR**，uint8 |
| 量化 | scale = 1/127.5、zp = 128 → **`x_uint8 = 像素值`（恒等！）** 实测验证 |
| 说明 | 因此 IP 输入端不需要任何归一化运算，RS232/VDMA 送进来的像素直接就是激活值 |
| ROI | 由前置模块按 **手部方块** 裁剪并 `INTER_AREA` 缩到 192×192（训练时同口径） |

### 输出
| 项 | 规格 | 反量化 | PS 侧处理 |
|---|---|---|---|
| `hm` | 21×12×12 uint8 | scale 0.059959、zp 181 | softmax over 144 cells + 质心 → 21 点（0..1 归一化） |
| `presence` | 1 uint8 | scale 0.118838、zp 122 | logit，sigmoid 后与 0.5 比 |
| `handed` | 2 uint8 | scale 0.073212、zp 125 | argmax |

热图解码放 PS（3024 个 exp，A9 @1GHz 约 90 µs，相对 33 ms 预算可忽略），
**不要**在 PL 里做 ReLU-质心近似解码（实测掉一半精度，见 `self_run4_192/int8_hm/README.md`）。

## 3. 定点规则（已验证）

```
acc_int32 = Σ_c (x_uint8[c] - x_zp) * w_int8[c]        # w 的 zp 恒为 0，scale 逐输出通道
acc_int32 += bias_int32                                # 图里已按 sx*sw 量化
y_uint8    = clamp( ((acc*mult + round) >> (31+shift)) + y_zp , 0, 255 )
             M = sx * sw_c / y_sy，线性拆成 (mult:int32, shift:0..15)
```

三条容易踩的坑（都已踩过并修正）：

1. **量化域零填充 = x_zp，不是 0**（否则边界一圈全错）。
2. **残差支路有各自的 zp/scale**：`out = ((a-a_zp)*sa + (b-b_zp)*sb)/sy + y_zp`，
   且 ONNX 里 Add 的 a 是跳跃支路、b 是 block 投影输出。
3. **ReLU6 由量化区间实现**：ReLU6 输出的张量是 zp=0、scale=max/255，饱和钳位即等于 ReLU6，
   不需要额外的反向钳位逻辑。

## 4. 层表

52 层（1 普通 3×3 + 26 逐点 + 13 逐通道 3×3 + 8 残差 + 1 GAP + 3 FC），
逐层参数**自动生成**在 `fpga/ip/kpt_net/data/kpt_layers.h`，权重/偏置/requant 系数在
`kpt_weights.bin`（380.9 KB = 权重 323.5 + bias 25.5 + mult 25.5 + shift 6.4）。

前 6 层（验证层表格式）：

| # | 类型 | cin→cout | stride | 输出 |
|---|---|---|---|---|
| 1 | conv3x3 | 3→24 | 2 | 24×96×96 |
| 2 | depthwise3x3 | 24→24 | 1 | 24×96×96 |
| 3 | pointwise | 24→16 | 1 | 16×96×96 |
| 4 | pointwise | 16→96 | 1 | 96×96×96 |
| 5 | depthwise3x3 | 96→96 | 2 | 96×48×48 |
| 6 | pointwise | 96→24 | 1 | 24×48×48 |

重新生成：`python -u tools/gen_ip_header.py --onnx results/stage1/self_run5_192/int8/best_hm_int8.onnx`

## 5. 计算与存储预算（125 MHz）

| 项 | 数 | 结论 |
|---|---|---|
| 总 MAC | 112.9M | 30fps(33.3ms) 需 **≥54 MAC/clk**；15fps 需 27 |
| MAC 阵列 NMAC=64 | 1.76M 周期 | **14.1 ms/帧**（占 30fps 预算的 42%），留足余量 |
| 权重 | 323.5 KB | 可全驻 BRAM（约 90 个 BRAM36），或按层从 DDR 流式读（9.7 MB/s@30fps） |
| 最大单层激活 | 96×96×96 = 864 KB | **不能整层放 BRAM** → V2 定为激活走 DDR（每帧 15-20 MB，30fps ~500 MB/s） |
| 片上存储 | 权重块 + 3 行窗口 | 逐点层 WT[64][cin] ≤ 27.6 KB；逐通道 WD[32][3][3] 288 B |
| DSP48E1 | 2020 片 | int8 可用 8×8 乘法打包，NMAC=64 约需 64 片（约 3%），与两个 IK 核共存无压力 |

## 6. 接口与模块划分（v1）

```
VDMA/相机 ──AXI4-Stream(192×192×3 u8)──> [kpt_net_top]
                                            ├─ im2col 行缓冲（首层 3×3 用）
                                            ├─ pw_engine  （1×1，NMAC 并行）
                                            ├─ dw_engine  （3×3 逐通道，行缓冲）
                                            ├─ requant    （int32 → uint8）
                                            ├─ skip FIFO  （残差）
                                            └─ gap/fc（PS 侧也可代劳）
AXI4-Lite: ctrl(启动/完成/帧号) + 输出寄存器/BRAM 窗口
PS: softmax+质心解码 → POSE1(x,y,z,roll,gripper,seq) → 逆解核
```

## 7. 验证计划（按顺序，不许跳）

| 级 | 内容 | 状态 |
|---|---|---|
| V0 | numpy 定点参考 vs onnxruntime int8 | ✅ 平均 0.32 LSB |
| V1 | C++ 功能模型 vs onnxruntime int8（热图 + presence + handed） | ✅ 平均 0.87 LSB，pres/hand ±1 |
| V2 | HLS 结构版（kpt_hls.cpp，NMAC=64 / NCH=32）vs 功能模型 | ✅ **逐位一致，最大差 0 LSB** |
| V2.5 | 去 std::vector / 去 double，激活改 m_axi DDR 指针 + ping-pong | ✅ 逐位一致（1.83 MB scratch） |
| V3 | HLS 综合 + 时序（125 MHz 收敛）+ 资源报告 | 待做（工具：Vivado HLS 2018.3，已装且已授权） |
| V4 | 板级：同一段录像的 30 帧，PL 输出与 PC int8 输出逐元素比对 | 待做 |
| V5 | 端到端：相机→POSE1→IK 实机验收 | 待做 |

## 8. 已知限制 / 待定

1. **HLS 尚未综合**：V2 只证明“怎么算”对（结构版与功能模型逐位一致），不证明时序/面积。
2. 激活目前走 DDR；行带合并是 phase-2 优化（激活读量降 3 倍），不是上板前提。
3. 预处理（ROI 方块裁剪 + `INTER_AREA` 192）算在 PS/前置模块，尚未定点对齐；
   与 cv2 的偏差需要单独量化（会影响热图峰值位置 → theta 抖动）。
4. 相机采集链路：仍按 640×480@30 YUYV（USB2 实测 480p 30fps 没问题，1080p 只有 5fps）。
5. 输入分辨率固定 192×192 —— **冻结后不可改**，改分辨率要重训模型并重新生成层表。
6. `hls.tcl` 已按 Vivado HLS 2018.3 + xc7z100ffg900-2 + 125 MHz 写好，**未运行过**。
   V2.5 已完成：`forward_hls` 里已无 `std::vector`/`double`，激活走外部 DDR scratch。
