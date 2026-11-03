---
name: tiny-net-hls
description: 把一个小 CNN 做成 int8 + FPGA/HLS 可部署 IP 的完整流程：MAC/DSP 预算、int8 QDQ 量化语义、位精确 numpy/C++ 参考模型、V0-V5 验证阶梯、Vivado HLS 落地与排错。用于：给训练好的小网做 int8 量化、把 CNN 写成 HLS/RTL、C 仿真或 RTL 和 golden 对不上、HLS 综合出空设计或 latency 0/资源 0、判断一张网能不能塞进板子。
---

把「训练 → 量化 → 硬件」当成一条**软件先于硬件**的流水线来跑。核心不是写得快，而是每一级都有一个位精确的 golden 可以对，让错误停在最便宜的那一层。

## 主流程

### 0. 先算预算，再写一行代码

对目标器件算三件事，写进规格文档：

- **MAC 总量**：`sum(Ho*Wo*N*Cin*kh*kw)`，`Cin` 直接取 `weight.shape[1]`（depthwise 权重形状是 `[N,1,kh,kw]`，再除 group 会得到 0）。
- **每时钟 MAC 数**：`MAC总 * fps / fclk`。例：112.9M MACs、30 fps、125 MHz → 需要 27 MAC/clk，取 `NMAC=32` 留余量。
- **最大中间张量**：MobileNet 类扩张块的 `H*W*C` 常常是几十万到上百万字节，决定激活放 BRAM 还是 DDR。

完成判据：一张表里给出 `MAC / MAC每clk / 最大张量 / 权重体积 / DSP 估算 / 目标 fps`，并且**输入分辨率已经冻结**——改分辨率等于重训 + 重新生成层表 + 重做 IP，冻结后不要再动。

预算推不动时先砍分辨率再砍通道宽度，输出头（把 21 点热图换成 5 通道回归）几乎不省 MAC，不要在那里浪费时间。

### 1. 量化：把 decode 移出图外

静态 QDQ PTQ（`onnxruntime.quantization.quantize_static`，QUInt8 激活 + QInt8 per-channel 权重）。**softmax / soft-argmax / 任何 exp 都必须移出量化图**：留在图里能让精度掉 200% 以上，移出去只掉 3% 左右。

导出两个图：

- `best.onnx`：训练框架原样，含 decode，给 PC 端 demo 用。
- `best_hm.onnx`（`--logits`）：图在最后一层卷积的**原始 logits** 处结束，decode 由主机 float 完成。这才是给板子的图。

完成判据：同一份测试集上 `int8 NME / fp32 NME - 1 ≤ +5%`，且**三个输出头都验**（热图 + 每个标量/向量头）。只比热图不算数。

量化数学、字段解析、常见踩坑见 [`references/qdq-math.md`](references/qdq-math.md)。

### 2. V1：C++ 功能模型对 int8 ONNX

用主机 g++ 写一份可执行的功能模型（不涉及任何 HLS 语法），从 ONNX 的 QDQ 参数生成层表头文件（`layers.h` + 权重 blob），逐层实现 conv/pw/dw/add/gap/fc。

完成判据：`check_cpp_ip` style 的闸门脚本对 ORT int8 给出 **mean |diff| ≤ 1.0 LSB**，max 几 LSB，且**所有输出头都对**。

### 3. V2：HLS 结构双胞胎，逐位一致

把功能模型改写成 HLS 形状（pragma、`static` on-chip 数组、m_axi scratch、定点化）作为**第二份实现**，两份都链进同一个测试程序，对同一批输入比对。

完成判据：**0 LSB 差异**，所有输出头。`N/N 帧逐位一致`，否则不许进综合。

这一步是本流程最有价值的一环：没有工具链也能把 RTL 之前的所有算术风险消掉。

### 4. V3：HLS csim

同一份源码先用 `csim_design` 跑通（秒级，走同一套前端，不做调度）。测试程序**不能收 argv**（csim 会 `exit(2)`），路径写死并多试几个前缀（csim 的 cwd 在 `prj/solution1/csim/build`）。

完成判据：csim 0 error，且打印的数值和 g++ 版本一致。

### 5. V4：综合

跑 `vivado_hls -f hls.tcl`。**先读三个数字，再看时序**：`SYNCHK 200-77` 计数必须为 0、DSP48 必须非 0、`Latency` 必须非 0。

排错顺序（症状 → 第一嫌疑）：

| 症状 | 第一嫌疑 |
| --- | --- |
| Latency 0 / `has no outputs` / 全端口 dangling | m_axi 指针传进了未内联的子函数，整个调用被当成无副作用删掉 → `#pragma HLS INLINE recursive` |
| 综合 20 分钟没动静、`cdfg-build` 卡住 | 全内联后的大函数惩罚；层表循环 vs 直通生成的选择 |
| `cannot completely unroll ... variable trip count` | 内层 `cin` 是运行期值 |
| `Failed to meet timing` / `II Violation` | NMAC 太大、累加器携带依赖、用了 volatile 把循环边界变成黑盒 |
| DSP 数量 ≈ 层数 x NMAC | 直通生成让每层各起一套 MAC 阵列 |

细节与修法见 [`references/hls-pitfalls.md`](references/hls-pitfalls.md)。

完成判据：`SYNCHK 200-77` = 0、DSP 在预算内、`Latency` 非 0、目标时钟下 II=1 或已知的 II 代价被接受。

### 6. V5：板级

只有在 V0–V4 全绿之后才碰板子。板级只验证三件事：预处理是否与训练一致（颜色通道、插值方式、归一化）、数据通路带宽够不够、端到端延迟。

## 贯穿全程的三条判断规则

1. **测量结果异常先怀疑测量**。离线指标和肉眼观感差 20 倍时，九成是脚本 bug（单位、角点/中心约定、BGR/RGB、双重归一化）。
2. **任何恰好等于 0 的差异都要查单位**。`0.00%` 通常不是逐位一致，而是分子分母同乘了一个错系数。
3. **只对一个输出头的匹配不算验证**。热图对了、尾部 GAP→FC 饱和到 255 的情况真实发生过，而且能躲过好几轮闸门。

## 训练侧：小网能不能成，看数据域不看结构

量化/综合之前，先把「这张网值得不值得上板」定下来：用同一批数据、同一套指标，先改**数据域**再改结构。域内数据能把中位误差减半、把崩溃率从 18% 打到 0.3%，而换头结构只影响 10%。

规则、阈值和小网特有坑见 [`references/data-and-model.md`](references/data-and-model.md)；调试方法论见 [`references/verification.md`](references/verification.md)。

## 交付物清单

一份冻结规格（输入契约、输出契约、定点规则、层表、预算、接口、验证计划）+ 生成器（从 ONNX 出 `layers.h` / `params.h` / 权重 blob）+ 三个测试程序（对 ORT、对功能模型、csim）+ 闸门脚本 + 量化报告。层表和参数一律**生成，不手打**。
