# INT8 量化实测报告（红线 3 遗留项：INT8 未实测 → 已实测）

**日期**：2026-10-15 | **结论一句话**：**检测网 INT8 可用**（中位 5.0 → 5.2~5.5 px，抖动 4.7 → 5.0 px，域内误触发仍 0%，双门槛都过）；**分类网 INT8 掉 0.7~1.2 个点**（acc 92.0% → 90.8~91.3%，fist recall 90.35% → 87.6~89.6%）；**关键前提**：soft-argmax 那段算术必须留在浮点——用 onnxruntime 把全部算子量化后检测中位直接崩到 **9.67 px（不过 8px 门槛）**。

## 1. 为什么这么做（不是"跑个工具"）

《网络结构清单》v1.1 §5 只给了量化**方案**，没有实测；而没有 INT8 实测就无法判断：
① A9 保命版能不能真的用 int8 跑；② PL/HLS 的逐层误差预算能不能照现在的结构分；
③ 清单里"softmax/soft-argmax 不量化"这个设计决定到底值多少精度。

## 2. 两种独立实现（互为交叉验证）

| 实现 | 位置 | 口径 |
|---|---|---|
| **A. 自写 PTQ 假量化模拟器**（主证据，产出 HLS 要的 scale 表） | `model/common/qfake.py` | BN 折进 conv；权重**逐输出通道对称** `s_w=max|w_c|/127`；激活**逐张量**（`sym` 对称 zp=0 / `affine` 非对称）；**int8×int8→int32 精确累加**；bias 浮点；softmax/sigmoid 浮点；热图额外量化一拍 |
| **B. onnxruntime 官方静态量化**（第三方交叉验证） | `results/w1_int8/ort_int8_eval.py` | QDQ 格式、per-channel、MinMax 校准；跑两个变体：**只量化 Conv** / **量化全部可量化算子** |

- 校准集 = **全 val**（检测 982 张、分类 1000 张），分位 99.9%，抽稀固定种子（可复现）
- 阈值/评测脚本与官方一致（直接调用 `detect/eval.py --int8 {sym,affine}`、`classify/eval.py --int8 ...`、`results/w1_detect_neg/eval_negatives.py --int8 ...`）

## 3. 检测网结果（val 564 正样本，官方口径）

| 方案 | 中心误差中位 | p90 | 接受 n | 召回@0.5 | 抖动 jitter | 域内误触发@0.5 | 门槛 |
|---|---|---|---|---|---|---|---|
| **float32（现冻结版）** | **4.997 px** | 52.58 | 539 | 95.57% | 4.717 | **0%** | ✅ |
| 模拟器 INT8 `sym` | 5.495 px | 51.53 | 539 | 95.57% | 4.704 | **0%**（conf max 0.241） | ✅ |
| 模拟器 INT8 `affine` | **5.064 px** | 49.78 | 540 | 95.74% | 4.695 | **0%** | ✅ |
| ORT INT8 **只量化 Conv** | **5.237 px** | 53.49 | 536 | 95.04% | 4.969 | **0%**（conf max 0.123） | ✅ |
| ORT INT8 **量化全部算子** | **9.673 px** ❌ | 50.82 | 536 | 95.04% | 5.572 | 0% | ❌ 中位不过 |

- **代价**：中位 +0.07~0.5 px、抖动 +0~0.25 px（余量都还有 2.5 px 以上）
- **安全性没退化**：自录域内负样本误触发仍是 **0%**（@0.2 也只 1.08%，最高 conf 0.241 << 0.5）
- HaGRID `no_gesture` 误触发：浮点 17.85% → 模拟器 20.92% / ORT 17.2~17.5%（域外数据，本身已接近随机）

### ⭐ 关键发现：soft-argmax 必须留浮点
同一份权重、同一个评测脚本，只有量化范围不同：
- ORT 量化**全部算子**（含热图之后的 Softmax/Mul/Add/ReduceSum 那串坐标期望）→ 中位 **9.673 px，不过关**
- ORT **只量化 Conv**（坐标期望留浮点，其余与上者相同）→ 中位 **5.237 px，过关**

→ 《网络结构清单》§5 里"热图 int8 交后端算 softmax+期望"的决定**不是保守，而是必需**，现在有数据支撑。

## 4. 分类网结果（val 1000 张）

| 方案 | val acc | 闭合 fist recall | 张开 palm recall | 其它→误动作率 |
|---|---|---|---|---|
| **float32（现冻结版）** | **92.00%** | **90.35%** | 92.13% | 7.11% |
| 模拟器 INT8 `sym` | 90.80% | 88.42% | 90.49% | 7.57% |
| 模拟器 INT8 `affine` | 91.10% | 88.80% | 89.84% | 6.65% |
| ORT INT8 全部算子 | **91.30%** | 89.58% | 91.15% | 7.57% |
| ORT INT8 只量化 Conv | 91.20% | 87.64% | 91.48% | 6.88% |

- **代价**：acc −0.7~−1.2 个点；**闭合(握拳)方向 recall 掉到 87.6~89.6%**（浮点 90.35%，我自定的 ≥90% 门槛在边缘）；其它手势误动作率基本不变
- 两种实现结果一致（90.8~91.3%），说明模拟器**不偏乐观**——它比 ORT 还略保守
- 重量级结论：INT8 对分类网的伤害比检测网明显，因为分类网是 **GAP + 1 个 1×1 头**，最后那 96→3 的 logits 直接被量化噪声主导（逐层表里 `head` 输出 SNR 只有 12.5 dB，而余弦 0.9897 —— 排序基本没变，但边界样本会翻）

## 5. 逐层 scale 表 + 逐层误差预算（HLS 直接可用）

完整表：`results/w1_int8/int8_layer_table.md`（机器版 `int8_layer_table_sym.json`）
工具：`model/tools/int8_report.py`（`cd model && HAGRID_ROOT=data/cache256 python tools/int8_report.py --mode sym`）

检测网摘录（校准 982 张）：

| 层 | 输入 scale `s_x` | 权重 scale `s_w` 中位/最大 | 权重被压成 0 | 输入 SNR | 输出 SNR | 输出余弦 |
|---|---|---|---|---|---|---|
| `stem.0` | 0.0207874 | 0.003247 / 0.006365 | 0.93% | 45.8 dB | 39.6 dB | 1.00001 |
| `s2.0.0` | 0.0330094 | 0.01348 / 0.03049 | 0.00% | 24.1 dB | 20.5 dB | 0.99559 |
| `s4.1.0` | 0.0410960 | 0.003508 / 0.005146 | 0.81% | 22.4 dB | 14.7 dB | 0.98350 |
| `s6.1.0` | 0.0354341 | 0.005826 / 0.007417 | 0.76% | 16.2 dB | **10.7 dB** | 0.96037 |
| `conf_head` | 0.0105816 | 0.003536 | 0.00% | 42.5 dB | 10.7 dB | 0.97940 |
| `uv_head.2`（热图） | 0.0382268 | 0.003952 | 0.00% | 29.6 dB | 30.3 dB | 0.99954 |

- 全层输入量化 SNR：**16.2 ~ 45.8 dB**（中位 24.1），输出 SNR **10.7 ~ 40 dB**，输出余弦 **0.96 ~ 1.0**
- 权重被压成 0 的比例 ≤ 1.04%（没有"深度卷积权重被量化死"的问题）
- 权重体积：检测 27.9K 参数 → **≈28 KB** int8（fp32 权重文件 140 KB）；分类 15.7K → **≈16 KB**（fp32 82 KB）；导出 ONNX int8 实测 91 KB / 47 KB
- BN 折叠正确性已校验：折叠前后网络输出最大绝对差 **6.3e-07（检测）/ 2.0e-06（分类）**，即纯 float32 噪声

## 6. 踩到的坑（都已修，写下来避免后人重犯）

1. **affine 的 zero_point 约定写错**：把 `[lo,hi]` 映射到 0..255（uint8 约定）却塞进 int8 → 正半轴只剩 127 码，ReLU 型激活被截断一半（初测 52% clip、SNR 3.5 dB 的"灾难"其实是我自己的 bug）。正确写法 `zp = round(-lo/s) - 128`，修正后 affine 反而比 sym 好（SNR 54 dB vs 48 dB）。
2. **逐层 SNR 的对照基准必须是"同样折叠了 BN 的浮点网"**：拿未折叠的浮点网比，比的是 BN 的 scale 差，会得出 −5 dB 的假结论（差点把错误数字发去做 HLS 预算）。
3. **校准抽稀要固定种子**：否则每层分位估计抖动 → 精度 ±0.2pt，不可复现。
4. **onnxruntime 默认会量化 softmax 及坐标期望算术**：检测精度直接崩到 9.67px。要么 `op_types_to_quantize=['Conv']`，要么按 HLS 方案自己写量化。
5. torch 2.11 默认 ONNX 导出器要 `onnxscript`（本机没装）；用 `torch.onnx.export(..., dynamo=False)` 走旧导出器 + `dynamic_axes` 才能喂 ORT。

## 7. 判定与建议

| 项 | 结论 |
|---|---|
| 检测网 INT8（A9/NCNN、PL/HLS） | ✅ **可用**。两门槛（中位、抖动）都过，域内误触发仍 0%，代价 ≤+0.5 px |
| 分类网 INT8 | ⚠️ **可用但掉点**：acc −0.7~1.2pt，闭合 recall 到 88~89.6%。若坚持 ≥90% 门槛，需 QAT 补精度 |
| soft-argmax / sigmoid | ✅ **必须留浮点**（独立量化器已证明量化它会崩） |
| 逐层误差预算表 | ✅ 已产出（HLS 可直接用） |

**后续实测（2026-10-15 下午）**：检测网的**固定 scale QAT 无收益**（best 5.48px vs PTQ 5.495px），见 `results/w1_detect_oob/README.md` §6；分类网 QAT 未做（同样预计收益有限，瓶颈在标签噪声与逐张量激活量化）。

**下一步可选（等你定）**：
1. **分类网 QAT**（约 20~30 分钟）：把分类网做量化感知微调，目标把 int8 的 acc 拉回 ≥92%、fist recall ≥90%
2. **混合精度**：分类头/最后几层保持 int16 或 fp16（PL 侧代价很小），其余 int8
3. **不动**：直接用 int8 权重（检测已达标，分类接受到 91%），把重点放回板级/视觉通路

## 8. 产物清单

| 路径 | 内容 |
|---|---|
| `model/common/qfake.py` | INT8 PTQ 假量化模拟器（BN 折叠 / 逐通道权重 / 逐张量激活 / int32 累加 / 逐层 SNR） |
| `model/tools/int8_report.py` | 逐层 scale + 误差预算表生成器 |
| `results/w1_int8/int8_layer_table.md` / `.json` | **逐层量化表（HLS 用）** |
| `results/w1_int8/{detect,classify}_eval_int8_{sym,affine}.json` | 模拟器 INT8 官方指标 |
| `results/w1_int8/{detect,classify}_scales_{sym,affine}.json` | 模拟器量化表（compact） |
| `results/w1_int8/neg_eval_int8_{none,sym}.json` | 误触发/拒识（float vs int8） |
| `results/w1_int8/ort_int8_eval_{all,convonly}.json` | ORT 交叉验证 |
| `results/w1_int8/*.onnx` | 导出图（fp32 + int8×2） |
| `results/w1_int8/run_logs/` | 训练/评测日志 |

## 9. 复现命令

```bash
cd model && export HAGRID_ROOT=data/cache256
# 逐层量表
python tools/int8_report.py --mode sym --n 1000
# 官方指标（检测）
python detect/eval.py --weights frozen/detect_fp32.pt --int8 sym \
    --eval-out ../results/w1_int8/detect_eval_int8_sym.json --int8-report ../results/w1_int8/detect_scales_sym.json
# 官方指标（分类）
cd classify && python eval.py --weights ../../model/frozen/classify_fp32.pt --int8 affine \
    --eval-out ../../results/w1_int8/classify_eval_int8_affine.json
# 误触发率（INT8）
cd .. && python ../results/w1_detect_neg/eval_negatives.py --weights frozen/detect_fp32.pt \
    --int8 sym --tag int8_sym --out ../results/w1_int8/neg_eval_int8_sym.json
# 第三方交叉验证（只量化 Conv / 全部算子）
python ../results/w1_int8/ort_int8_eval.py --which both --op-types conv
python ../results/w1_int8/ort_int8_eval.py --which both --op-types all
```
