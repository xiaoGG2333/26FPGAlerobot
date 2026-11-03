# W1 检测网 —— 三组训练 + 尾部根因调查（完整证据）

> 时间：2026-10-14 19:29–21:35 | 环境：`D:\Anaconda3\envs\fpga_amd-pi\python.exe`（torch 2.11.0+cu128 / RTX 5060 Laptop）
> 主人口径：**中位误差与抖动都要 ≤8 px**

## 1. 一句话结论

1. **方案 A（镜像缓存）**：建缓存 38 秒、40 epoch 训练 8–9 分钟 —— 比原预估快 3~5 倍。
2. **旧结构（GAP 头）精度被证死**（中位 25.81 px），**改 soft-argmax 热图头后达标**。
3. **最终冻结**：`softargmax_hm`（裁剪对齐 + 高斯热图监督）→ 中位 **4.41 px** ✅ / 抖动 **4.52 px** ✅。
4. **但两个更深的问题被查清，比"调模型"重要得多**：
   - **标签管线 bug**：训练裁剪窗口与部署/评估不一致（旧版训练取近全图非方窗、纵轴被压 25%），且会把手掌标注中心裁出画面（实测 7.33% 的裁剪窗口切掉标注中心，标签 v 值跑到 -0.49~1.96）。
   - **HaGRID 标注噪声 ≈ 8 px 门槛本身**：近同帧配对（画面差异 0.2%）的标注中心中位跳 **11.4 px**，70% 的对跳 >8 px → **在 HaGRID 上继续压指标已触数据天花板**，真正的验收必须用自录演示数据（该目录至今为空）。

## 2. 产物

| 文件 | 说明 |
|---|---|
| `model/detect/net.py` | `DetectNet(head='softargmax')`：stride-4 热图 + soft-argmax；`head='gap'` 保留旧结构对照 |
| `model/detect/train.py` | uv 损失 `beta=0.02`(≈L1)、可选高斯热图监督（`--hm-weight/--hm-sigma/--hm-loss`）、写 `detect_train_meta.json` |
| `model/detect/dataset.py` | 训练时传标注中心 + `keep_center=True` |
| `model/common/transforms.py` | `geom_params` 裁剪策略与部署对齐 + `crop_origin` 保证标注中心不被裁掉 |
| `model/data/build_cache.py` | 镜像缓存构建（方案 A） |
| `model/detect/eval.py` | 修掉 `default_rng().randint` bug（原来评估脚本根本跑不起来） |
| `model/tools/dump_layers.py` | 导出逐层表（《网络结构清单》用） |
| `docs/网络结构清单.md` | **冻结版 v1.0**（逐层算子/shape/MACs/量化方式 + 限制） |
| `model/frozen/detect_fp32.pt` 等 | **冻结权重**（= run3 softargmax_hm）+ 曲线 + 元信息 + eval |
| `results/w1_detect_train/{gap_head_baseline,softargmax_oldcrop,softargmax_cropfix,softargmax_hm}/` | 四组权重/曲线/评估/诊断留档 |
| `results/w1_detect_train/*.log` | 四组训练日志 + 建缓存日志 |
| `results/w1_detect_train/diagnose_detect.py`、`check_label_pipeline.py`、`check_label_noise.py` | 三个可复现验证脚本 |
| `results/w1_detect_train/detect_diagnose.json`、`detect_label_noise.json`、`detect_pred_grid*.png` | 诊断报告与可视化（对应 run3） |
| `results/w1_detect_train/layers_dump.md` | 逐层表原始输出 |

## 3. 方案 A 实测 vs 预估

| 指标 | 预估（2026-10-13） | 实测 |
|---|---|---|
| 建缓存 | 4.6 分钟 / 184 MB | **38.2 秒 / 186 MB**（8162/8162 成功，23 workers） |
| 每 epoch | 约 43 s | GAP 头 14.7 s；soft-argmax 头 11.3 s；+热图监督 13 s |
| 40 epoch | 30–50 分钟 | GAP 9.8 分钟；soft-argmax 7.5 分钟 |
| 单样本加载（单进程 300 次） | 3.5 ms | 18.5 ms（原图同法 56.0 ms → **3.0×**） |

## 4. 旧结构（GAP 头）为什么不行

| 探针（val，144 尺度） | 中位误差 |
|---|---|
| 恒预测画面平均中心（无模型基线） | 36.82 px |
| GAP 头检测网 | 26.17 px |
| 同上 + 事后线性校准 | 25.53 px（无改善） |
| **同网络 96 维 GAP 特征的线性探针** | **26.89 px ← 天花板** |
| pred std 15.4/9.2 px（GT 31.7/31.4）、r = 0.61/0.42 | 严重向均值回归 |

- 控制实验排除缓存嫌疑：同一权重换原始原图评估 = 26.17 px（差 0.36 px）。
- 结论：`mean(dim=(2,3))` 把空间信息平均掉 + 逐位置共享的线性读出，**结构天生不能定位**。

## 5. 四组训练对照（同 40 epoch / cache256 / workers=2）

| 组 | 结构/管线 | 中位 ✅≤8 | 抖动 ✅≤8 | val >8px 占比 | in-crop p90 | 手线性尺寸≥0.15 的 p90 |
|---|---|---|---|---|---|---|
| gap_head（旧） | GAP+1×1 | ❌ 25.81 | ✅ 2.58 | — | — | — |
| run1 softargmax_oldcrop | 热图头，旧裁剪 | ✅ 4.44 | ✅ 4.36 | 37.1% | 35.65 px | 21.9 px |
| run2 softargmax_cropfix | +裁剪对齐/标签修正 | ✅ 4.48 | ✅ 4.48 | 40.8% | 39.6 px | **13.4 px** |
| **run3 softargmax_hm（冻结）** | +高斯热图监督 | ✅ **4.41** | ✅ **4.52** | 38.1% | 42.8 px | 15.4 px |

- run2/run3 的改动让**大手场景**（演示场景）明显变好（p90 21.9 → 13~15 px），但对整体 >8px 占比无改善 —— 原因见 §6。
- 三组 soft-argmax 的中位/抖动差异（4.41~4.48）**都在标注噪声（~11 px）之内，属噪声级差异**，不该据此做取舍；因此按"管线最正确 + 官方中位最好 + conf 召回最高(95.9%)"选 run3 冻结。
- 训练曲线（run3）：ep10 8.7 px → ep20 5.6 px → ep30 4.6 px → **best ep37 4.41 px**。

## 6. 两个真问题（关键证据）

### 问题 1：裁剪标签管线（已修）

| 检查（`check_label_pipeline.py`，1500 个 train 正样本） | 结果 |
|---|---|
| 旧行为（`keep_center=False`）：标注中心被裁出画面 | **110/1500 = 7.33%**，标签 v 范围 **-0.492 ~ 1.958** |
| 修复后（`keep_center=True`） | **0/1500 = 0.00%**，v 范围 0.051 ~ 0.949 |

- 同时修掉一个更隐蔽的失配：旧训练裁剪是 **192×144 近全图非方窗**（纵轴被压 25%），而评估/部署是 **144×144 中心裁剪** → 同一只手在 val 里看起来约 **1.33 倍**高。现训练用同口径 k×k 方窗（k=size×U[0.88,1]）+ 随机位置。
- 影响：模型输出被 sigmoid/soft-argmax 约束在 [0,1]，出界标签**物理上不可预测**，训练时还会污染梯度。

### 问题 2：HaGRID 标注噪声就是 8 px 门槛（`check_label_noise.py`）

| 指标 | 值 |
|---|---|
| 训练集误差（部署同款裁剪） | 中位 4.76 px / p90 53.2 / >8px **35.8%** |
| 验证集误差 | 中位 5.02 px / p90 59.4 / >8px **40.8%** |
| **近同帧配对**（同 user_id、画面差异中位 0.55/255） | 标注中心中位差 **11.39 px**（p90 72.85），**70.1% 的对 >8 px** |
| 同批配对里模型预测差 | 中位 15.82 px（p90 51.4） |

- **训练误差 ≈ 验证误差** → 不是过拟合/泛化问题，是 (图, 标签) 对本身的噪声地板。
- 近同帧的标注自己就跳 11 px → **在 HaGRID 上"≤8px"已无法再压**，模型一致性不会比标签更好。
- 结论：`results/w1_detect_train/README.md` 里所有 4.x px 的中位数都受这个地板的约束；**要判断系统是否够用，必须用自录演示数据（`model/data/recorded/`，目前为空）**。

## 7. 验收判定（主人口径：两者都要过）

| 指标 | 门槛 | 冻结模型（run3） | 判定 |
|---|---|---|---|
| 中心点误差中位 | ≤8 px | **4.41 px** | ✅ |
| 中心点抖动（±2px 输入扰动 30 次） | ≤8 px | **4.52 px** | ✅ |
| 分类 val acc | ≥90% | 93.09% | ✅ |
| 两模型 MACs | ≤30M | 25.84M | ✅ |

**但必须同时记录**：val 上仅 62% 样本 ≤8 px（p90 55.4 px）；其中 4.8% 是手不在部署裁剪窗口内（拍摄取景，非模型），其余主要由标注噪声解释。

## 8. 复现命令

```bash
cd model
D:/Anaconda3/envs/fpga_amd-pi/python.exe data/build_cache.py                       # 38 秒
HAGRID_ROOT=data/cache256 D:/Anaconda3/envs/fpga_amd-pi/python.exe \
    detect/train.py --epochs 40 --workers 2 --hm-weight 1.0                        # 冻结版（~11 分钟）
HAGRID_ROOT=data/cache256 D:/Anaconda3/envs/fpga_amd-pi/python.exe detect/eval.py  # 中位+抖动
D:/Anaconda3/envs/fpga_amd-pi/python.exe ../results/w1_detect_train/diagnose_detect.py <权重>
D:/Anaconda3/envs/fpga_amd-pi/python.exe ../results/w1_detect_train/check_label_pipeline.py
D:/Anaconda3/envs/fpga_amd-pi/python.exe ../results/w1_detect_train/check_label_noise.py <权重>
D:/Anaconda3/envs/fpga_amd-pi/python.exe tools/dump_layers.py --markdown           # 逐层表
```

## 9. 决策与遗留

| 时间 | 决策 | 结果 |
|---|---|---|
| 19:29 | 主人："先跑 A 选项" | 缓存+训练完成，比预估快 3–5 倍 |
| 19:50 | 主人："1. 允许改头 2. 两者都要过" | soft-argmax 头 → 双指标达标 |
| 21:0x | 主人："修，同时产出" | 修裁剪标签 bug + 热图监督；产出《网络结构清单》v1.0 |
| **待办** | INT8 量化方案（QAT/PTQ + 校准 + 复测）| 清单 §5 只给方案，未实测 |
| **待办** | **自录演示数据**（`recorded/` 为空）→ 最终验收判据 | 阻塞项，需主人录制 |
| **待办** | conf 不自检（坏样本 conf 0.991）→ 闭环需热图熵/时序一致性兜底 | 设计项 |

> 未留任何工程外临时文件；`model/_scratch_test/`、`results/w1_detect_train/_run3.pt`（临时权重）等已清理。

## 10. 闭环自检兜底的实测评估（2026-10-14 21:40）

脚本 `check_selfcheck.py` → `detect_selfcheck.json`（564 个 val 正样本，冻结权重）

### 10.1 开销（问题：花多少算力）

| 兜底 | 额外算子 | 相对检测网 | PL 100MHz | 相对 33ms 帧预算 | 新增硬件 |
|---|---|---|---|---|---|
| 热图熵 H(p) | 1296 格 ×(LUT log2 + mul + add) ≈ **3.9K 操作** | **0.019%** | 13 µs（1格/周期）~ 52 µs（4周期/格） | **0.04% ~ 0.16%** | 512 B log LUT（或 CORDIC） |
| 时序一致性 | ≈ 10 次整数运算 + 4 个寄存器 | 0.00005% | ≈ 0 | ≈ 0 | 无 |
| （顺带测）边界检查 | 2 次比较 | 0.00001% | ≈ 0 | ≈ 0 | 无 |

**算力层面：三个都是白送。** 真代价是 **1 天的 HLS/软件实现 + 阈值调参**，以及时序一致性带来的 **1 帧判定延迟**（首帧无法判；要 N 帧确认则再加 N 帧延迟）—— 那是延迟代价，不是算力代价。

### 10.2 有效性（问题：白送的东西有用吗）—— 答案：基本没用

| 自检信号 | 与误差相关性 | >8px 坏样本召回（报警率10% / 20%） | 评判 |
|---|---|---|---|
| 热图熵 | r = 0.199 | **28.7% / 37.9%** | 弱 |
| 热图峰值 maxp | r = 0.145 | 18.4% / 33.3% | 弱 |
| conf（已有） | r = 0.082 | 14.9% / 32.2% | 无用（证实旧结论） |
| 时序一致性 τ=8px | — | 抓到 83%，但**报警里只有 27% 是真坏** | 分不清"真运动/标注噪声"与"模型失败" |
| 边界检查（专治手出画） | r = 0.089 | 对 OOB 只抓 **11%~37%** | 无用 |

三条关键反证：
1. **最严重的失效恰好是"自信地错"**：手出画样本的熵 **1.493 < 正常 1.639**（热图更尖），熵法只抓到 22%。
2. **热图不往边上跑**：手出画时预测点离画面边距离 0.301 vs 正常 0.285（几乎无区别）→ 模型不会用"靠边"表达"手不在"。
3. **时序跳变 ≈ 标注/姿态跳变**：近同帧 Δpred 13.18px vs Δgt 11.39px（模型一致性与标签同级）→ 时序法和标签噪声混在一起。

**根因**：`uv` 头的训练集里**从来不存在"裁剪窗内没有手"的样本**（`keep_center=True` 保证裁剪必含标注中心），所以模型没有"我不知道/手不在"的表达能力 —— 任何输出后处理都救不了。

### 10.3 建议（真正该做的事，零 MACs）

| 方案 | 算力成本 | 说明 |
|---|---|---|
| ❌ 熵/时序/边界兜底 | 0，但要 1 天实现 | **不推荐**：召回 29%，且漏掉最严重的出画失效，还会给人虚假安全感 |
| ✅ **给 conf 头喂"窗内无手"负样本** | **0 MACs** | 按部署口径的中心裁剪生成"裁剪窗切掉手"的样本，标 conf=0（不进 uv 回归）→ 让"手不在"变成可学、可检的信号 |
| ✅ 演示场景兜底（工程最简） | 0 | 相机固定、手在中间：用上一帧目标 + 连续性门限保持（hold-last），配合超时停机，比自检更可靠 |

> 结论一句话：**兜底不是性能问题，是有效性问题。** 零算力的输出后处理救不了"模型没学过不知道自己不知道"，要从数据侧给它上这一课。
