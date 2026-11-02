# 交接：阶段1 M0 数据管线 + sanity 模型开工（2026-11-02 定稿）

> **给新对话的第一个动作**：读本文件 → 按第 2 节向主人确认 3 个决策点 → 读第 8 节前置文档里标"必读"的两份 → 按第 4 节任务清单开工。
> 本文件自带全部前置结论，已定事项不要重新论证。任务来源：《阶段1-视觉模型训练-计划书》§12"立即可以开始的三件事"，主人 2026-11-02 授权主线开工。

---

## 0. 执行记录（2026-11-02，与正文冲突处以本节为准）

| 项 | 结论 | 证据 |
|---|---|---|
| 决策① backbone | **已按建议 A 执行**（MobileNetV2 w0.75、cut=14、纯 ReLU6），主人"直接按照规划开始"视为同意 | `tools/train_kpt.py` HandKptNet |
| 决策② 现成模型扫描 | **已执行，候选死链**：ModelScope `tzlzy321/hand` 的 model.onnx 服务端文件为空（resolve 与 API 路径均返回空/500）。T6 搁置 | 下载产物仅 145B JSON 错误体 |
| 决策③ M2 录制 | **取消**：主人 2026-11-02 拍板"不用自录视频，找网上数据集（远近都包含）蒸馏"。`sw/collect_data.py` 已写但停用 | 本节 |
| T2 calib_rec.mp4 | **全域外，弃用**：keep 207/7687=2.7%，88% 帧无手；画面为人坐桌前持手柄、手长 hp≈0.05~0.075（部署域 0.14~0.20）。教师无问题，是素材域不对 | `results/stage1/label_check_calib_rec.png`、`calib_rec_labels_meta.json` |
| 新数据路线 | **HaGRID 本地子集（9062 张，6 类）+ 裁剪窗双向尺度增广**：目标 hp log-uniform 0.06~0.34，增广后部署带内占 27%。图像 i.i.d. → 随机 8:1:1，先按图像划分再增广（防同源泄漏） | `tools/label_images.py`；普查 `results/stage1/hagrid_survey.{json,png}`（hp 中位 0.085、80%<0.12、近端 0.7%）；增广证据 `results/stage1/distill_check.png` |
| FreiHAND | 调研未混入：官网可达 3.7GB，但**许可 research-only 禁商用**（同类红线需书面确认），且 RGB 为 224² 手部裁切图与全帧模型尺度约定冲突。留后备 | 官方页 Terms of use 原文 |
| 门槛口径增补 | 时序抖动门槛在 i.i.d. 图像缓存上**按构造不适用**（相邻样本无时间关系）→ `eval_kpt.py` 按缓存 meta 的 `temporal` 标志显式 SKIP 并写入 eval.json 理由；M2 视频数据恢复该门槛。非事后挑指标 | `tools/eval_kpt.py` |
| 环境 | train-pi 已建并验证：`2.11.0+cu128 (12, 0) True` | T1 |
| 缓存布局 | 图像路线由 `label_images.py` 直接产出（imgs/pts/presence/hand/scale .npy + split.json + meta.json），与 `build_cache.py`（视频路线，M2 用）同布局；`train_kpt.py`/`eval_kpt.py` 两者通吃 | `results/stage1/cache_hagrid/` |
| sanity_run1（gap 头） | **6 FAIL / 3 PASS**：nme 0.2745（门槛 0.030）、pck05 0.109、hp_ratio 0.137、theta 10.7°、ext 0.731 FAIL；uv 0.016 / jitter / ort 4.4ms PASS。误差形态（vis_grid.png）：干净单手 best-12 红绿几乎重合（nme 0.05~0.09），worst-12 是硬样本长尾（持物/遮挡/极小手/双手歧义）→ 结构精度天花板 + 数据尾噪叠加 | `results/stage1/sanity_run1/` |
| 分支执行 | 按 §3 分支升级 **heatmap 头**（stride-8 trunk、21×32×32、网内 soft-argmax，导出仍为坐标；部署端热图后端用归一化 ReLU 质心零 exp）：`sanity_run2` 40ep 后台训练中（231 s/ep，约 2.6 h）。单变量对照：数据不动 | `tools/train_kpt.py --head heatmap` |
| sanity_run2 终评 | **7 FAIL**：nme 0.2176、pck05 0.241、hp_ratio 0.147、theta 10.8°、ext 0.666、ort 14.4ms（heatmap 头 CPU 超延迟门槛）；uv/jitter PASS。对照 run1 改善有限 | `results/stage1/sanity_run2/eval.json` |
| 诊断（ep29 中期） | ① train med 0.089 ≈ val med 0.117 → **欠拟合为主**，非泛化差；② 按手长分桶：带内 med 也有 0.096，非尺度单因子；③ H3 软质心收缩假说**被否**（soft 0.091 优于 hard-argmax 0.165 → 热图峰本身不尖不对）；④ run1 uv PASS(0.016) 说明"手在哪"学会、"手指形状"没学会 → **单阶段全帧小网同时背检测+姿态两任务，超出其容量** | `_scratch/diag_run2.py`、`_scratch/diag_h3.py`（临时脚本，结论已录） |
| 路线修正 | 触发计划书 §5.2 预登记分支：**两阶段**（粗定位 ROI → ROI 内精回归）。阶段二 sanity 用教师框 oracle-ROI（`tools/build_roi_cache.py`，margin 1.3~2.0 增广，roi.npy 存仿射参数，评测回全帧、门槛口径不变）；阶段一真检测器复用 W1 DetectNet 思路另行立项。run1/run2 归档为单阶段基线 | `results/stage1/cache_hagrid_roi/` |
| sanity_run3（ROI+gap） | 5 FAIL / 3 PASS / 1 SKIP：nme 0.1167、pck05 0.329、theta 6.75°、ext 0.807 FAIL；**uv 0.0066、hp_ratio 0.0888、ort 4.7ms PASS**（通道指标首次过半）。结论：裁切解决了"在哪"，GAP 头的形状天花板仍在 → 两因子分解成立 | `results/stage1/sanity_run3_roi_gap/` |
| sanity_run4 | ROI + heatmap stride-16，40ep 后台训练中（任务 b18jl2fqk）。若仍不达标再上 stride-8（约 1.7h） | `results/stage1/sanity_run4_roi_hm16/` |
| sanity_run4 终评+诊断 | 5 FAIL：nme 0.1005、theta 6.41°、ext 0.805；uv 0.0062 / hp_ratio 0.0838 / ort 4.4ms PASS。分层诊断：**train med 0.0366（拟合已近门槛）/ val med 0.055~0.059 / tail 6%** → 矛盾从欠拟合转为**泛化差距 + 野姿长尾**；heatmap 方向对但 stride-16 太粗、增广包太弱 | `_scratch/diag_run2.py`（注意：该脚本 cache 路径需按 argv[2] 传，曾误用全帧缓存测 ROI 模型得出假结论，已修） |
| sanity_run5 | ROI + heatmap **stride-8 + σ1.0 + 增广包**（旋转±15°/亮度对比/运动模糊，`--aug`），40ep 后台（约 1.7~2 h）。针对实测泛化差距的组合改动，非单变量，已注明 | `results/stage1/sanity_run5_roi_hm8_aug/` |
| 门槛口径预告 | HaGRID 野姿长尾（持物/遮脸/双手歧义）比部署域（配合用户、干净单手、固定机位）难得多；若 run5 在"部署类似层"（hp 带内+教师高分+单手）过门槛而全集 mean 仍败，将带分层数据与主人讨论门槛作用域重登记——**不自行改门槛** | — |
| 实现坑增补 | HaGRID 原图达 4K，教师全分辨率推理 ~0.2s/张 → 检测前降到 ≤1280（归一化坐标不变），裁切仍用原图；`eval_kpt` stage2 仿射回全帧时 r 需 `[:,None]` 广播 | `tools/build_roi_cache.py`、`tools/eval_kpt.py` |
| 实现坑两则 | ① `np.memmap` 被 pickle 进 DataLoader worker 会整块进内存（Windows spawn 必炸）→ CacheDS 懒加载；② torchvision `ConvBNReLU` 是 Sequential，改 depthwise stride 要设 `conv[1][0].stride`，且 HeatmapHead 已加特征图尺寸断言防静默错配 | `tools/train_kpt.py` |

新增/停用文件：`tools/label_images.py`（新，图像打标+增广）、`tools/{build_cache,train_kpt,eval_kpt}.py`（新）、`sw/label_auto.py`（新，视频版留 M2 用）、`sw/collect_data.py`（新但停用）。

---

## 1. 任务定义与边界

**目标**：建立"视频 → 自动打标 → 训练 → 任务层评测"的最小闭环，用现有 `calib_rec.mp4`（7687 帧）训出 sanity 模型，回答一个问题：**"小网蒸馏 MediaPipe"这条路线在我们的部署域上可行吗？** 可行才值得投 2 小时录 M2 正式素材。

**明确不做**（越界前必须问主人）：
- ❌ 不接板子、不发串口、不动 `fpga/`、不开摄像头长跑（本轮全部离线）
- ❌ 不做 M2 录制（等 sanity 结论 + 主人拍板）
- ❌ 不做量化/上板（M3/M4 路线未拍板，见 `docs/调研-DPU路线可行性-Zynq7100-20261101.md`）
- ❌ 不改 `sw/pose_map.py` / `sw/pose2arm.py` 的映射、标定、协议逻辑（只加 `--model-kind` 开关，T4）
- ❌ 不动 `model/frozen/` 冻结资产、不碰 HaGRID

## 2. 开工前问主人的 3 个决策点（先问再动手）

| # | 决策 | 建议 | 依据 |
|---|---|---|---|
| ① | **backbone 结构**：A. MobileNetV2 width 0.75、去最后 stage、纯 ReLU6（建议）／ B. 计划书原稿 MobileNetV3-Small 前 4 段 | **A** | DPU 调研 §5.2：hard-swish/SE 在三条部署路线（VAI 1.4.1 DPU / FINN / hls4ml）全部容不下；换 A 属计划书 §5.1 结构变更，按项目规矩需主人签字 |
| ② | 是否**并行做半天"现成模型扫描"**（下载 ModelScope `tzlzy321/hand` pose_hand21 ONNX（Apache 2.0）+ 腾讯云 SqueezeNet 21 点项目权重，当第二基线跑同一套任务层评测） | 做（不阻塞主线） | 2026-11-01 对话调研结论：现成模型省不了任务层验收/域微调/量化三步，但可当 baseline；下载外部文件需主人点头（落 `model/data/external/`） |
| ③ | sanity 达标后，是否排期 **M2 的 2 小时素材录制**（需主人本人配合，采集规范见计划书 §4.2） | 达标就排 | 计划书 §12 策略："先 1+2 出结论，再决定是否投入录制" |

## 3. 预登记验收门槛（踩坑清单铁律：先登记再开跑，不许事后挑指标）

**sanity 门槛**（单视频数据，比 M1 正式门槛放宽；这是"路线可行性"门槛，不是最终验收）：

| 指标 | sanity 门槛 | M1 正式门槛（计划书 §6.3，对照用） |
|---|---|---|
| NME（手掌尺度归一化） | ≤ 0.030 | ≤ 0.025 |
| PCK@0.05 / @0.10 | ≥ 85% / ≥ 95% | ≥ 90% / ≥ 98% |
| 任务层：u,v 偏差（vs 教师） | ≤ 0.04 | ≤ 0.02 |
| 任务层：手长比偏差 | ≤ 10% | ≤ 5% |
| 任务层：theta 偏差（周期差） | ≤ 6° | ≤ 3° |
| 任务层：伸指数完全一致率 | ≥ 90% | ≥ 95% |
| 单帧抖动 | ≤ 教师 1.5× | 同 |
| PC 推理（ORT CPU 4 线程 256²） | ≤ 8 ms | ≤ 5 ms |

**任务层指标才是验收依据**（计划书 §6.3 原话：只看 NME 不够，伸指数在阈值附近跳变会导致夹爪乱开合）。
评测方法：预测 21 点喂 `pose_map.Geometry.compute` → `PoseMapper`（现场 `results/w3_pose/calib.json`）→ 与教师输出比 5 通道；theta 比较必须用周期差（-179° vs +179° = 2° 不是 358°）。

**分支判定（结果出来照着走，不临场发明）**：
- 全达标 → T4 + 汇报，问决策③
- NME 不达标但任务层达标 → 记录后继续（任务层为准）
- 都不达标 → 先看可视化定位原因；若误差图显示"点位无空间分辨率"（预测点塌向中心/与输入无关），升级头部为 **21 通道热图（36×36）+ soft-argmax**（踩坑清单 §F 方案）重训，预算 +1 天；重跑前第一轮产物归档到 `results/stage1/sanity_run1/`，不许覆盖

## 4. 任务清单（按序做，每步验收后再下一步）

### T1 建 `train-pi` 环境（半天，含验证）

```bat
set CONDA_ENVS_PATH=D:\Anaconda3\envs
D:\Anaconda3\_conda.exe create -n train-pi python=3.10 -y
D:\Anaconda3\envs\train-pi\python.exe -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
D:\Anaconda3\envs\train-pi\python.exe -m pip install numpy opencv-python onnx onnxruntime-gpu tqdm tensorboard -i https://pypi.tuna.tsinghua.edu.cn/simple
```

装完**必须**跑一行验证（RTX 5060 = Blackwell sm_120，torch<2.7/cu<12.8 会报 `no kernel image is available`）：

```bash
D:/Anaconda3/envs/train-pi/python.exe -c "import torch;print(torch.__version__, torch.version.cuda, torch.cuda.get_device_capability(), torch.cuda.is_available())"
# 期望: ≥2.7  ≥12.8  (12, 0)  True
```

验收：四个值全对。已核实（2026-11-02）：`D:/Anaconda3/envs/` 下现有 hand-pi / fpga_amd-pi 等，**无 train-pi**；D 盘余 108 GB。

### T2 写 `sw/label_auto.py` + 打标 + 可视化抽查（1 天，跑在 hand-pi）

用 MediaPipe 当教师给 `results/w3_pose/calib_rec.mp4`（已核实存在，205 MB，1280×720，7687 帧）逐帧打标：

- **IMAGE 模式**逐帧独立推理（禁 VIDEO 模式——跟踪漂移会把错误标签成片带进数据集），`num_hands=1`
- 模型：`model/data/external/mediapipe/hand_landmarker.task`（已核实存在）
- 过滤（计划书 §4.3）：`score ≥ 0.8`；手长（p0→p9 归一化）`≥ 0.08`；与上一有效帧最大点位移 `≤ 0.15`（剔跳变）
- 输出 `results/stage1/calib_rec_labels.npz`：
  `pts (N,21,2) float32`（归一化 0~1）/ `handedness (N,)`（教师原始输出，**全局 swap 未验证**，沿用 B1 遗留注记）/ `score (N,)` / `valid (N,) bool` / `discard (N,) uint8` 原因码（0保留 1无手 2低分 3手小 4跳变）/ meta（视频、帧数、fps、mediapipe 版本、过滤参数、时间戳）
- 丢弃清单落 `results/stage1/calib_rec_discard.json`（帧号+原因，留档反查教师缺陷）
- 吞吐预期：解码+推理 ≈ 2~5 分钟跑完；丢弃率预期 10~20%，**保留率 <70% 就停下查教师**，别硬打
- **可视化抽查（主人硬规矩，优先级高于一切数字）**：抽 100 帧（首/中/尾各 12 + 均匀抽样，覆盖握拳/张开/过渡帧），教师骨架叠加拼 contact sheet 落 `results/stage1/label_check_*.png`，**贴图给主人看**，同时报保留率与丢弃原因分布

坑（必踩清单，见第 7 节）：`landmarker.close()` 卡 42 s → 抄 `pose_map.py` 的 `close_quietly()` + `os._exit()`；后台跑必须 `python -u`。

验收：npz 字段齐全可读、保留率在预期区间、主人看过 contact sheet 无异议。

### T3 训 sanity 模型（1 天，跑在 train-pi）

新文件 `tools/train_kpt.py`（+ 同目录 dataset/eval 脚本，计划书 §10 规划的路径）：

1. **数据缓存**：valid 帧 resize 到 256×256 存 `results/stage1/cache_sanity/`（npz `mmap_mode='r'` 或 JPEG q95，约 1.5 GB，磁盘已核实够）；presence 负样本 = discard 原因 1（无手）的帧（若 <100 帧，presence 头只训不评，注明）
2. **划分**：时间块状 70/15/15（train/val/test，不打乱）。注明：单视频划分只验"可训练性"，泛化留给 M2 的多视频划分（计划书 §4.4）
3. **结构**（按决策①，默认 A）：MobileNetV2 w0.75 去最后 stage、纯 ReLU6，输入 256²×3（`x/127.5-1`）→ GAP → 2 层 FC → 三头：`21×2` 坐标（sigmoid 或 clamp [0,1]）+ `presence` 1 logit + `handedness` 2 logits。参数量/MACs 记入 config（预算 ≤1.5M / ≤300M）
4. **损失**：`5.0·Wing(w=10,ε=2)`（仅 valid 帧）`+ 1.0·BCE(presence) + 0.5·CE(handedness)`（骨长正则可选，sanity 不加）
5. **配置**：AdamW lr 1e-3 / wd 1e-4，cosine + warmup 5ep，batch 64（显存富余可 128），AMP fp16，**40 epoch（5060 上约 20~40 min）**，seed 42
6. **流程纪律**：先 1 epoch 冒烟（loss 在降、shape 对、可视化脚本能出图）→ **长训前问主人一句** → 每 5 ep 存 `last.pt`/`best.pt` → config.yaml + 归档号（本项目非 git，用 `archive/` 快照习惯）落 `results/stage1/sanity_run1/`
7. **评测**（按第 3 节门槛）：NME/PCK + 任务层 5 通道（复用 `Geometry`/`PoseMapper`，参考 `sim/verify_channels.py` 的做法；离线评测按 30 fps 帧序喂 `PoseMapper.map` 的 dt）+ ORT CPU 4 线程延迟
8. **可视化（硬规矩）**：test 段 预测(红) vs 教师(绿) 叠加 **best-12 + worst-12 网格**、5 通道偏差散点/直方、连续 60 帧序列拼图 → 落 `results/stage1/sanity_run1/`，**贴图汇报**
9. 导出 `best.onnx`：`torch.onnx.export(dynamo=False)`，batch=1 固定 shape 即可（要 dynamic 必须显式 `dynamic_axes`，踩坑清单 §E）

验收：第 3 节门槛 + 分支判定执行到位 + 图已贴给主人。

### T4（可选，时间富余再做）`sw/vision_nn.py` 骨架 + A/B 开关

- `VisionNN.detect_for_video(bgr, ts_ms)` 返回 **mediapipe 同构对象**（`.hand_landmarks[0]` 21 点带 `.x/.y`；`.handedness[0][0].category_name/.score`），接口签名照计划书 §10
- `sw/pose2arm.py` 加 `--model-kind {mp,nn}`（默认 mp，现网行为零变化）+ `--nn-model results/stage1/sanity_run1/best.onnx`
- 只做 **PC 回放验证**（`--source <视频>` 或 jsonl 回放，`--no-jsonl --quiet` 口径），**不开摄像头、不接板**——真机 A/B 留下轮主人在场时做
- 改 `pose2arm.py` 前必须先读原文（它继承自 1176 行的 pose_map 血统，凭记忆写替换串会 AssertionError；改前 `cp *.bak`）

## 5. 产出物落点汇总

```
sw/label_auto.py            tools/train_kpt.py（+dataset/eval）      sw/vision_nn.py（T4）
results/stage1/
  calib_rec_labels.npz      calib_rec_discard.json   label_check_*.png
  cache_sanity/             sanity_run1/{config.yaml, best.pt, last.pt, best.onnx, eval.json, 可视化图}
  README.md                 ← 本轮结论（惯例：results/<轮次>/README.md）
```

## 6. 环境铁律（全部踩过坑，别再踩）

| 规矩 | 内容 |
|---|---|
| 环境分工 | **hand-pi** = 打标/运行时（mediapipe 1.1.1）；**train-pi** = 训练/导出（本轮新建）；fpga_amd-pi = W1 遗留，不动 |
| 污染红线 | 绝不往 fpga_amd-pi 装 mediapipe；train-pi 不装 mediapipe（用不上） |
| conda | 可执行文件是 `D:\Anaconda3\_conda.exe`，必须带 `CONDA_ENVS_PATH=D:\Anaconda3\envs`（否则建到 C 盘）；环境名 `-pi` 结尾 |
| python | 一律 `python`/绝对路径解释器；`python3` 是 Microsoft Store 空壳（无输出、退出码 0） |
| 路径 | git-bash 的 `/d/...` 不要传给 python 参数，统一 `D:/C_O_D_E/2026/FPGA_AMD/...`；相对 `--out` 先确认 cwd |
| 长脚本 | 不用 bash heredoc 内联（~9KB 截断），写到 `_scratch/xxx.py` 再执行，用完删 |
| 并发 | 开训前 `Get-CimInstance Win32_Process` 查残留训练进程（`wmic` 返回空）；两个训练绝不共享 `--out`；训练时不跑真机/GUI |
| 非 git 仓库 | 改现有文件前 `cp 文件 文件.bak`；覆盖权重前先归档旧产物 |
| 汇报纪律 | 数字必须配可视化图；下载的数据/标签必须出 contact sheet 贴图；先登记门槛再开跑；报精度必带召回口径 |
| 字符 | 文档/代码不用 `✓`（PIL 微软雅黑无此字形） |

## 7. 本轮直接相关的坑（从踩坑清单摘编，开工前过一遍原文 §0/§A/§F）

1. `mediapipe 1.1.1 landmarker.close()` 固定卡 ~42 s → `close_quietly()`（daemon 线程 + join 1.5s）+ `os._exit()` 硬退；FPS/耗时统计要在 close 之前取
2. GAP+坐标回归的空间分辨率风险（W1 检测头 25.81px 惨案）→ 已内置为 T3 分支判定（热图头备份方案），不要临场慌
3. theta 是角度量：损失与评测都要处理周期性（本轮 theta 只出现在评测端，用周期差）
4. `json.dump` 不能直接塞 numpy 标量（先 `int()/float()`）；f-string 表达式里不放反斜杠；numpy 数组 truthiness 陷阱（`if arr:` 报错）
5. 教师标签不是真值：handedness 有"镜像自拍视角"全局 swap 陷阱（B1 遗留未验证项），本轮只如实记录教师输出，不自行翻转
6. 别信文件名当标签；别在训练运行时改 dataset/transforms
7. 评测的 `n` 要说清是"有效帧数"还是"总帧数"，横向比较先对齐口径

## 8. 文件索引与前置阅读

| 文件 | 关系 |
|---|---|
| `docs/阶段1-视觉模型训练-计划书.md` | **必读**主依据（§4 数据管线 / §5 模型 / §6.3-6.4 训练与门槛 / §10 接缝 / §12 三件事） |
| `docs/调研-DPU路线可行性-Zynq7100-20261101.md` | **必读**（§5.2 = 决策①的依据；三路线分叉背景，解释为何 float 小网是共同前置） |
| `docs/踩坑清单-下一轮关键点模型.md` | 开工前过 §0/§A/§F；本文第 7 节只是摘编 |
| `docs/B1-阶段总结与下一步.md` | 控制逻辑定案（5 通道 ↔ 自由度映射） |
| `docs/接口-视觉5通道-到逆解核-确认.md` | 任务层 5 通道的精确定义 + `sim/verify_channels.py` 复现口径 |
| `sw/pose_map.py` | `Geometry` / `PoseMapper` / `load_calib` 复用源（同进程 import 方法见 `docs/交接-简易3D仿真开工.md` §3） |
| `sw/pose2arm.py` | T4 加开关的目标文件（阶段0 主链，已真机验收） |
| `results/w3_pose/calib.json` | 现场标定（任务层评测用真实零位） |
| `results/w3_pose/calib_rec.mp4` | 打标对象（7687 帧，已核实存在） |
| `archive/20261101_stage0_v2/` | 阶段0 基线快照（回退点，不动） |

## 9. 已核实的事实（2026-11-02，本机实测，非回忆）

- `results/w3_pose/calib_rec.mp4` 存在（205,272,005 B，2026-10-17）
- `model/data/external/mediapipe/hand_landmarker.task` 存在
- `sw/pose_map.py`、`sw/pose2arm.py`、`sw/pose_link.py` 存在；`tools/` 目录尚不存在（T3 新建）
- `D:/Anaconda3/envs/`：有 hand-pi、fpga_amd-pi 等 10 个环境，**无 train-pi**（T1 新建）
- D 盘剩余 108 GB；`archive/` 有 20261101_stage0_v1/v2 两个快照
