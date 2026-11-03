# W1 检测网「域内负样本」实验（2026-10-14 22:40–23:40）

**一句话**：用主人自录的 L1 空镜 + 机械臂静止做负样本重训后，检测网在**主人自己场景里的误触发从 41% 降到 0%**（conf 最高 0.057），代价是正样本中位误差 4.41 → 5.00px（门槛 8px，仍达标）。

## 1. 结论与验收

| 指标 | 基线 run3 | run4（池顺序 bug） | **run5 = 现在的冻结版** |
|---|---|---|---|
| 正样本中位误差（官方 eval） | 4.408 px | 4.367 px | **4.997 px** |
| 正样本抖动 jitter（门槛 ≤8） | 4.52 px | 4.43 px | **4.72 px ✓** |
| 正样本召回 @conf>0.5 | 95.4% | 97.0% | 95.6% |
| p90 | 55.4 px | 58.9 px | 52.6 px |
| **自录域内误触发 @0.5** | 40.5% | 76.3% | **0.0%** |
| 自录域内误触发 @0.2 / @0.3 | 76% / 68% | 96% / 94% | **0% / 0%** |
| 自录负样本 conf 中位 / 最高 | 0.426 / 0.926 | 0.674 / 0.961 | **0.008 / 0.057** |
| HaGRID no_gesture 误触发 @0.5 | 16.0% | 19.1% | 17.8% |
| 全部负样本误触发 @0.5 | 21.5% | 30.6% | **13.9%** |

- **验收判定：正样本双门槛（中位 ≤8px、抖动 ≤8px）双双通过**，且域内误触发清零 → **通过，已提升为冻结版**（`model/frozen/detect_fp32.pt`，SHA256 `bd6edde32e980257e1da026d091d8434694aa50bde5c8c4155ab9aa62e9225e5`）
- 旧基线完整存档在 `softargmax_hm/`（可一键回退）

## 2. 为什么必须做这件事（动机的实测依据）

冻结基线的检测网在**主人实验室空镜**（画面里没有手）上：conf 中位 0.5945、最高 **0.926**、@0.5 误触发 **40.5%**、@0.2 误触发 **76%**。
即"没人做手势，闭环却以为手在那里"。HaGRID 的 1839 张 no_gesture 覆盖不到主人的背景/工位/机械臂 → 必须用**域内负样本**。

## 3. 负样本构成（本次实验的自变量）

| 来源 | 帧数 | 说明 |
|---|---|---|
| L1_open_light | 41 | 训练环境空镜（含其他人、显示器、货架；光照变化 113~177 灰阶） |
| robot_arm1 | 33 | 机械臂**弯曲**姿态，静止 |
| robot_arm2 | 19 | 机械臂**直立**姿态，静止 |
| 合计（自录） | **93** | 在负样本池中重复 5 次 → 占每 epoch 负样本槽位 **20.5%** |
| HaGRID no_gesture | 1839 | 原有背景负样本 |

自录负样本一律用**部署口径窗口**（短边缩到 144 + 中心裁剪 144×144）+ 50% 概率光度抖动；
验证/评测时**不加**抖动，即"部署看到的画面"。

## 4. 踩到的坑（都已修，记录在此避免复发）

1. ⭐ **负样本池顺序 bug（最关键）**：训练每 epoch 只取池的**前** `int(len(pos)*0.4)=1781` 条，
   而自录负样本被追加在 1839 条 HaGRID 之后 → **一条都没被采到**（实测 1781 个槽位里自录条数 = 0）。
   run4 因此什么都没学到，误差反而随机漂到 76%（OOD 输入上 conf 无约束，纯随机）。
   修法：构建池后做**确定性打乱**（`random.Random(1234).shuffle`），使任意前 1781 条都按比例覆盖自录负样本。
2. **进程未杀干净**：设错环境变量的第一次训练被"杀"失败，与新进程共用日志与 `--out`，把新进程最优权重覆盖
   （导致一次假评测 16.24px）。修法：用 PowerShell 查 `CommandLine` 确认无残留后才启动新训练。
3. **cwd 陷阱**（两次）：`cd A && cmd &` 会把 `cd` 只作用于后台子壳，前台命令仍在原目录；
   `cd model/classify` 后 `--out ../results/...` 实际落在 `model/results/`（已搬回）。
   今后一律用绝对路径。
4. **评测脚本口径错误**：`eval_negatives.py` 起初把"全部正样本"当成"被接受的正样本"报成 recall@0.5 → 已改为与官方 `eval.py` 同口径。

## 5. 顺带发现的设计问题（**待主人决策**）

- **HaGRID `no_gesture` 含手**（官方定义：画面里有"第二只不做手势的手"或自然搭手姿态）。
  把它当**检测负样本**（目标 conf=0）等于教模型"有手也别报警"，与"检测=有手/无手"的语义冲突 ——
  实测仍有 17.8% 的 no_gesture 帧被报警（模型在对抗这个标签）。
  **建议**：检测负样本只用"画面确实没有手"的样本（L1 空镜 + 机械臂），
  把 no_gesture 改为检测**正样本**（有手）+ 分类 **other**，与三分类设计一致。这需要再跑一次（~11 分钟）。
- 目前域内负样本只有 93 帧、1 个工位、2 种光照。若演示时光线/机位变化大，建议再录 2–3 段。

## 6. 产物

| 路径 | 内容 |
|---|---|
| `_neg2.pt` / 冻结副本 | run5 权重（= 现冻结版） |
| `_neg1_poolbug.pt` | run4 权重（池顺序 bug 版，作为反例留档） |
| `base_softargmax_hm.pt` | run3 基线权重（与旧冻结版同一 SHA256） |
| `neg_eval_{base,neg1_poolbug,neg2}.json` | 三方「正样本 + 各源误触发」评测 |
| `detect_eval.json` / `detect_eval_poolbug.json` | 官方门槛评测（新 / bug 版） |
| `history_poolbug.json`、`train_neg{1_poolbug,2}_stdout.log` | 训练日志 |
| `eval_negatives.py` | 评测脚本（正样本指标 + 各源误触发率） |

## 7. 复现命令

```bash
# 训练（工程根）
cd model
HAGRID_ROOT=data/cache256 D:/Anaconda3/envs/fpga_amd-pi/python.exe -u detect/train.py \
    --epochs 40 --workers 2 --hm-weight 1.0 --hm-sigma 1.0 --neg-extra-weight 5 \
    --out ../results/w1_detect_neg/_neg2.pt
# 官方门槛
HAGRID_ROOT=data/cache256 D:/Anaconda3/envs/fpga_amd-pi/python.exe -u detect/eval.py \
    --weights ../results/w1_detect_neg/_neg2.pt
# 误触发率
HAGRID_ROOT=data/cache256 D:/Anaconda3/envs/fpga_amd-pi/python.exe -u \
    ../results/w1_detect_neg/eval_negatives.py --weights ../results/w1_detect_neg/_neg2.pt --tag neg2
```
