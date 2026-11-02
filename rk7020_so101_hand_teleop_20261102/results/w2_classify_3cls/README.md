# 分类网改三分类（张开 / 闭合 / 其它）——2026-10-14 23:30–23:40

**一句话**：按主人口径把分类器从"拳/掌二分类"改成"**张开 / 闭合 / 其它**"，其它手势（竖拇指/OK/剪刀手/自然搭手/无手背景）一律判为"其它 → 夹爪不动"；验证准确率 **92.00%**（门槛 90% ✓），**其它手势误动作率 7.11%**（我建议的 ≤5% 未达成，错集中在 OK / 剪刀手）。

## 1. 语义（主人 2026-10-14 定义）

| 类别 | 含义 | 动作 |
|---|---|---|
| 0 `fist` | 五指闭合（握拳） | 夹爪闭合 |
| 1 `palm` | 五指张开 | 夹爪张开 |
| 2 `other` | **其它一切**：竖拇指/OK/剪刀手/自然搭手/无手背景 | **夹爪不动** |

## 2. 数据（`model/data/hagrid_subset/dataset.json` v2，含 `group` 字段）

| 来源 | 类 | train | val | 说明 |
|---|---|---|---|---|
| HaGRID `fist` | fist | 2254 | 259 | 原有 |
| HaGRID `palm` | palm | 2200 | 305 | 原有 |
| HaGRID `like`（竖拇指） | other | 219 | 28 | **本次新下载 300 张** |
| HaGRID `ok` | other | 235 | 23 | **本次新下载 300 张** |
| HaGRID `peace`（剪刀手） | other | 231 | 29 | **本次新下载 300 张** |
| HaGRID `no_gesture`（自然搭手） | other | 1839 | 325 | 原有 2164 张，本次补上标注框 |
| 自录负样本帧（L1 空镜 + 机械臂） | other | 62 | 31 | 模拟"检测误触发"时送进分类器的 ROI |
| **合计** | | **7040** | **1000** | other 占 2586 / 436 |

- 三类训练样本量基本均衡：fist 2254 / palm 2200 / other 2586
- **数据来源权威性**：`like/ok/peace` 取自 **HaGRIDv2 官方发布**（SberCloud 直链，ai-forever/hagrid，arXiv 2412.01508），
  与已有 fist/palm **同源同标注体系**（比 Roboflow 上任意项目更一致、可复现）；
  用 `model/data/extract_zip_range.py` 从 43 GB 类 zip 里按 HTTP Range 抽 300 张（每类 ~2 分钟）。
  标注用官方 `annotations.zip`（686 MB，本次重新下载，已恢复）。
- 划分沿用官方 train/val/test（按 `user_id`）；`no_gesture` 沿用原 85/15 按文件名切分，保证检测侧 val 负样本集不变。

## 3. 模型与训练

- `ClassifyNet`：stem12→32→64→64，80×80 输入，GAP 后 1×1 头 **96→3**（原 96→2）
- **4.781 M MACs**（三分类头只比二分类多 288 MACs）、15.7 K 参数 → 总预算 21.06+4.78 = **25.84 M ≤ 30 M**
- 训练：40 epoch、batch 128、lr 3e-3、AdamW + cosine，**379.6 s**（6.3 分钟），best = epoch 28
- 图源：`HAGRID_ROOT=data/cache256`（256px 短边镜像缓存；本次已把新三类补进缓存，缓存 207 MB）

## 4. 评测（val 1000 张）

| 指标 | 值 | 门槛 | 判定 |
|---|---|---|---|
| 总体准确率 | **92.00%** | ≥90% | ✅ |
| 张开 `palm` recall | 92.13%（281/305） | ≥90% | ✅ |
| 闭合 `fist` recall | 90.35%（234/259） | ≥90% | ✅ |
| **其它 → 误动作率** | **7.11%**（31/436 被判成拳头/张开） | ≤5%（**我建议的**，计划书尚未定义） | ❌ |
| `other` recall | 92.9%（405/436） | — | — |

**误动作按来源拆解（这是关键）**：

| 其它来源 | n | 误动作率 | 主要错法 |
|---|---|---|---|
| `ok` | 23 | **39.1%** | 8 张判成"张开" |
| `peace` 剪刀手 | 29 | **37.9%** | 6 张判成"拳头"、5 张判成"张开" |
| `like` 竖拇指 | 28 | 7.1% | 2 张判成"拳头" |
| `no_gesture` 自然搭手 | 325 | 2.8% | 4→拳、5→张 |
| **自录背景/机械臂** | 31 | **0.0%** | — |

→ OK/剪刀手在手形上离"五指张开"最近，300 张样本还不够把边界学开；
   背景/机械臂与自然搭手几乎不会误动作（这两类才是闭环里最容易撞上的情况）。

## 5. 与旧二分类模型对比（旧模型 = `baseline_2cls/`）

| 模型 | "其它手势"上的误动作率 | 说明 |
|---|---|---|
| 旧二分类（2026-10-13 冻结） | **100%**（436/436） | 它没有"其它"类，任何输入必输出拳或掌 → 夹爪一定会动 |
| **新三分类** | **7.11%** | 已能把 92.9% 的其它手势按住不动 |

- ⚠️ 旧模型的 93.09% 与新模型的 92.00% **不可直接比较**：验证集不同（564 二分类 vs 1000 三分类），
  且旧模型的 93.09% 是在**原图**上评测，而本次在 **cache256** 口径下重测旧模型只有 **90.07%**
  → 说明"训练图源 / 评测图源不一致"本身就值 ~3%，**图源口径需要统一**（见待决项）。

## 6. 待主人决策

1. **扩大其它手势样本**：`like/ok/peace` 从 300 提升到 1200–2000/类（每类下载 ~8–15 分钟，脚本现成），
   预计可把 OK/剪刀手的误动作率从 ~38% 压到 5% 以内。
2. **时序兜底**：闭环要求手势连续 K 帧一致才动作（比如 3 帧≈100 ms → 64 ms 内不动）。
   零 MACs 成本，可把"偶发误判"挡掉，但会加 ~0.1 s 延迟。**建议做**（FPGA/HLS 侧一个计数器即可）。
3. **图源口径**：分类器该在原地（1920px）裁 ROI 还是在 256px 缓存裁？部署时 ROI 来自相机原帧（~100–200px），
   与 256px 口径更接近 → 建议统一到 cache256，并把旧模型的 93.09% 标记为"原图口径，不可比"。
4. **《网络结构清单》需升版 v1.1**：分类头 2→3 通道、两个权重 SHA256 变化（见该文档）。

## 7. 产物与复现

| 路径 | 内容 |
|---|---|
| `_3cls.pt` / 冻结副本 `model/frozen/classify_fp32.pt` | 三分类权重（SHA256 `1e22b880b1ab696e851a7a6e391f2bfa19e330db38229a2e352da437ef5da910`） |
| `classify_train_{history,meta}.json` | 训练曲线（本次同时补齐了历史缺失的训练记录）与数据分布元信息 |
| `classify_eval.json` | 三分类评测（3×3 混淆矩阵、按来源误动作率） |
| `eval_2cls_on_other.py` / `eval_2cls_on_other.json` | 旧二分类模型在"其它手势"上的对照实测 |
| `baseline_2cls/` | 旧二分类权重与评测存档（可回退） |

```bash
# 数据（工程根）
cd model/data && python -u extract_zip_range.py --url "https://rndml-team-cv.obs.ru-moscow-1.hc.sbercloud.ru/datasets/hagrid/hagrid_dataset_new_554800/hagrid_dataset/ok.zip" --class ok --count 300 --out hagrid_subset/ok --workers 6
python -u prepare_dataset.py            # 重建 dataset.json（含 group 字段）
cd ../.. && python model/data/build_cache.py   # 把新类补进 cache256
# 训练 + 评测
cd model/classify && HAGRID_ROOT=data/cache256 python -u train.py --epochs 40 --out ../../results/w2_classify_3cls/_3cls.pt
HAGRID_ROOT=data/cache256 python -u eval.py --weights ../../results/w2_classify_3cls/_3cls.pt
```
