# 环境与资产清点报告

日期：2026-10-14　清点人：pi（本机实测，非回忆）

---

## 0. 四个问题的直接回答

| 问题 | 答案 |
|---|---|
| 训练环境装进系统了吗？ | **是，装在系统里**——`C:\Python310`，不是工程内的 venv |
| 训练脚本在工程目录吗？ | **是**，全在 `model/` 下 |
| 数据集在工程目录吗？ | **是**，全在 `model/data/` 下（9.4GB） |
| 有没有乱放？ | 有。工程内 3 处是我弄的**已清掉**；1 处 1.18GB 待你定夺；工程外 4 处（§4） |

---

## 1. 训练环境（在系统里，不在工程里）

| 项 | 值 |
|---|---|
| 解释器 | `C:\Python310\python.exe`，Python **3.10.11** |
| 是否 venv | **否**（系统级安装，`sys.prefix == C:\Python310`） |
| 包目录 | `C:\Python310\lib\site-packages` |
| torch | **2.11.0+cu128** @ `C:\Python310\lib\site-packages\torch` |
| torchvision | 0.26.0+cu128 |
| numpy / OpenCV | 2.2.6 / opencv-python-headless 5.0.0.93 |
| ONNX | onnxruntime 1.23.2、onnxslim 0.1.96 |
| GPU | RTX 5060 Laptop 8GB，`torch.cuda.is_available()` = **True** |

**本机有 4 个 Python**（`py -0p`）：3.15（py 启动器默认）/ 3.14 / 3.13 / **3.10（工程实际用的）**。
所以训练命令必须用 `python`（`which python` → `/c/Python310/python`），不能用 `python3`（那是 Microsoft Store 空壳，无输出、退出码 0）。

**工程内没有任何 venv**（`find` 全工程无 `pyvenv.cfg`；全盘只在 `~/.workbuddy/binaries/python/envs/default` 找到一个与本工程无关的）。

> 风险提示：环境在系统里 → 换机器/重装要重来，其他 Python 工程也可能污染这一套包。
> 要隔离的话我可以给工程建 `model/.venv`（只新增，不动现有环境），你说了才做。

---

## 2. 工程内资产清单（体积实测）

| 路径 | 体积 | 说明 |
|---|---|---|
| `PLAN.md` | 12K | 执行计划 v1.0 |
| `项目计划书-Zynq7020手部遥操作SO101.md` | 16K | 你的原方案 |
| `docs/` | 4K | `w1_d1_petalinux_checklist.md` |
| `fpga/` | 159K | 诊断固件补丁等 |
| `sw/` | 331K | `sw/diag/run_w1_d1.py` 等 |
| `refs/26FPGAlerobot/` | 13M | 原仓库只读副本 |
| `results/` | 1K | **只有** `w1_diagnostics/README.md`（零实测数据） |
| `.pi/meta/` | 28K | 持久文档（GOAL/LEDGER/STATUS/ROADMAP/训练方案） |
| `model/` | **9.4G** | 见下 |

**训练脚本（全在 `model/`，共 12 个 .py）**
```
model/detect/   train.py  eval.py  net.py  dataset.py  __init__.py
model/classify/ train.py  eval.py  net.py  dataset.py  __init__.py
model/common/   transforms.py  __init__.py
```

**权重（`model/frozen/`）**
```
classify_fp32.pt       84,591 B   ✅ 分类网（val acc 93.09%）
classify_eval.json        190 B   ✅ 复测记录
detect_fp32.pt            不存在  ❌ 检测网一次都没训过
```

**数据集（`model/data/`，9.4GB）**
```
hagrid_subset/     8.3G   fist 2999 / palm 2999 / no_gesture 2164
                          dataset.json: train 6293 / val 889 / test 980
                          （另有 extract_manifest.json、summary.txt）
annotations.zip    686M   ← 待清理（§4）
no_gesture.zip     494M   ← 待清理（§4）
数据脚本 4 个：prepare_dataset.py / split_recorded.py / extract_zip_range.py / README.md / RECORD_GUIDE.md
```

⚠️ **自录数据目录 `model/data/recorded/` 根本不存在**——更正我之前的记录（我写成"空目录"）。也就是说自录素材还没提供，`split_recorded.py` 还没用武之地。

---

## 3. 训练会写到哪里（全部在工程内，不会外溢）

| 产出 | 路径（脚本里的默认值） |
|---|---|
| 检测权重 | `model/frozen/detect_fp32.pt`（`--out` 默认 `ROOT/frozen/detect_fp32.pt`） |
| 检测训练记录 | `model/frozen/detect_train_history.json` |
| 分类权重 | `model/frozen/classify_fp32.pt` |
| 分类训练记录 | `model/frozen/classify_train_history.json` |
| 数据集根 | `model/data`（`IMG_ROOT = model/data`） |
| 默认超参 | detect: epochs 40 / batch 64 / workers 2；classify: epochs 40 / batch 128 / workers 2 |

---

## 4. 乱放清单

### 4.1 已清理（都是我造成的，2026-10-14 清理完毕）
| 位置 | 是什么 | 处置 |
|---|---|---|
| `model/data/_bench_cache2/`（20 张）、`_bench_cache3/`（20 张） | 我做性能基准时建的缓存，脚本异常退出没删干净 | **已删** |
| `C:\Users\YUBEN\AppData\Local\Temp\bench_pipe.py` | 我的基准测试脚本，放到了工程外 | **已删** |
| `/tmp/lc/.pi/meta/` | 测扩展时 pi 在临时目录自动生成的 | **已删** |

### 4.2 待你确认（我没敢自己删）
| 位置 | 体积 | 说明 | 建议 |
|---|---|---|---|
| `model/data/annotations.zip` + `no_gesture.zip` | **1.18 GB** | HaGRID 原始压缩包。抽取脚本 `extract_zip_range.py` 走 HTTP Range 远程取，**本地留着没用**（数据已解到 `hagrid_subset/`） | 删，你点头我就删 |
| `subsample.log`（工程根） | **0 字节** | 空文件 | 删 |
| `D:\tmp\x.jpg` | 13K | 144×144 JPEG，10-13 15:28，**疑似**我调试时留下的 | 删？ |
| `D:\tmp\pi-github-repos\` | 0 字节 | 空目录，pi 抓 GitHub 时的临时目录 | 删 |

### 4.3 不是本项目/不是我建的（仅报备，不动）
| 位置 | 说明 |
|---|---|
| `D:\tmp\mcp.log`（3.7KB） | **Trae CN 的 byted-solo MCP 写的**，与训练无关 |

---

## 5. 工程外的本项目相关文件（都是配置，不是数据）

| 位置 | 体积 | 说明 |
|---|---|---|
| `~/.pi/agent/extensions.disabled/` | 112K | 昨天按你指令删掉的 `bash-guard` + `ask-user-question.ts` 的备份 |
| `~/.pi/agent/skills/` | 空 | 之前那 4 个 skill（含 72MB venv）已删干净 |
| `~/.pi/agent/npm/` | — | pi 的扩展包，与工程无关 |

---

## 6. 结论

1. **训练脚本、数据集、权重、结果，全部在 `D:\C_O_D_E\2026\FPGA_AMD` 内**；训练过程中不会往工程外写任何东西（产出落在 `model/frozen/`）。
2. 唯一"外置"的是 Python 运行环境（系统级 `C:\Python310`）——这是历史选择，不是散落；要隔离我可以建工程内 venv。
3. 我确实乱放过 3 处（都已被我清掉），另有 1.18GB 的包和几处小东西等你定夺。已报备，没有隐瞒。
4. 顺带更正两处我之前记错的事实：`model/data/recorded/` 不存在（不是"空目录"）；`results/w1_diagnostics/` 零实测数据。
