# 本项目训练环境说明

> 建立日期：**2026-10-14**　按主人全局规则：每个项目独立建环境、conda 环境名以 `-pi` 结尾。

> **第二个环境（2026-10-17 新增）**：`D:\Anaconda3\envs\hand-pi\python.exe`
> —— B1 视觉走线专用（mediapipe 1.1.1 + opencv-contrib-python GUI）。
> **原理由**：mediapipe 会带进 `opencv-contrib-python`，顶掉 `fpga_amd-pi` 里刚修好的 `opencv-python`。
> 两个环境**绝不能互相污染**；hand-pi 的完整说明见 `docs/B1-视觉走线-使用与标定.md`。

## 一句话
训练**一律用这个环境的绝对路径 python**：
```
D:\Anaconda3\envs\fpga_amd-pi\python.exe
```
**不再用**系统 `C:\Python310`（那是旧环境的遗留，仅作备份）。

---

## 1. 环境基本信息

| 项 | 值 |
|---|---|
| conda 环境名 | **`fpga_amd-pi`** |
| 位置 | `D:\Anaconda3\envs\fpga_amd-pi`（4.6 GB） |
| Python | 3.10.22 |
| 解释器 | `D:\Anaconda3\envs\fpga_amd-pi\python.exe` |
| pip | 26.2.1 |
| 包数量 | 31（精简，只装训练与导出所需） |

## 2. 已装的关键包

| 包 | 版本 |
|---|---|
| torch | **2.11.0+cu128** |
| numpy | 2.2.6 |
| opencv-python-headless | 5.0.0.93 |
| onnx | 1.23.2 |
| onnxruntime | 1.23.2 |
| onnxslim | 0.1.98 |

GPU：**RTX 5060 Laptop**，`torch.cuda.is_available()` = **True**，cuDNN 91900。

## 3. 验收记录（2026-10-14，用项目真实代码冒烟测试）

```
数据集加载 OK: detect train=6235, classify train=4454
batch 形状: (8, 3, 144, 144) (8, 3) | (8, 3, 80, 80) (8,)
检测网 3 步训练 OK (前向+反传+优化器)
分类网 前向 OK, 输出 (8, 2)
```
→ 环境可用，能直接跑本项目训练/评估代码。

## 4. 怎么用

**推荐：绝对路径调用**（不依赖 conda init / activate）
```bat
cd /d D:\C_O_D_E\2026\FPGA_AMD\model
D:\Anaconda3\envs\fpga_amd-pi\python.exe detect\train.py --epochs 40 --workers 2
D:\Anaconda3\envs\fpga_amd-pi\python.exe classify\eval.py
```

**或者 activate**（需要 shell 里先跑过 conda init）
```bat
D:\Anaconda3\_conda.exe activate fpga_amd-pi
```
注意 conda 可执行文件叫 **`_conda.exe`**（带下划线），不是 `conda.exe`。

## 5. 重建命令（万一环境损坏）

```bat
:: 1) 建环境 —— 必须带 CONDA_ENVS_PATH，否则会建到 C 盘（C 盘只剩 18G）
set CONDA_ENVS_PATH=D:\Anaconda3\envs
D:\Anaconda3\_conda.exe create -n fpga_amd-pi python=3.10 -y

:: 2) 装 torch（cu128，约 4 分钟）
D:\Anaconda3\envs\fpga_amd-pi\python.exe -m pip install torch==2.11.0 --index-url https://download.pytorch.org/whl/cu128

:: 3) 装其余
D:\Anaconda3\envs\fpga_amd-pi\python.exe -m pip install "numpy<3" opencv-python-headless onnx onnxruntime onnxslim
```

⚠️ **踩过的坑**：直接运行 `D:\Anaconda3\_conda.exe create -n ...` 时，conda 会把用户环境目录认成
`C:\Users\YUBEN\.conda\envs`（C 盘只剩 18 GB），第一次就建错地方了。必须显式设置
`CONDA_ENVS_PATH=D:\Anaconda3\envs`，让环境和其他环境一起放在 D 盘。

## 6. 相关规则（来自全局 AGENTS.md，2026-10-14 起）

1. 项目所有文件（脚本、临时文件、缓存、日志、下载物、训练产物）**必须写在项目目录内**，临时文件用项目内 `tmp/` 或 `_scratch/` 且用完即删。
2. 严禁写到项目外（`D:\tmp`、`AppData\Local\Temp`、用户主目录等）。
3. 每个项目独立建环境，conda 环境名必须以 `-pi` 结尾。
4. 训练产出位置：`model/frozen/`（`detect_fp32.pt` + `detect_train_history.json` 等）。
