# 交接 · B1 视觉走线开工（2026-10-16 定稿）

> **给新对话的第一个动作**：读本文件 → 再读 `docs/接口-视觉侧接入原摇杆控制链.md` → 然后按第 6 节任务清单开工。
> 本文件自带全部前置结论，**已定的事项不要再重新讨论/重新论证**。

---

## 1. 当前任务（唯一授权范围）

**选项 B1**：新建独立 conda 环境 `hand-pi` 装 **MediaPipe Tasks（HandLandmarker）**，在 **PC 上**搭一条
`摄像头 → 21 点关键点 → (x, y, θ, φ, gesture) → POSE1 指令行` 的**映射 + 可视化走线**。

- **不训练**任何模型（模型训练是选项 A，本轮不做）
- **不接板子**（先只打印/存文件，验证走线通）
- 目的是先试手感、定标定参数，同时产出 A 之后也要用的映射层代码

## 2. 为什么先 B 不做 A（已定）

1. B 的产物（映射层 + 零位/符号/增益标定参数）A 之后一样要用
2. B 能立刻暴露手感问题（手转多少 → 腕转多少、死区、pitch 软限幅），这些只能真机试出来，凭想象定会白训一轮
3. A 卡在**左右手标签**：HaGRID 的 `meta` 只有 age/gender/race，**没有左右手**；MediaPipe Tasks 的 `handedness` 可伪标，但有"镜像自视"全局符号陷阱（见第 5 节），需要一次验证

## 3. 已定结论（不要再改、不要再提）

| 项 | 结论 |
|---|---|
| 模型现状 | 检测 DetectNet 冻结 run-5（≈5.0px，5.4M MACs 级小网）；分类 ClassifyNet 3 类 92%。四次精度实验（OOB 负样本 / QAT / sigma 2.0 / zoom 0.60）全部负结果，已到 HaGRID 标签噪声底（≈8-11px），**不要再试图靠公开数据提精度** |
| 下一步主线 | W3：自己训 **21 点关键点模型**（目标板跑不了 MediaPipe，所以推理必须自训；MediaPipe 只作 PC 侧对照/伪标工具） |
| 板子 | 已从 Zynq7020 换成 **Zynq-7100**，资源宽松（444K LC / 2020 DSP / 26.5Mb BRAM） |
| 手势语义 | 张开(palm) / 闭合(fist) / **其他**；其他手势**夹爪必须不动** |
| 手部姿态 | **两个自由度都用**：`wrist_flex`(ID4, 俯仰 pitch) + `wrist_roll`(ID5, 绕前臂轴 roll) |
| yaw | **由位置反算** `yaw = atan2(y, x-0.0612)`（并选靠近当前 yaw 的 π 分支）——**绝不由手给** |
| 爪子 | 走 `POSE1` 第 7 个字段 `gripper(0..1)` → ID6 counts `1975..3567`；**不进逆解**（原设计如此，不是缺陷） |
| IK 核 | **一行不改**。`FPGA_AMD/fpga/diag_fw/firmware/ik_iteration.c` + PL IP `xf_solver_lm_so101_0/1` |
| 已否决 | ① "学手→关节角、跳过 IK"（主人明确否决，IK 核必须复用）② YOLO26-depth 单目深度（主人实测判定"深度不可用"）③ 手部关键点不需要用于抓取控制的老结论已被新需求取代 |

## 4. 角度/手势的算法定义（不训练，全部几何后处理）

用 MediaPipe Tasks 的 21 点（索引：0=腕，1-4 拇指，5-8 食指，9-12 中指，13-16 无名，17-20 小指；**Tasks 的 21 点顺序与 HaGRID 标注一致**）：

```
θ（画面内旋转） = atan2( (p9 - p0).y , (p9 - p0).x )        # 注意图像 y 向下，符号需实测确认
掌面法线 n      = cross(p5 - p0, p17 - p0)                  # 用手 world_landmarks 更稳
φ（腕部俯仰）   = atan2( n_z 分量 , 面内投影长度 ) 一类的倾角量   # 具体形式待现场标定
伸指数          = Σ_finger [ |tip - p0| > |PIP - p0| + margin ]   # 张开=5, 闭合=0, OK=3, 剪刀=2
(x, y)          = 掌心中心（p0,p5,p9,p13,p17 均值）经仿射标定映射到工作空间
```

**手性/翻转**：`n` 的方向（配合 handedness）区分掌↔背；配合 handedness 可消除 2D 的镜像歧义。

## 5. MediaPipe Tasks 的两个额外红利 + 一个陷阱

1. **`handedness`（左右手）** → 补 HaGRID 没有左右手标签的缺口
   ⚠️ **陷阱**：官方文档明确"handedness 假定输入是**镜像的自拍视角**，非镜像图需自行交换左右"。HaGRID 是否镜像未知 → 伪标可能**整体反了**（不是随机错）。**必须验证一次**：用主人自录的一小段（已知左右手）跑一遍，看是否需要全局 swap。
2. **`hand_world_landmarks`（米制 3D，相对手掌中心）** → 算掌面法线定 roll/pitch 的符号与零位，比纯 2D 稳。**但 z 精度有限，只能当标定参考，不能当深度用**。

## 6. 本轮任务清单（B1）

> **状态 2026-10-17**：第 1/2/3/4/5/6/7 项已完成并实测；第 8 项（handedness 全局 swap）与真机标定仍等主人提供素材/配合。
> 使用与标定细节见 `docs/B1-视觉走线-使用与标定.md`。

- [x] **建环境**：`CONDA_ENVS_PATH=D:\Anaconda3\envs D:\Anaconda3\_conda.exe create -n hand-pi python=3.10 -y`
  （conda 可执行文件名是 `_conda.exe`；**环境名必须 `-pi` 结尾**）
- [x] **装包**：`D:\Anaconda3\envs\hand-pi\python.exe -m pip install mediapipe`（走清华镜像）→ 装到 **mediapipe 1.1.1 + opencv-contrib-python 5.0.0.93**
  装完**必须验证**：`import cv2` 的 `getBuildInformation()` 里 `GUI` 不能是 `NONE`；若是 NONE →
  `pip uninstall opencv-contrib-python -y && pip install opencv-python`
  （教训：`opencv-python-headless` 会让 `cv2.imshow` 直接崩，10-15 已踩过一次）
- [x] **下模型**（存项目内，别放系统缓存）：
      `curl -L -o model/data/external/mediapipe/hand_landmarker.task https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task`
      已验证 **HTTP 200** 可下（`/latest/` 路径也可用）
- [x] **写 `sw/pose_map.py`**：摄像头 → `HandLandmarker`（VIDEO 模式，`detect_for_video` 需单调递增时间戳）→ 第 4 节几何 → 组 `POSE1 <seq> 1 x y z roll pitch yaw gripper` 字符串
      - 必须做**固件硬范围夹紧**：x∈[-0.5,0.6]、|y|≤0.6、z∈[-0.3,0.7]、|roll|≤π、**|pitch|≤1.4**、gripper∈[0,1]（越界固件直接 `ERR1` 拒绝）
      - pitch 先做**软限幅 ±0.6~0.9 rad**，别去顶 1.4 硬限
      - 输出：窗口叠加骨架 + θ/φ/伸指数 + 生成的 POSE1 行；同时写 `results/w3_pose/pose_run_<时间>.jsonl`
- [x] **标定模式**：采一次"零位"（哪个手姿 = roll0/pitch0）+ 定符号 + 定增益（手转 1° → 腕转几度）+ 死区（建议 3-5°）
- [x] **时域平滑**：角度必须在 **sin/cos 分量**上做 EMA（直接平滑角度会在 ±180° 撕裂）
- [x] **GUI 自检**：沿用 `sw/live_demo.py` 的做法——启动时用 8x8 dummy 试 `namedWindow+imshow`，失败退出并给修复提示；用 `getWindowProperty` 检测窗口被 × 关掉

## 7. 主人要提供的东西（**仍未提供，是当前唯一阻塞**）

1. **一小段自录视频**（已知左右手、已知张开/闭合）→ 验证 handedness 是否需要全局 swap
2. 标定采集时配合做几个固定手姿（零位、roll 正向、pitch 正向）

具体怎么录/怎么按见 `docs/B1-视觉走线-使用与标定.md` 第 5 节；摄像头已实测可用（`--source 0`，DSHOW 640×480）。

## 8. 环境与铁律（踩过坑，别再踩）

- 环境：`fpga_amd-pi`（Python 3.10.22, torch 2.11+cu128, RTX 5060），**绝不能被污染**——所以本轮另建 `hand-pi`
- 禁止把 `mediapipe` 装进 `fpga_amd-pi`：pip 会带进 `opencv-contrib-python 5.0.0.93`，顶掉现有 `opencv-python 5.0.0.93`（该包 10-15 刚修好，当前 `GUI: WIN32UI` 正常）
- 所有文件留在项目目录内（代码/临时/日志/产物），临时文件用 `_scratch/` 且用完即删
- **主人的两条硬规矩**：① 下载数据集后要**打开图给他看**（出 contact sheet）② 测试完要**给他看预测结果图**，不能只报数字
- 后台跑 GUI 脚本必须 `python -u`，否则重定向日志是空的
- CUDA tensor 不能直接 `np.clip` → `x.detach().cpu().numpy()`
- 长脚本不要在 bash heredoc 里内联（会被截断）→ 先写文件再执行
- `--out` 之类相对路径要对照实际 cwd（`cd X && ...` 在管道里是子 shell，cd 不生效）

## 9. 文件索引

| 文件 | 内容 |
|---|---|
| `docs/接口-视觉侧接入原摇杆控制链.md` | 原摇杆链路、POSE1 字段、固件硬范围、自由度计数、两条通道对比（**必读**） |
| `docs/调研-手势遥操开源项目.md` | 开源项目调研、IK 核接口、π 退化实测数据 |
| `docs/踩坑清单-下一轮关键点模型.md` | 80+ 条"不要…"，下一轮训练的避坑清单 |
| `docs/网络结构清单.md` | 冻结版 v1.3（v2.0 待关键点模型定稿后升版） |
| `fpga/diag_fw/firmware/pose_uart.inc` | 固件侧 POSE1 解析 + 4×4 目标位姿构造 + 爪字段 |
| `refs/26FPGAlerobot/rk7020_so101_xbox_*/host/right_joycon.py` | 主机侧 yaw 反算 + 位置/旋转限幅的参考实现 |
| `sw/live_demo.py` | 现有实时 demo（GUI 自检/中文显示/EMA/3 帧投票可复用） |
| `sw/pose_map.py` | **B1 主脚本**：HandLandmarker → 几何 → 标定 → POSE1 行（打印+jsonl+窗口+标定模式） |
| `sw/pose_sheet.py` | 离线图片自检（IMAGE 模式 → 标注 contact sheet + 伸指数/手性统计） |
| `docs/B1-视觉走线-使用与标定.md` | **本轮使用说明**：环境、命令、按键、标定流程、实测数据、新增坑 |
| `model/frozen/` | detect_fp32.pt（≈5.0px）、classify_fp32.pt（3 类 92%） |

## 10. A 轮的预登记（本轮不做，但别忘了）

A 开工前必须先**预登记验收门槛**（PCK 阈值 + 按手大小/关键点分组），再训练——上一轮的教训就是"事后挑指标"（官方 median 会因 conf 阈值漂移而虚高，必须同时报 recall）。

**A 的最大不确定项仍是左右手标签来源**：MediaPipe 伪标（需验证第 5 节的全局符号） vs 主人自录。**没有第三个来源。**
