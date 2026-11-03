# B1 视觉走线：使用与标定（2026-10-17 建）

> 任务定义与已定结论见 `docs/交接-B1-视觉走线开工.md`；字段口径见 `docs/接口-视觉侧接入原摇杆控制链.md`。
> 本文只写"怎么跑、怎么标、已验证到什么程度"。

---

## 1. 环境（独立，**不要混 fpga_amd-pi**）

| 项 | 值 |
|---|---|
| conda 环境名 | **`hand-pi`**（Python 3.10） |
| 位置 | `D:\Anaconda3\envs\hand-pi` |
| 解释器 | `D:\Anaconda3\envs\hand-pi\python.exe` |
| mediapipe | **1.1.1**（Tasks API：`vision.HandLandmarker`，已无 `mp.solutions`） |
| cv2 | **5.0.0.93 `opencv-contrib-python`，`GUI: WIN32UI`**（可用 `imshow`） |
| numpy | 2.2.6 |
| 模型 | `model/data/external/mediapipe/hand_landmarker.task`（7.82 MB，float16/1） |

重建命令（如需）：
```bat
set CONDA_ENVS_PATH=D:\Anaconda3\envs
D:\Anaconda3\_conda.exe create -n hand-pi python=3.10 -y
D:\Anaconda3\envs\hand-pi\python.exe -m pip install mediapipe -i https://pypi.tuna.tsinghua.edu.cn/simple
D:\Anaconda3\envs\hand-pi\python.exe -c "import cv2; print([l for l in cv2.getBuildInformation().splitlines() if 'GUI' in l])"
```
`GUI` 若是 `NONE` → `pip uninstall -y opencv-contrib-python opencv-python-headless` 再 `pip install opencv-python`。

模型下载（已验证 HTTP 200）：
```bat
curl -L -o model/data/external/mediapipe/hand_landmarker.task ^
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
```

## 2. 文件

| 文件 | 作用 |
|---|---|
| `sw/pose_map.py` | **主脚本**：摄像头 → HandLandmarker(VIDEO) → 几何 → 标定映射 → `POSE1` 行；窗口叠加骨架/角度/伸指数/指令行；写 jsonl |
| `sw/pose_sheet.py` | 离线自检：对图片目录跑 IMAGE 模式，出标注 contact sheet + 分类统计（不用摄像头） |
| `results/w3_pose/calib.json` | 标定参数（零位/符号/增益/死区/手性），按 `w` 保存；不存在则用默认值 |
| `results/w3_pose/pose_run_<时间>.jsonl` | 每帧一行：θ/φ/ψ/法线/掌心 uv/伸指数/手势/夹紧标记/POSE1 |
| `results/w3_pose/pose_session_<时间>.json` | 一次会话摘要（帧数、FPS、检出率、手势与手性分布） |

## 3. 怎么跑

```bat
cd /d D:\C_O_D_E\2026\FPGA_AMD

REM 实时（摄像头 0，带窗口；标定就用这条）
D:\Anaconda3\envs\hand-pi\python.exe -u sw\pose_map.py --source 0

REM 无窗口自检（headless，跑 60 帧就退）
D:\Anaconda3\envs\hand-pi\python.exe -u sw\pose_map.py --source 0 --no-window --max-frames 60

REM 整段录像 + 标注视频（事后逐帧看）
D:\Anaconda3\envs\hand-pi\python.exe -u sw\pose_map.py --source 0 --record results\w3_pose\rec.mp4

REM 离线视频回放（主人给的自录视频就用这条，先验证 handedness）
D:\Anaconda3\envs\hand-pi\python.exe -u sw\pose_map.py --source C:\...\clip.mp4 --no-window --max-frames 0

REM 离线图片自检（出 contact sheet，不需要摄像头）
D:\Anaconda3\envs\hand-pi\python.exe -u sw\pose_sheet.py --classes palm fist like ok peace no_gesture --per-class 6
```

`POSE1` 输出示例（每帧一行，直接可以喂给固件）：
```
POSE1 96 1 0.26489966 0.0461103724 0.2383 -0.5747 -0.155112268 0.633169405 1
      ^seq ^core(PL核)   x        y        z      roll     pitch        yaw     gripper
```

### 3.1 选摄像头（2026-10-17 新增外接摄像头后）

```bat
REM 枚举：打印所有能出帧的序号（外接一般排在后面）
D:\Anaconda3\envs\hand-pi\python.exe -u sw\pose_map.py --list-cams

REM 用枚举到的最后一个（通常就是外接）
D:\Anaconda3\envs\hand-pi\python.exe -u sw\pose_map.py --source auto

REM 指定序号 / 指定分辨率
D:\Anaconda3\envs\hand-pi\python.exe -u sw\pose_map.py --source 1 --width 1280 --height 720
```
本机实测：`0` = 内置(640x480)，`1` = 外接(可到 1280x720)，后端均 DSHOW。

### 3.2 界面（2026-10-17 改版：符号 + 分列对齐 + 0.1 精度）

左上角深色面板三段，每段三列，数值右对齐、单位在值里：

```
① 几何      θ 画面内旋转   φ 腕俯仰   ψ 法线3D诊断
            θ -86.1°      φ +23.2°   ψ -21.2°
            u 横 77.9%    v 纵 40.4%  手长 65.9px
② POSE1     x 前后 y 左右 z 上下（已夹硬范围+限速）
            x 27.5cm      y 6.1cm    z 23.8cm
            roll +0.8°    pitch -6.5° yaw +37.9°
            状态 slew_xyz
③ 标定      零位θ0 +0.0° φ0 +0.0° u0/v0 50.0/50.0
            roll sgn+1 g1.0 dz4°   pitch sgn+1 g1.0 dz4° s0.8
```
底部三行：`原始行 POSE1 …`（完整下发串，未取整）+ 两行按键提示；右下角仍是 roll/pitch 限位条。

**按键有两个通道**（2026-10-17 补）：视频窗口有焦点时走 `cv2.waitKey`，控制台窗口有焦点时走 `msvcrt`；
每次按键日志都会打 `[键] c` 这类行，便于确认到底有没有收到。

### 3.3 旋转源（2026-10-17 定：不用翻腕）

| 通道 | 默认源 | 含义 | 换源 |
|---|---|---|---|
| **roll** | `theta` | 画面内旋转（翻腕/转腕都表现为同一个"转"） | `t` → `psi`（法线出平面，需斜掌 45°） |
| **pitch** | `v` | **掌心纵向位置**（抬/落手），不是腕部角度 | `u` → `phi`（旧的翻腕映射） |

原因见 `docs/研究-翻腕与转腕的可分性.md`：掌正对镜头时翻腕与转腕是同一根旋转轴，单目必然退化。
默认参数：`pitch.v_scale_deg=90`（整幅上下移动≈90°）、`gain=0.6`、`deadzone_deg=3`。

CLI 等价写法：`--roll-source theta|psi`、`--pitch-source v|phi`。
切换后都会清平滑，并打印/显示当前源（面板 ③ 段）。

## 4. 键盘（窗口内）

| 键 | 作用 |
|---|---|
| `q` / `ESC` | 退出 |
| `空格` | 冻结画面（看某一帧细节） |
| `s` | 快照 → `results/w3_pose/snapshots/` |
| `r` | 开始/停止录像 |
| `k` | 清空平滑状态与手势投票 |
| **`c`** | **采零位**：把当前手姿记为 roll0/pitch0 与位置基准 (u0,v0) |
| `x` / `p` | roll / pitch 符号取反 |
| `y` | 横向（u→y）符号取反 |
| **`t`** | roll 源切换 theta ↔ psi |
| **`u`** | pitch 源切换 v ↔ phi |
| `g`/`G`、`h`/`H` | roll / pitch 增益 −0.1 / +0.1 |
| `d`/`D`、`f`/`F` | roll / pitch 死区 −1° / +1° |
| **`w`** | **保存 `calib.json`** |

## 5. 标定流程（需主人现场配合）

0. **自动标零（推荐，不用按键）**：把手放到舒服的位置 → **握拳两次**（每次都张开再握）→ 再**张开手掌保持 5 帧**，
   零位（含 `u/v`、`hand_px0`、`theta/cuff2d` 等）就自动采好了，屏幕上会闪 `握拳 1/2` / `好！张开手掌就标定` /
   `已自动标定零位（5 帧平均）`。两次握拳要在 **2.5 s** 内（`calib.auto_zero.window_s`）。
   手长小于 **0.08（≈51px@640）** 会被判成「手太小/误检」而拒收，此时把手凑近摄像头再试。
   2026-10-31 修过：原来窗口只有 1.2 s 且无任何提示，看起来就像功能失效。

1. 坐正，手掌正对摄像头、手指朝上，**保持不动** → 按 `c` 采零位（手动兜底，效果同第 0 步）。
2. **roll 符号**：手在画面内顺时针转（从自己视角看）→ 看屏幕右下 `roll` 条是否往"直觉方向"走；反了按 `x`。
3. **roll 增益**：手转 45°，腕部条走多少？目标 1:1（手转 45° → roll 45°）；偏离用 `g`/`G` 调。
4. **pitch 符号/增益**：手背朝自己（指尖往镜头压/抬）→ 看 `pitch` 条；反了按 `p`，幅度不够 `h`/`H`。
5. **死区**：手保持"自然姿势"时角度应在 ±(死区) 内为 0；抖得厉害就 `D`/`F` 加大（建议 3–5°）。
6. **横向符号**：手往右移，屏幕 `y` 是否变正（对应机械臂 y 增加）；反了按 `y`。
7. 满意后按 `w` 存盘 → 下次启动自动载入。

## 6. 已验证（2026-10-17，本机实测）

| 项 | 结果 |
|---|---|
| 21 点 + 手性 + world landmarks | 正常，contact sheet 见 `results/w3_pose/sheet_*.png` |
| 伸指数（margin_ratio=0.15） | palm→5: **34/36**；fist→0: **30/38**；like/ok/peace/no_gesture→"其他/保持"：112/140 |
| 帧率（CPU） | 纯检测 **16.7 ms/帧**；整套脚本无窗口 **28 fps**，带窗口 **18–20 fps** |
| 摄像头 | `--source 0` DSHOW 640×480 正常 |
| GUI 自检 | 8×8 dummy `namedWindow+imshow` 通过；`getWindowProperty` 检测 × 关窗 |
| 端到端走线 | 合成视频（palm→fist→palm→peace→fist，160 帧）跑通：夹爪 1→0→1、其他手势保持不动；`results/w3_pose/demo_walk.mp4` |
| yaw | 由位置反算 + 选靠近当前 yaw 的 π 分支（`right_joycon` 同策略），不由手给 |
| 双限幅 | 位置 0.02 m/s、角度 0.15 rad/s（`right_joycon` 同量级），越限标记 `slew_xyz` |

**θ/φ 的统计（HaGRID 正面手掌 74 张）**：φ 中位 −14.4°、std 12.7°（p5..p95 = −33.6..−0.1）；ψ std 15.0°；φ~θ 相关系数 **−0.46**。

→ 含义：**φ 有明显系统偏置（零位会吃掉）+ 约 ±13° 噪声，且与画面内旋转还有残余耦合**。所以 pitch 通道必须靠死区 + EMA + 软限幅（默认 `soft 0.85 rad`），标定时**先固定 θ 再动 φ**。

## 7. 未完成 / 待主人

1. **handedness 全局 swap 未验证**（必须主人自录一小段：已知左右手 + 已知张开/闭合）。
   - 现状：HaGRID 子集里左右手各占一半，**证不了是否整体反了**。
   - 验证动作：录 10 秒"右手举掌 → 右手握拳"，跑 `--source clip.mp4`，看窗口左上 `Left/Right` 是否与真实一致；反了用 `--hand-swap` 或改 `calib.json` 的 `handedness.swap`。
2. **真机标定（第 5 节）未做**——需要主人现场配合按 `c`/`x`/`p`/`w`。
3. 板子仍**未接**（B1 只到"打印/存文件"）。
4. `gripper` 目前只进 `POSE1` 第 7 字段；固件侧"仅预览、未真下发 ID6"这一点本轮没动。

## 8. 踩坑（本轮新增）

1. **`mediapipe 1.1.1` 的 `HandLandmarker.close()` 会卡 ~42 秒**（实测 VIDEO 模式 42.19 s），且解释器退出时 C++ 调度线程还会再卡一次。
   → 处理：`close_quietly()` 用 daemon 线程限时 1.5 s 等，脚本末尾 `os._exit()` 硬退。**不要**在 `finally` 里裸调 `close()`。
2. **FPS 统计要在 `finally` 之前取时间**，否则会把 close 的 40 s 算进平均帧率（本轮第一版就报了 3.3 fps，实际 28 fps）。
3. `mediapipe 1.1` **已删除 `mp.solutions`**（`drawing_utils` 等还在但不该依赖）→ 21 点连接表自带一份（`HAND_LINKS`）。
4. 中文叠字仍走 PIL（`C:/Windows/Fonts/msyh.ttc`），**底部提示分两行**，一行太长会被 640 宽的窗口横向截断。
5. 装 `mediapipe` 会带进 `opencv-contrib-python 5.0.0.93`；本环境没问题（`GUI: WIN32UI`），但**绝不能装进 `fpga_amd-pi`**（会顶掉那边刚修好的 opencv）。
