# 交接：简易 3D 仿真开工（2026-10-31）

> 新对话第一个动作：读本文件 → 读 `sim/README.md` → 读 `sim/kin.py`。
> 然后按第 6 节「待主人拍板」问一句，再按第 7 节动手。

## 1. 任务定义

**目标**：把 `sw/pose_map.py` 已经跑通的「手 → POSE1」链路，接上一个 3D 视图，让机械臂
在屏幕上跟着手动起来，用于**演示与自查**（不接板子、不下发舵机）。

**明确不做**（本轮边界）：
- 不接 Zynq 板、不真下发 POSE1/舵机、不驱动夹爪 ID6
- 不改 `fpga/diag_fw/firmware/` 任何一行（IK 核是权威，仿真只对齐不复用）
- 不训练模型（W3 关键点模型是另一条主线，与本任务无关）

## 2. 已经做完的底座：`sim/`

| 文件 | 行数 | 内容 |
|---|---|---|
| `sim/kin.py` | 248 | FK / 雅可比 / DLS IK / 关节表 / counts 转换 / 固件限幅常量 |
| `sim/test_kin.py` | 84 | 自检（FK 对齐、固件场景复现、在线序列、可达性、counts） |
| `sim/README.md` | 60 | 对齐关系、结论、API 用法 |

### 对齐关系（谁是权威）

| 实现 | 权威来源 | 方式 |
|---|---|---|
| `kin.fk()` `kin.fk_parts()` | `fpga/diag_fw/firmware/ik_iteration.c:ik_fk` | 逐字移植，ET 模型同 tx/tz |
| `kin.ik_error()` | 同上 `ik_error` | 保留 `frame='target'` 分支可逐字对账 |
| `kin.solve()` | 板端 HLS IP `xf_solver_lm_so101_0/1` | IP 源码不在库内，给等价 DLS |
| `kin.JOINTS` `q_to_counts` `gripper_to_counts` | `fpga/diag_fw/firmware/servo_mapping.h` | zero/sign/限位/651.9 counts·rad⁻¹ 全同值 |

### 自检实测（已全部通过）

```
[1] FK(HOME_Q) pos=[0.27265 0.04286 0.23829] rpy=[0.15 -0.10 0.20]  偏差 4.7e-5 m / 6.9e-17 rad
[2] 固件 SCENARIO0/1 复现：16 / 37 步收敛，q 与固件打印一致
[3] 在线形态（2mm 小步 + 热启动）150 帧：1.23 ms/次，平均 2.8 步，150/150 收敛
[4] 姿态不变只平移 2mm → code=1 MAX_ITER（5 轴固有，不是 bug）
[5] HOME_Q → counts [1991 1960 2154 1856 1939]；夹爪 0/0.5/1 → 1975/2771/3567
```

### 关键常量（已写进 `kin.py`）

```
TX = (0.0612, 0.02943, 0.02798, 0.13504, 0.0593)
TZ = (0.0598, 0.05504, 0.11270, 0.00519, 0.00996)
关节轴：joint0 绕 z、joint4 绕 x、其余绕 y
HOME_Q = [0.2, -0.3, 0.4, -0.2, 0.15]   →  末端 [0.2726, 0.0429, 0.2383]，rpy [0.15, -0.10, 0.20]
SO101_COUNTS_PER_RAD = 651.8986469
夹爪 ID6：0.0..1.0 → counts 1975..3567
```

## 3. 上游链路：`sw/pose_map.py`（1176 行，已可跑）

### 数据流
```
摄像头 → HandLandmarker(VIDEO) → Geometry.compute → PoseMapper.map → POSE1 行 → 打印/jsonl
```

### 最终控制逻辑（主人 17:39 / 17:30 拍板，已定案）

| 手的量 | 映射到 | 说明 |
|---|---|---|
| 横向位置 u | y 左右 | `y = start_y + sign_u*gain_y*deadzone(u-u0)` |
| 纵向位置 v | z 上下 | `z = start_z - sign_v*gain_z_v*deadzone(v-v0)` |
| 手长比 `hp0/hpx` | x 前后 | 手更大=更近，`x = start_x + sign_size*gain_x_size*(-dsz)` |
| 手腕转角 θ | roll（爪头自转） | 死区 4°，增益 1.0，偏移夹 ±90° 饱和 |
| 伸指数（5/0/其他） | 夹爪 | 张开→1、握拳→0、其他→保持 |
| pitch | **固定 hold** | `-0.0977 rad`，不参与控制 |
| yaw | 位置反算 | `atan2(y, x-0.0612)`，绝不由手给 |

### POSE1 协议（固件接口，仿真应照此解析）
```
POSE1 <seq> <core> x y z roll pitch yaw gripper\n
```
- 七个数全部 `%.9g` 格式、必须有限
- 固件硬范围（越界固件回 ERR1 拒绝）
  ```
  x∈[-0.5,0.6] y∈[-0.6,0.6] z∈[-0.3,0.7]
  |roll|≤π     |pitch|≤1.4   gripper∈[0,1]
  ```
- 目标位姿构造顺序：`T = T(xyz) · Rz(yaw) · Ry(pitch) · Rx(roll)`
  （`kin.make_target()` 已实现）

### 现场标定文件 `results/w3_pose/calib.json`（主人已存盘，是真实零位）

```
zero: theta -129.62° / phi -49.19° / psi 13.86° / twist -41.56° / palm 13.95°
      point 27.63° / cuff 24.24° / cuff2d 9.18° / palm_r0 1.0611
      u 0.32166 / v 0.78799 / hand_px0 0.0（未采，第一帧有手时自动补）
roll: source theta, sign -1.0, gain 1.0, deadzone 4.0°, max_offset 90.0°
pitch: source hold, gain 0.6, deadzone 3.0°, soft_limit 0.85 rad
pos:  sign_u 1.0, sign_v 1.0, sign_size 1.0, gain_y 0.6, gain_z_v 0.6,
      gain_x_size 0.5, deadzone_uv 0.008, deadzone_size 0.02
slew: 0.02 m/s, 0.15 rad/s
auto_zero: enable, grasps 2, window 1.2s, open_wait 4.0s, average 5 帧, cooldown 2.5s
```

### 复用方式（推荐，不要解析 stdout）

`pose_map.py` 的 CLI 在 `main()` 里、`mediapipe` 也在 `main()` 里才 import，
所以同进程 import 是安全的：

```python
import sys; sys.path.insert(0, r'D:/C_O_D_E/2026/FPGA_AMD/sw')
import pose_map
calib = pose_map.load_calib('results/w3_pose/calib.json')
geo = pose_map.Geometry()
mapper = pose_map.PoseMapper(calib)
# 每帧：g = geo.compute(landmarks, world_landmarks, scale_hint, hand_sign)
#       pose, flags = mapper.map(g, dt)
#       line = pose_map.encode_pose1(seq, 1, pose)
```

注意 `PoseMapper.map()` 内部靠 `self.last_t` 推 `dt` 并做 EMA 与限速，
**必须每帧按真实时间间隔调用**，否则限速行为与现场不一致。

## 4. 必须记住的物理约束（本次踩到的最大坑）

**5 轴臂不能任意摆姿态。** 目标必须落在可达流形上：

```
yaw = atan2(y, x - 0.0612)     ← 由位置反算，不是自由量
```

违反它（例如「只平移 2mm、姿态完全不变」）会得到 `code=1 MAX_ITER`、残差卡在 0.4mm 左右。
这是固件本来就会有的行为，**不是仿真 bug**；原设计靠「yaw 反算 + 每帧位置/旋转限幅」
保证目标始终落在流形上。

仿真渲染时要接受这件事：遇到 `code=1` 就沿用上一帧的有效解（与主机侧同策略），
并在画面上标出 `MAX_ITER`。

## 5. 已踩过、别重踩的坑

1. **旋转误差必须用世界系**：`rotvec(R_t R_cᵀ)` + `J_rot = z_i`。
   写成目标系 `rotvec(R_tᵀ R_c)` 却配世界系雅可比，位置块与旋转块符号相反，
   迭代振荡不收敛（本次第一个 bug）。
2. **`joint4 绕 x` 不是绕 z**：固件 `ik_fk` 里 `i==0` 绕 z、`i==4` 绕 x、其余绕 y。
   雅可比里搞错就完全不收敛。
3. **mediapipe `landmarker.close()` 固定卡 42 秒**（1.1.1 已知问题）。
   现有代码用 daemon 线程 + `join(1.5)` 限时，再 `os._exit()` 硬退。
   仿真脚本若也开 landmarker，照抄 `close_quietly()` + 硬退。
4. **GUI 读键必须双通道**：`cv2.waitKey` 只在窗口有焦点时收键；后台启动时焦点在终端，
   已加 `ConsoleKeys`（msvcrt 轮询）兜底。仿真别只靠 `waitKey`。
5. **摄像头不能按序号选**：设备序号会随外接插拔整体平移（GWI-1200 与 ASUS 互换过）。
   现有逻辑按设备名判断（`pygrabber` + `_looks_builtin` + `pick_auto`）。
6. **改 `pose_map.py` 前必须先看原文**：它 1176 行、多次打补丁，凭记忆写替换串会
   `AssertionError`。用 `sed -n`/`grep -n` 看准再改；补丁脚本的写盘在末尾，
   assert 失败＝全没写（此时 `py_compile` 通过不代表改动生效）。
7. **项目不是 git 仓库**，改文件无法回滚。大改前先 `cp 文件 文件.bak`。
8. 文档/代码里不要用 `✓`（PIL 微软雅黑无此字形，显示成方块）。

## 6. 待主人拍板（开工前问这一句）

3D 视图怎么呈现：

| 选项 | 做法 | 优点 | 代价 |
|---|---|---|---|
| **A（推荐）** | 同一个 OpenCV 窗口左右分屏：左摄像头+HUD，右自绘 3D 线框臂 | 零新依赖、出结果最快、同屏对照 | 线框，无光照阴影 |
| B | 独立 3D 窗口（PySide6 / pyrender / matplotlib 3D） | 实体渲染好看 | 要往 hand-pi 装 qt/pyrender，可能拖依赖 |
| C | PyBullet | 有碰撞/重力，最像真实 | 最重，要自建 URDF |

推荐 A：先把「手 → 臂动起来」这条链看到，画质以后再说。

## 7. 实施清单（拍板后按序做）

1. 新建 `sim/live_sim.py`：
   - 摄像头 → landmarker（复用 `pose_map` 的相机/读键/画字工具）
   - `Geometry` + `PoseMapper` 复用，拿 `pose` 七元组
   - `kin.make_target()` → `kin.solve()`（热启动上一帧解、`max_steps=100`、`tol=1e-8`、`λ=0.01`）
   - 3D 线框渲染：5 根连杆 + 基座 + 夹爪（用 `kin.fk_parts()` 的 origins/axes）
   - 画工作空间包络（`WORKSPACE_MIN/MAX`）、桌面网格、辅助坐标系
2. 增加可视化要素：
   - 末端轨迹拖尾（最近 N 帧）
   - 目标位姿 ghost（半透明）+ 实际解（实线），能一眼看出 `MAX_ITER` 时的偏差
   - 画面上打 `code` / `iterations` / `pos_err` / `rot_err`
3. 按键：`q` 退出、`s` 快照、`r` 录像、空格冻结、`v` 切换视角（正/侧/斜）
4. 离线回放模式 `--replay results/w3_pose/pose_run_*.jsonl`：
   不开摄像头、不依赖实时，先验证渲染与 IK 链路
5. 写 `docs/仿真-使用说明.md`，更新 `sim/README.md` 状态
6. 清理 `_scratch/`

### 验收标准

- `--replay` 能完整回放一段已录 jsonl，臂平滑跟随，无跳变
- 实时模式 30 fps 以上（IK 实测 1.2ms/次，瓶颈在渲染与 landmarker 17ms）
- 画面能看出「握拳→夹爪闭合」「手腕转→爪头自转」「手前后→臂伸缩」
- `MAX_ITER` 时画面明确标注且沿用上一帧解

## 8. 运行环境（不要污染）

- 用 `D:\Anaconda3\envs\hand-pi\python.exe`（Python 3.10，mediapipe 1.1.1，numpy 2.2.6，
  opencv-contrib 5.0.0 且 GUI=WIN32UI 正常）
- **绝不能往 `fpga_amd-pi` 里装 mediapipe**（会顶掉它刚修好的 opencv）
- 后台跑 GUI 用 PowerShell `Start-Process`，日志重定向到 `results/w3_pose/xxx.log`；
  cmd `start` 的引号/反斜杠会吞路径
- 脚本加 `-u`；Git Bash 路径用 `/d/C_O_D_E/2026/FPGA_AMD`

## 9. 执行权限（硬规矩）

- **不许自行启动**摄像头长跑、GUI、录屏、压测；要给命令、先问一句再跑
- 只读操作（读文件/看日志/看进程）不限
- 所有文件留项目内，临时件放 `_scratch/` 用完即删

## 10. 文件索引

| 路径 | 用途 |
|---|---|
| `sim/kin.py` | 运动学底座（本轮成果） |
| `sim/test_kin.py` | 自检 |
| `sim/README.md` | 底座说明 |
| `sw/pose_map.py` | 手→POSE1 主链（几何/映射/HUD/标定） |
| `sw/pose_sheet.py` | 离线图片批处理自检 |
| `sw/axes_check.py` | 旋转退化数值分析 |
| `results/w3_pose/calib.json` | 现场标定（真实零位） |
| `results/w3_pose/pose_run_*.jsonl` | 回放素材（每帧完整几何+POSE1） |
| `fpga/diag_fw/firmware/ik_iteration.c` | IK 权威实现 |
| `fpga/diag_fw/firmware/servo_mapping.h` | 关节表/counts 权威 |
| `docs/接口-视觉侧接入原摇杆控制链.md` | POSE1 字段与固件硬范围 |
| `docs/研究-手部自由度测量逻辑-v2.md` | 观测量推导与最终定案 |
| `docs/B1-阶段总结与下一步.md` | B1 全阶段总结 |
