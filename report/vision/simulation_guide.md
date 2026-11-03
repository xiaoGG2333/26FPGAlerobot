# 简易 3D 仿真使用说明（`sim/live_sim.py`）

> 一条链：**手 → POSE1 → IK → 屏幕上的线框臂**。不接板子、不下发舵机、不驱动夹爪。
> 目的是演示与自查：握拳看夹爪合、手腕转看爪头自转、手前后看臂伸缩。

## 1. 它是什么 / 不是什么

| 是 | 不是 |
|---|---|
| `sw/pose_map.py` 那条已跑通的手→POSE1 链路 | 新的映射逻辑（复用同一份 `Geometry` / `PoseMapper` / `calib.json`） |
| 自写的线框 3D 视图（球面轨道相机 + 针孔投影） | 物理引擎（没有碰撞、重力、动力学） |
| `sim/kin.py` 的 FK/IK（与固件对齐） | 板端 IK 的复用（HLS IP 源码不在库内，这里是等价 DLS） |
| 演示 / 自查 | 真机控制（不做串口、不发 POSE1、不碰 `fpga/`） |

画面布局：**左 = 摄像头 + 手骨架 + ASCII 关键值**（按 `h` 可切成 `pose_map.draw` 的完整中文 HUD），
**右 = 3D 线框臂 + 仿真状态 HUD**（目标位姿、实际解、残差、迭代数、夹爪）。

## 1.1 性能（实测 2026-10-31，内置 ASUS 摄像头 640x480）

总帧时间约 **33 ms → 30 fps**（验收线），分解（日志每 60 帧打印一次）：

| 环节 | 耗时 | 说明 |
|---|---|---|
| 读帧 | 8 ms | DSHOW 缓冲后基本不阻塞 |
| landmarker | 16 ms | mediapipe 1.1.1 单帧推理，基本固定 |
| 左面板 | 2 ms | 快速模式（骨架 + putText）；`h` 切完整 HUD 会变 22 ms |
| IK | 1–2 ms | 热启动 + 停滞退出；不可达目标不会跑满 100 步 |
| 3D 渲染 | 2.7 ms | 三层渲染（静态层缓存 + 动态层 + ASCII 文字） |
| imshow/waitKey | 2.3 ms | 1280x480 合成图 |

> 优化前后对比：3D 渲染 40 ms → 2.7 ms。原来慢在 PIL 中文（18 ms/帧）、
> ghost 透明层（10 ms）、静态几何每帧重画（6.7 ms）、掩码贴图（8.4 ms）。
> 现在中文标签只画一次进静态层，动态数值一律 `cv2.putText`，动态层用 `np.maximum` 合成。

## 2. 环境

只用 `hand-pi`（Python 3.10 / mediapipe 1.1.1 / numpy 2.2.6 / opencv-contrib 5.0.0）：

```
D:/Anaconda3/envs/hand-pi/python.exe -u sim/live_sim.py ...
```

- 不要往 `fpga_amd-pi` 装 mediapipe（会顶掉它修好的 opencv）。
- 离线回放（`--replay`）不 import mediapipe，只依赖 numpy/opencv。
- 所有输出落在 `results/sim/`（快照 `snapshots/`、录像 `rec_sim_*.mp4`、会话摘要 `sim_session_*.json`）。

## 3. 两种模式

### 3.1 离线回放（先走这个，不开摄像头）

```bash
# 建议用较短的一段，别拿 30MB 的整段长跑
D:/Anaconda3/envs/hand-pi/python.exe -u sim/live_sim.py \
  --replay results/w3_pose/pose_run_20261031_180612.jsonl
```

- 按记录的 `ts_ms` 节流，`--replay-speed 2` 加倍速；`--replay-loop` 循环。
- 无窗口自检（只验链路，不开 GUI）：

```bash
D:/Anaconda3/envs/hand-pi/python.exe -u sim/live_sim.py \
  --replay results/w3_pose/pose_run_20261031_180612.jsonl \
  --no-window --max-frames 200 --replay-speed 1000 \
  --dump-frame results/sim/snapshots/replay_check.png
```

### 3.2 实时（摄像头 + landmarker）

```bash
D:/Anaconda3/envs/hand-pi/python.exe -u sim/live_sim.py --source auto
```

- `--source auto` 按设备名避开内置摄像头（序号会随插拔整体平移，不要按序号选）。
- 后台跑 GUI 用 PowerShell `Start-Process`，日志重定向到 `results/w3_pose/xxx.log`；
  cmd 的 `start` 会吞引号/反斜杠。
- 退出时 mediapipe `close()` 会卡约 40 s（1.1.1 已知问题），脚本用 `close_quietly()` + `os._exit()` 硬退。

## 4. 按键

窗口有焦点走 `cv2.waitKey`，控制台有焦点走 `msvcrt`（`ConsoleKeys` 双通道）——两路都能用。

| 键 | 作用 |
|---|---|
| `q` / `ESC` | 退出 |
| `s` | 快照到 `results/sim/snapshots/sim_<时间>.png` |
| `r` | 开始/停止录像（合成画面） |
| `空格` | 冻结/解冻输入（臂停在当前目标，摄像头继续） |
| `v` | 切换视角：斜视 → 侧视 → 正视 → 俯视 |
| `-` / `=` | 相机远近（拉远 / 推近），看全工作空间包络时用 `-` |
| `h` | 左面板在「快速 ASCII」和「完整中文 HUD」之间切换（后者慢约 20 ms/帧） |
| `z` | 重采「手长基准」`hand_px0`（拿手在舒服的位置按，前后伸缩才准） |
| `w` | 把当前 `calib.json`（含自动标定的零位）存盘 |
| `k` | 清空平滑与末端轨迹 |

## 5. 画面上每个元素是什么

| 元素 | 含义 |
|---|---|
| 橙色折线 + 白色球 | 5 根连杆（`kin.fk_parts()` 的 origins），折线端点即关节轴位置 |
| 关节处两条短线 | 该关节的旋转轴方向（joint0 绕 z、joint4 绕 x、其余绕 y，与固件一致） |
| 浅蓝夹爪 | 末端系画出来的示意夹爪，开度随 `gripper` 变 |
| 紫红色暗框 + 夹爪 | **目标位姿 ghost**（`kin.make_target()` 直接画的理想末端；颜色比实线暗一档） |
| 青白渐变折线 | 末端轨迹拖尾（最近 160 个点，分 4 段批量画） |
| 末端到地面的虚线 | 末端在地面上的投影，帮助判断深度 |
| 暗蓝虚线长方体 | 工作空间包络 `WORKSPACE_MIN/MAX`（比真实可达域大，见 5.1） |
| 状态 / 迭代 / 位置残差mm / 姿态残差deg | 与固件同号的 IK 结果（0 收敛 / 1 MAX_ITER / 3 数值错误） |

**状态三种颜色（日志里写作 `OK / ACCEPT / HOLD`）：**

| HUD | 日志 | 含义 | 臂的表现 |
|---|---|---|---|
| `code 0 CONVERGED`（绿） | `OK` | 固件同义收敛 | 正常跟随 |
| `code 1 ACCEPT`（橙） | `ACCEPT` | 没跑到 1e-8，但位置≤3mm、姿态≤0.29° | **仍然跟随**（用最后一步迭代值） |
| `code N HOLD`（红 + 横幅） | `HOLD` | 残差超容差 | 冻结在上一帧解 |

## 5.1 上游 `pose_map.py` 已改的三处（2026-10-31）

仿真暴露的三个问题都在 `sw/pose_map.py` 里，**已按真机口径改掉**（真机同样受益）：

| # | 位置 | 原来的问题 | 现在 |
|---|---|---|---|
| 1 | `PoseMapper.map` | `yaw` 在位置 **slew 限幅之前**算；限速一介入，指令 yaw 就领先实际位置（实测 23/125 行偏 >3°，最大 47°），目标离开 5 轴可达流形 `yaw=atan2(y,x-0.0612)` → 固件只会回 MAX_ITER、臂卡住 | yaw 挪到 xyz 限速**之后**由位置反算，仍取靠近上一帧的分支。验证：60 帧连续大机动，偏离流形 **0 帧**（最大 2e-16 rad） |
| 2 | `PoseMapper.map` | 工作空间盒（x 0.12..0.45、z 0.03..0.45）比真实可达域大很多；5 轴臂从 shoulder_pan 轴算最大臂展只有 **0.3738 m**，z 拉到 0.4 就够不着 | 新增 `calib['reach'] = {enable, margin_m}`，目标径向夹进 `0.3738 - margin` 的球（默认内缩 30mm）。径向缩放不改方位角，yaw 约束仍成立；越界打 `reach` 标记。验证：120 帧大机动 IK **120/120 收敛**，最大残差 0.141mm |
| 3 | `AutoZero`（握拳两次自动标零） | ①`window_s` 只有 1.2s，两次握拳常来不及，过期就**默默清零**；②除成功外没有任何提示，看不出「没识别到」还是「没到位」；③单帧 `n_ext` 噪声就算一次握拳；④手太小/误检也会采零位 | `window_s` 2.5s、连续 `hold_frames=2` 帧才算一次握拳、每次握拳都闪 `握拳 1/2`、手丢了 `gap_reset_s` 后自动取消、手长 < `min_hand_px`(0.08≈51px) 一律拒收并提示「手太小，凑近摄像头」 |

`live_sim.py` 里的 `fix_yaw()` / `clamp_reach()` 保留为**幂等保险**（上游修好后不再有任何动作，
已加断言验证），万一以后回退到旧 calib 或旧代码也不会卡死。

## 5.2 手长比（前后 x）的基准

前后伸缩靠 `zero.hand_px0`（标定时腕→中指根的归一长度）做归一。它有两个坑：

- **没采过**（现场 `calib.json` 里是 0.0）→ x 永远不动。现在会在第一帧「像样的手」时补采。
- **采错**：内置摄像头会锁到 1～8px 的假目标，拿它当基准 `size_ratio` 就爆了（x 被顶到边界）。
  所以统一加了门限 `pose_map.MIN_HAND_PX = 0.08`（≈51px@640）：低于它不采基准、也不参与自动标定。
- 想重采：把手放到舒服的位置按 **`z`**；握拳两次 + 张开手掌则会走完整自动标定（含 `hand_px0`）。
- 即使基准对，手长测量本身受视角/模糊影响（同一只手实测 92～102px 抖），
  所以 x 相对起始点的偏移再夹 **±8 cm**（`--x-range`）。

## 6. 常用参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--replay PATH` | 无 | 离线回放 jsonl（给了就不起摄像头） |
| `--replay-speed` | 1.0 | 回放倍速 |
| `--source` | `auto` | 摄像头序号或 `auto` |
| `--panel-w/--panel-h` | 640/480 | 3D 面板尺寸（合成图 = 左面板 + 它） |
| `--fov` / `--dist` | 38 / 0.55 | 3D 相机视场角(°) / 到注视点距离(m) |
| `--view` | 斜视 | 初始视角（斜视/侧视/正视/俯视） |
| `--full-hud` | 关 | 一开始就用 `pose_map.draw` 的完整中文 HUD（慢） |
| `--x-range` | 0.08 | 前后(x)相对起始点的最大偏移 m，挡手长比噪声 |
| `auto_zero.min_hand_px` | 0.08 | （calib 里）手长低于此值不采基准、不参与自动标定 |
| `--max-steps` | 100 | IK 最大迭代 |
| `--stall` | 8 | 能量连续多少步不再下降就判不可达、提前退出（0=关，跑满 100 步） |
| `--reach-margin` | 0.030 | 可达球内缩量 m |
| `--no-clamp-reach` | 关 | 不夹目标（复现固件卡住的行为） |
| `--accept-pos` / `--accept-rot` | 0.003 / 0.005 | 未完全收敛时的接受阈值（m / rad） |
| `--max-frames` | 0 | 跑够多少帧就停（自检用） |
| `--no-window` | 关 | 不开 GUI（自检用） |
| `--dump-frame PATH` | 无 | 把最后一帧合成图存成 png |

## 7. 排错

| 现象 | 处理 |
|---|---|
| `[错误] 当前 opencv 没有 GUI 后端` | 装的是 headless 版：卸掉 `opencv-python-headless`，装 `opencv-python` |
| 按键没反应 | 焦点在终端也能用（双通道已兜底）；若仍无反应看启动日志里「控制台读键 可用/不可用」 |
| 臂不动但摄像头里有手 | 先看右面板底部有没有「目标超出可达球…已夹到边界」；有 `--no-clamp-reach` 时超可达目标会冻结 |
| 日志/面板「px 只有几 px」或提示「手太小」 | 误检或手太远：把手凑近摄像头（< 0.08≈51px 的值一切不采信）；前后伸缩不准按 `z` 重采基准 |
| 握拳两次没反应 | 看左面板左下角的提示：出现「握拳 1/2」说明第一次已计上，两次要在 2.5s 内；出现「手太小」说明手离得太远 |
| 前后伸缩总贴在边界 | 手长基准不对：按 `z` 重采；或者 `--x-range` 改小 |
| 目标 ghost 一直偏离实线 | 看 `code`，`MAX_ITER` 说明目标不可达（先查 `yaw` 是否由位置反算、roll 是否超出 ±90° 饱和） |
| 画面卡顿 | 先看 HUD 的 `IK xx ms`；渲染线数固定，掉帧一般是 landmarker（17 ms 左右） |

## 8. 与已有文档的关系

| 文档 | 关系 |
|---|---|
| `docs/交接-简易3D仿真开工.md` | 本轮任务的开工说明与边界 |
| `docs/交接-B1-视觉走线开工.md` | 手→POSE1 链路的定义（角度定义、标定项） |
| `docs/接口-视觉侧接入原摇杆控制链.md` | POSE1 字段与固件硬范围 |
| `sim/README.md` | 运动学底座的对齐关系与自检结果 |
