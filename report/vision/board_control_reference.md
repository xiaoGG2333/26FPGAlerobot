# 参考：板端控制链路速查（读 rk7100_so101_bound_retry_20261031 源码整理）

来源：`github.com/xiaoGG2333/26FPGAlerobot` → 分支 main → `rk7100_so101_bound_retry_20261031/`
（本机直连 github.com:443 被墙，改用 `api.github.com` / `raw.githubusercontent.com` 抓取）

固件 HELLO 标识（对不上就是版本不配套）：
`BOARD_RK7100_F_V1  CHIP_THERMAL1_V1  OPT125_SCURVE_20261018_V1  POSITION_ROLL_IK_20261019_V1
 SERVO_SPEED200_2000_20261031_V1  BOUND_RETRY_IK_20261031_V1  GRIPPER_LT_RT_20261030_V1`

## 1. 软硬件分工

| 位置 | 干什么 |
|---|---|
| **PL**（XC7Z100，双核 IP @ `0x40000000` / `0x40010000`，125 MHz） | 雅可比、正规方程、**单步线性求解**（`lm_solver_so101.hpp` / `xf_solver_lm_so101.cpp`）；`e5` 保留输入当 0..15 四轴约束位掩码；负阻尼 = 位置任务模式 |
| **PS**（ARM 裸机，`firmware/`） | IK 迭代循环、关节/编码器限位、S 曲线轨迹、舵机下发、温控、故障策略 |
| **舵机** ST3215 | PS UART1 经 EMIO，1 Mbps；ID1~ID5 臂，ID6 夹爪 |

## 2. 板端真正用到 POSE1 的哪几个数

`firmware/pose_uart.inc` 原文注释：

> `/* POSE1 XYZ and roll/ID5 are task targets; pitch/yaw are reserved. */`

- 用：`x y z`（任务位置，米）+ `roll`（**直接就是 ID5 wrist_roll 的关节角指令**，不是末端姿态）+ `gripper`
- **不用**：`pitch`、`yaw`（只做范围检查 `|pitch|≤1.4`、`|yaw|≤π`，随后丢弃）
- 校验：`|roll|≤π`、`gripper∈[0,1]`、`seq` 必须严格递增（否则 `ERR1 STALE_SEQUENCE`）、无笛卡尔工作空间限制（2026-10-19 移除）
- `roll` 的硬约束其实在 ID5 关节角范围：`q4 ∈ [-2.9437, +2.8486] rad`；超出回 `code=4`

**实测核对（本地 `sim/kin.py`）**：`fk(HOME_Q)` 的 roll = +0.1500 = `HOME_Q[4]`；`q4 ± 0.6` 时 roll 跟着 ±0.6，差值 0.0000 —— 即 `roll` 与 ID5 是 1:1。

## 3. IK 任务定义（`firmware/ik_iteration.c`）

`ik_solve_position_roll(xyz, roll, q_seed, max_steps=200, tolerance=1e-8, damping=0.01)`：

- 残差只有**位置 3 维**：`e[0..2] = xyz - T[3,7,11]`，`energy = 0.5·|e|²`
- `q[4] = roll` 直接置位，**不参与迭代**；`rotation_error = |q[4]-roll|`（报的是 ID5 误差，不是姿态误差）
- 只迭代 `q[0..3]` → 3 个约束 / 4 个关节 = 1 个冗余（肘部），这就是 `BOUND_RETRY` 用 ±10° 肘部初值重试的原因
- 收敛条件：`energy < 1e-8` → `|位置误差| ≤ 0.141 mm`（现场看到的 0.14 mm 就是这个容差底）
- 步长上限 `|Δq| ≤ 0.08 rad`，回溯线搜索最多 12 次折半；边界冻结掩码只约束"已到限位且要往外走"的关节
- `_retry` 版：原目标失败后最多再试 2 组数值初值（肘 ±10°），每次 ≤80 步，共用 600 次 PL 调用预算

## 4. 轨迹与速度（`firmware/arm_control.c` + `online_curve.h`）

| 参数 | 值 |
|---|---|
| 轨迹形式 | 五阶 Bezier（quintic），5 轴**共用一个 duration**，从 0.04 s 起按 ×1.08 搜索 |
| 关节速度上限 `vmax` | **2000 counts/s** = 3.068 rad/s = **176 °/s** |
| 加速度上限 `amax` | 1200 / 1200 / 1500 / 1800 / 1800 counts/s²（≈1.8~2.8 rad/s²） |
| 加加速度 `jmax` | 6000 / 6000 / 7500 / 9000 / 9000 counts/s³ |
| 单位换算 | `SO101_COUNTS_PER_RAD = 651.8986` |
| 舵机下发节拍 | `arm_service` 里 `elapsed < 20 ms` 直接返回 → **50 Hz** |
| 速度寄存器 | `ceil(|v|) + 50`，夹到 `[200, 2000]`（0 在 ST 总线上等于不限速，故不写 0） |
| 位置增量 | 每次下发的是曲线上的 waypoint，起停由 S 曲线保证 |

`ARM1 PROFILE <axis> <vmax> <amax> <jmax>` 可在 RAM 里改其中一轴（上限 2000/10000/100000），掉电失效。

## 5. 上位机侧的速度上限（不是板端）

`host/right_config.json`：

```json
"max_translation_m_s": 0.02,        // 手柄默认
"xbox_max_translation_m_s": 0.05,   // Xbox 允许的最大值
"max_rotation_rad_s": 0.15
```

`host/right_joycon.py` 的 `Mapper.target()` 与 `sw/pose_map.py` 的 `PoseMapper.map()` 是同一套结构：
`xyz = arm_start + scale·Δ` → **按 `max_translation_m_s·dt` 截断** → roll/pitch 同样限速 → `yaw = atan2(y, x-0.0612)`。
注意原版里 **yaw 是在截断之后算的**（`pose_map.py` 曾把它挪到截断之前，是那次改动引入的 bug）。

`--source` 可选：`replay / joycon / buttons / stick / xbox`。
`stick`/`xbox` 是**速度模式**（`StickPose.update` 把摇杆偏转按 `speed·dt` 积分成目标位姿），joycon/视觉是**绝对位姿 + 速度上限模式**。

## 6. POSE1 之外必须的握手（`ARM1`）

板端只有在 `arm.active && (mode==2||mode==3)` 时才接受 POSE1，否则回 `ERR1 HOLD_MODE_NO_POSE`。典型序列：

```
HELLO  →  ARM1 CHECK   →  ARM1 ENABLE（或 TEST_MANUAL_ENABLE）  →  ARM1 KEEP（保活）
       →  POSE1 <seq> <core> x y z roll pitch yaw gripper       →  RESULT1 ...
       →  ARM1 PAUSE / STOP / FREEZE / RESUME，SERVO1/MOTOR1/GRIP1 独立通道
```

`RESULT1 <seq> <code> <iterations> <pos_err> <rot_err> q0..q4 <gripper>`；
失败时另发 `LeRobot ERROR REJECTED KEEP_LAST_GOAL TORQUE_UNCHANGED`，且**保持上一有效目标**（不冻结力矩）。
片上 ≥75 ℃ 会拒绝新目标并冻结五轴（`CHIP_THERMAL_HOLD`）；舵机 ≥55 ℃ 连续 1.5 s 关扭矩，两套独立。

## 7. 本地工程的差异清单（待办）

| 项 | 板端 | 本地现状 | 影响 |
|---|---|---|---|
| IK 任务 | 位置 + ID5（4 约束 / 5 关节） | `sim/kin.py` 只有**全位姿** `solve`（6 约束 / 5 关节） | 全位姿要求 yaw 落在 `atan2(y,x-0.0612)` 流形上，否则必 MAX_ITER；**板端根本不受 yaw/pitch 影响** |
| POSE1 发送 | 需要 ARM1 握手 + 串口 | `sw/pose_map.py` 只往 stdout/jsonl 打印 | 视觉链路还没接到板子 |
| 速度上限 | 板端 vmax 2000 counts/s | `pose_map` 用 `calib.slew` 0.02 m/s | 与 `right_config.json` 默认一致；Xbox 上限 0.05 |
