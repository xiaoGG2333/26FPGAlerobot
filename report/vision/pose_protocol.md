# 接口：视觉侧接入原摇杆控制链（2026-10-16 读码确认）

> ⚠️ **2026-10-31 更新：本文 §3 与 §7 已过时。** 板端固件从 `POSITION_ROLL_IK_20261019_V1` 起，
> POSE1 的处理由**全位姿 `ik_solve`** 换成了 **`ik_solve_position_roll`（位置 + ID5 关节角）**，
> 源码注释写明 *"POSE1 XYZ and roll/ID5 are task targets; pitch/yaw are reserved"*。
> 因此：
> - §3「yaw 必须按位置反算否则跑满迭代」**只对我方 sim 成立**，板端完全不读 yaw/pitch，
>   实测同一可达位置 yaw 偏 0.5 rad 时全位姿 IK 残差 107 mm，而位置+ID5 仍 0.139 mm 收敛。
> - §7「φ_hand → pitch → ID4 wrist_flex」**当前固件不支持**：pitch 被丢弃，ID4 由 IK 自己在
>   4 关节解 3 位置方程时的那 1 维冗余里选（带边界感知、以当前编码器角为初值）。
> - 要恢复 pitch→ID4，得改固件（给位置任务再加一个姿态项），属固件改动，需另行讨论。
>
> 最新事实见 `docs/参考-板端控制链路速查-rk7100_20261031.md`。

目的：把"手部关键点 / 手势 / 旋转 / 翻转"接到原项目的控制链上，**IK 核与固件协议不改**。

来源：`fpga/diag_fw/firmware/pose_uart.inc`（= `refs/26FPGAlerobot/rk7020_so101_xbox_temp1500_speed100_600_20261011/firmware/` 的原固件副本，已应用打点）+ `refs/.../host/right_joycon.py` + `refs/.../mapping_review/REVIEW.md`。

---

## 1. 原链路（事实）

```
主机 right_joycon.py                    固件 pose_uart.inc                      PL
 ─────────────────                      ─────────────────                       ─────────────
 摇杆 → (xyz, roll, pitch)  ──POSE1──►  校验范围 → 构造 4×4 目标 T  ──ik_solve──► LM 单步核
 yaw = atan2(y, x-0.0612)                warm start = 上一帧 q                   (xf_solver_lm_so101_0/1)
 位置/旋转双限幅(×dt)                    tol 1e-8, max 200 步, damping .01
 + gripper 字段(0..1)                    → q[5] → so101_q_to_counts → SYNC_WRITE ID1..5
                                         gripper 字段 → 1975..3567 counts（仅预览，未下发 ID6）
```

`POSE1 <seq> <core> x y z roll pitch yaw gripper`（10 个 token，7 个数值）。

## 2. 目标位姿构造（固件侧，`pose_uart.inc:104-108`）

```c
/* Intrinsic xyz / Rz(yaw) Ry(pitch) Rx(roll), SI units, row-major. */
double target[16] = {cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, v[0],
                     sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, v[1],
                     -sp,   cp*sr,        cp*cr,        v[2],
                     0,0,0,1};
int code = ik_solve(target, pose_q, 200, 1e-8, .01f, 1, arm_step, NULL, &selected, &result);
```

★ 关键：**`Rx(roll)` 是最内层 = 绕工具轴** → 对应 FK 里 j=4（绕 X、零位置耦合）→ **ID5 wrist_roll**。
`Ry(pitch)` → j=3（绕 Y，位置耦合）→ **ID4 wrist_flex**。`Rz(yaw)` → j=0 → **ID1 shoulder_pan**。

## 3. 原设计已经处理好的两件事（此前我担心的问题，对原设计不成立）

| 我此前提的隐忧 | 原设计的实际处理 |
|---|---|
| `ik_solve` 6 维能量无 mask，5 轴追 6 维位姿永远收敛不了 → 每帧跑满迭代 | **主机侧把 yaw 按位置反算**（`yaw=atan2(y, x-0.0612)`，并按靠近当前 yaw 选 π 分支）→ 目标位姿**天生落在 5 轴可达流形上** → 真能收敛（固件用 tol 1e-8 / max 200 步，本来就是按"能收敛"设计的） |
| 目标一步跳到 π 会撞 `angle_axis` 近 π 退化分支 | **主机侧对位置和旋转都做了每帧限幅**（`max_translation_m_s*dt` / `max_rotation_rad_s*dt`）→ 目标从当前位姿小步爬行，不会一步跳 π |

补充：关节限位虽然不在 `ik_solve` 内部，但在 `so101_q_to_counts()`（越界 → code=4）和 `arm_target()`（返回 `ARM_Q_LIMIT`）里查了。

**保留的结论**：以上只在"复用 yaw 反算 + 限幅"这条规则时成立。如果视觉层直接构造一个**任意完整朝向**塞进 POSE1（例如把手旋转矩阵拆成 rpy 后原样下发、yaw 由手给），就会退回成"不可达位姿 → 折中解 + 跑满迭代"。**视觉层必须沿用 yaw 反算。**

## 4. 固件的硬性范围校验（`pose_uart.inc:93-94`，越界直接 ERR1 拒绝）

| 字段 | 允许范围 |
|---|---|
| x | -0.5 .. 0.6 |
| y | \|y\| <= 0.6 |
| z | -0.3 .. 0.7 |
| roll | \|roll\| <= π |
| **pitch** | **\|pitch\| <= 1.4 rad（≈80°）** |
| yaw | \|yaw\| <= π |
| gripper | 0 .. 1 |

→ 视觉/映射层必须先夹到这些范围，否则固件报错拒绝（不是 IK 报错）。

## 5. 爪子通道：协议里已经有了

```python
# host/right_joycon.py
def encode(seq, core, pose, gripper):
    values = list(pose) + [gripper]      # 第 7 个字段
# firmware/pose_uart.inc
static float pose_gripper = .5f;
if(code==0){ memcpy(pose_q,result.q,sizeof pose_q); pose_gripper=v[6]; }
int gripper = (int)lroundf(1975 + pose_gripper*(3567-1975));   # → ID6 counts
```
`REVIEW.md`：**"夹爪ID6目前仅预览，不应与五轴逆解混为一谈"**。

→ 所以"加入爪子的动作" = **往这个已存在、已校验、已转 counts 的字段填真值 + 让它真正下发 ID6**，不是协议/架构改动。

## 6. 视觉侧要填的字段（映射结论）

| POSE1 字段 | 视觉来源 | 对应关节 | 位置耦合 |
|---|---|---|---|
| x, y（z 固定） | 掌心/腕部关键点经标定映射 | ID1..ID3 | — |
| **roll** | **手的画面内旋转角 θ（腕0→中指根9）**；翻转若为绕前臂轴则叠加在此 | **ID5 wrist_roll** | **零耦合（实测 TCP 位移恒为 0）** |
| pitch | **腕部俯仰 φ（手背/掌心朝向）** | ID4 wrist_flex | 与位置抢冗余自由度，且 ±1.4 rad 硬限 |
| yaw | **由位置反算，绝不由手给** | ID1 | — |
| gripper | 手势（伸指数 5/0；其他手势保持不动） | ID6 | 与 IK 无关 |

## 7. 手的姿态：可用 2 个自由度，正好对应 ID4 + ID5

主人 2026-10-16 确认：腕部**俯仰** + 绕**前臂轴**，机械臂上这两个自由度都有。

### 7.1 自由度计数（为什么恰好是两个角）

| 约束 | 用哪些关节 | 结论 |
|---|---|---|
| 位置 (x,y,z) | q0..q3（4 轴） | 4 轴解 3 个位置方程 → **1 维冗余** |
| 俯仰 pitch | 就是那 1 维冗余（肘部构型） | **可独立命令，但可用范围随位置变**；接近完全伸展时冗余退化 → 范围变窄甚至解不出 |
| 绕前臂轴 roll | q4 单独一个 | **与位置零耦合**（实测 TCP 位移恒 0） |
| yaw | 由位置反算（q0 跟随） | **手的偏航不会被复现**（夹爪始终朝"从基座往外"） |

所以：(位置 3) + (roll 1) + (pitch 1) = 5 个数 ↔ 5 轴，**满秩、可解**；`POSE1` 里 roll / pitch 本来就是两个独立字段，无需改协议。

### 7.2 两条通道的不对称（工程上必须处理）

| | roll → ID5 | pitch → ID4 |
|---|---|---|
| 对 TCP 位置 | 零影响 | **与位置抢那 1 维冗余** |
| 固件硬限 | \|roll\| <= π | **\|pitch\| <= 1.4 rad（约 80°）** |
| 失效模式 | 基本不会解不出 | 位置近工作空间边界时可能解不出（code=1 / ARM_Q_LIMIT） |
| 保护措施 | 无需特别处理 | ① 视觉侧软限幅（建议先限 ±0.6~0.9 rad）；② IK 未收敛（code!=0）时**回退上一帧有效解**（保位置、冻结 pitch），绝不下发部分解 |

### 7.3 视觉 → POSE1 最终接线

```
θ_hand（画面内旋转：腕0→中指9）    → roll   → ID5 wrist_roll
φ_hand（腕部俯仰：手背/掌心朝向）  → pitch  → ID4 wrist_flex
手势（伸指数 5/0/其他）            → gripper 字段(0..1) → ID6（需从"仅预览"改为真下发）
(x, y)（z 固定平面）               → x, y, z
—（绝不由手给）                     → yaw，保持原 atan2(y, x-0.0612) 反算
```

## 8. 待主人明确（剩余三项，都是标定参数）

1. **零位**：哪个手姿对应 roll=0 / pitch=0（需一次采集标定）。
2. **符号**：手往哪边转 = 正（注意 ID5 符号 −1、ID4 符号 +1，见 `mapping_review/REVIEW.md`）。
3. **增益**：手转 1° → 腕转几度（1:1 还是缩放），以及死区大小（建议 3-5°）。

已确定、不再讨论：腕部两个自由度都用（ID4+ID5）；yaw 不由视觉给；爪子走 gripper 字段而非逆解；IK 核与 POSE1 协议不改。
