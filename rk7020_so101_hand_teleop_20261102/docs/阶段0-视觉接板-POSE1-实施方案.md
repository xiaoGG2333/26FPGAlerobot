# 阶段 0：视觉 → POSE1 → 板端（零训练、零新 IP 核）

目标：**让手在画面里动，臂就跟着动**，同时把阶段 1 要用的数据采下来。
不改固件、不改 HLS、不训模型。只加一个串口发送模块。

来源依据：`docs/参考-板端控制链路速查-rk7100_20261031.md`（读 `rk7100_so101_bound_retry_20261031` 源码整理）。

---

## 0. 前置（两件硬件/环境，必须先解决）

| 项 | 现状 | 要做 |
|---|---|---|
| 板子调试串口接 PC | 本机 `SerialPort::GetPortNames()` 只有 COM3/4/5/6/9/10，全是"蓝牙链接上的标准串行"→ **板子没接在这台机上** | 用 USB-TTL 接板子 **UART0（MIO14 RX / MIO15 TX）**，115200 8N1；接好后会多出一个 USB 串口（原来的机器上是 COM16） |
| pyserial | `hand-pi` 里**没有** | `D:/Anaconda3/envs/hand-pi/python.exe -m pip install pyserial` |

舵机总线不用 PC 管：板子自己经 UART1/EMIO 驱动 ID1~6，PC 只连调试口。

---

## 1. 板端上电后的可用性判据

`main.c`：`servo_init() → motor_setup() → arm_setup() → cal_setup() → 自检 → pose_server()`
**自检不通过就不会进 `pose_server()`，串口不会有任何回包。** 所以"发了 HELLO 没反应"先查自检，不是查代码。

`HELLO` 必须回 `POSE1 READY `，且包含这几个版本标记（对不上说明 SD 卡里的 BOOT.BIN 不是这套）：

```
BOARD_RK7100_F_V1  CHIP_THERMAL1_V1  OPT125_SCURVE_20261018_V1
POSITION_ROLL_IK_20261019_V1  SERVO_SPEED200_2000_20261031_V1  BOUND_RETRY_IK_20261031_V1
```

---

## 2. 会话序列（从 `host/right_joycon.py` 抄的，已验证）

```
HELLO                          → "POSE1 READY ..."            只读，不动
ARM1 TEST_MANUAL_CHECK         → "ARM1 CHECK_OK ..."  [或 ARM1 END 包起来的 BLOCKED]
   （失败就先 ARM1 STATE 看诊断，不要继续）
[可选] ARM1 PROFILE <1..5> <v> <a> <j>  → "ARM1 PROFILE OK RAM_ONLY"   掉电失效
ARM1 TEST_MANUAL_ENABLE        → "ARM1 READY ..."            ← 力矩从这里才通
────────────────────────────── 正式循环 ──────────────────────────────
POSE1 <seq> <core> x y z roll pitch yaw gripper
                               → "RESULT1 <seq> <code> <iters> <pos> <rot> q0..q4 <grip>"
                               → 失败时另发 "LeRobot ERROR REJECTED KEEP_LAST_GOAL TORQUE_UNCHANGED"
                               → 协议错时 "ERR1 <seq> <REASON>"（STALE_SEQUENCE / RANGE / VALUE / HOLD_MODE_NO_POSE …）
────────────────────────────── 退出 ──────────────────────────────
ARM1 STOP                      → "ARM1 EVENT ... active=0 ... torque_off=1"
异常时：ARM1 FREEZE / ARM1 RESUME
```

**几个必须记住的固件行为：**

1. **看门狗 500 ms**（`RK_ARM_WATCHDOG_MS`，`arm_alignment.h`）。`arm_target()` 内部会调 `arm_keep()`，所以**持续发 POSE1 就等于喂狗**；
   但**一旦超过 500 ms 没发任何 ARM1/POSE1，板子自己 `arm_fault(ARM_WATCHDOG)` 冻结**。
   → 手丢了要发 `hold()` 的旧目标继续发，**绝不能停循环**。
2. `seq` 必须严格递增，重复/回退直接 `ERR1 STALE_SEQUENCE`。
3. `core` 只能 0 或 1（两个 PL 核）。
4. `TEST_MANUAL_ENABLE` → `mode=3, aligned=0, geometry=0, manual_session=1`：
   - 跳过对齐/几何检查（`ENABLE` 需要 `aligned=1 && geometry=1`，现场没做过对齐就会 BLOCKED）
   - **力矩被限到 `min(原值, 500)`**（`RK_TRIAL_TORQUE`），第一次试最安全
   - 手动会话**没有跳变检查**（`RK_ARM_MAX_TARGET_JUMP` 只对非手动生效）→ **速度上限只能靠上位机自己的 cap**
5. `pose_uart.inc` 的笛卡尔工作空间限制已移除（2026-10-19），越界与否只由 IK 和关节限位决定 → 上位机必须自己夹。
6. **`active=0` 时 POSE1 不会被拒**：源码里只有 `if(arm.active)` 那一层才校验 `mode`（→ `ERR1 HOLD_MODE_NO_POSE`）、才调 `arm_target()`；
   IK 解算在任何情况下都照跑。所以**未使能时 POSE1 是一台「纯 IK 预言机」——算并回 `RESULT1`，绝不碰舵机**。
   `pose_q` 只在 `code==0` 时被更新，所以连 "预热启动" 的行为与真机一致；`RESET1 <seq>` 可把它复位到 home。
   （早前文档里"未使能时 POSE1 回 ERR1 HOLD_MODE_NO_POSE"的说法是旧固件的，2026-10-31 实机验证已推翻。）

---

## 3. 串口带宽（决定了帧率上限）

115200 8N1 ≈ 11 520 B/s。一帧 `POSE1` 约 60 B，`RESULT1` 约 130 B，合计 ≈190 B
→ **30 Hz 要 5.7 KB/s（约 50% 占用）**，还要留 `STATE1`（约 200 B）/`THERMAL1` 的位置。

结论：
- **POSE1 用 20 Hz**（≈3.8 KB/s，安全），确认稳定后再试 30 Hz
- `ARM1 STATE` **每 10 帧读一次**，不是每帧
- 板端 `send_text` 是忙等发字节，回包太多会抢 IK 循环的 CPU

---

## 4. 要写的代码（两级，职责分开）

### 4.1 新文件 `sw/pose_link.py` —— 只管协议，不懂视觉

```
class BoardLink:
    open(port, baud=115200, timeout=0.3)
    hello()  -> dict(ok, markers, raw)          # 校验版本标记
    arm(cmd, expect=None) -> str                # 单条 ARM1 命令，收到 "...\r\nARM1 END" 为止
    check() / enable(mode='manual') / keep() / state() / stop() / freeze() / resume()
    send_pose(seq, core, pose:dict, gripper)    # 非阻塞写一行 POSE1
    poll() -> list[dict]                        # 非阻塞取回包：RESULT1 / ERR1 / LeRobot / THERMAL1
    close()
```

实现要点：**写用主线程、读用守护线程**（否则 130 B 的回包在 115200 上要 ~11 ms，会把 20 Hz 的循环拖死）；
解析按 `\r\n` 分行，`RESULT1` 拆成 `dict(seq, code, iters, pos, rot, q=[...], gripper)`。

### 4.2 给 `sw/pose_map.py` 加 4 个开关（约 60 行）

```
--send COM16            # 打开串口；不加就等于现在的纯打印
--hz 20                 # 发送节流（默认 20）
--enable-motion         # 不加则只 HELLO+CHECK，不发 ENABLE、不发 POSE1（默认最安全）
--envelope x0,x1,y0,y1,z0,z1,rollmax   # 首次试跑的矩形包络
```

循环里插一句：

```
pose, flags = mapper.map(g, dt)        # 现有逻辑完全不动
pose = clamp_envelope(pose)            # 仅在上位机夹
link.send_pose(seq, 0, pose, pose['gripper'])
for r in link.poll(): 记日志
```

**视觉侧一行都不改**：`pose_map` 的 5 通道映射、EMA、死区、`hold()` 全部照旧；
`pitch/yaw` 照旧填（板端会丢弃，只做范围检查）。

---

## 5. 验收（四步，每步单独确认后再下一步）

| 步 | 做什么 | 判据 |
|---|---|---|
| **S1 只读** | `HELLO` + `ARM1 STATE` + `STATE1 1` | HELLO 标记对得上；`ARM1 STATE` 里 `active=0`；**臂一动不动** |
| **S2 预言机验证** | 不发 ENABLE，直接把视觉输出当 POSE1 发（见 §5.1） | 收到 `RESULT1` 而非 `ERR1`；`code=0` 占绝大多数；**臂全程 `active=0` 不动** |
| **S3 小范围动** | CHECK → TEST_MANUAL_ENABLE → 手在画面中央小幅移动 | `RESULT1` code 全 0、`iters` 个位数、`pos_err < 0.5 mm`；手停→臂停；手丢→臂保持（看门狗不响） |
| **S4 采数据** | `--record` 录一段 60~90 s 带全通道动作的视频 + jsonl | 视频与 jsonl 按 `ts_ms` 对得上；这份素材就是阶段 1 的蒸馏数据 |

**首次动作的包络（建议值，可改）**：
`x 0.20~0.32，y −0.08~+0.08，z ≥ 0.20，|roll| ≤ 0.6 rad，cap 0.02 m/s，20 Hz`
理由：包在 HOME 附近、离桌面有余量、roll 只给 ID5 的一小段，`z ≥ 0.20` 防压桌。

**中止条件**（任一触发立刻 `ARM1 STOP`）：
`RESULT1 code != 0 连续 5 帧` / 出现 `ARM_FROZEN` / `ERR1` 连续 10 条 / 臂的实际动作与画面不一致。

---

## 5.1 S2 实测结果（2026-10-31 已跑，真板 COM18）

做法：拿 `results/w3_pose/pose_run_20261031_180612.jsonl`（修复**前**录的、带旧 yaw 的真实视觉输出）
沿整段均匀抽 60 帧，`RESET1` 复位后逐帧当 POSE1 发给板子，只收 `RESULT1`。**全程 `ARM1 STATE active=0`，臂未动。**

| 指标 | 结果 |
|---|---|
| `code=0` | **58 / 60** |
| 迭代 | 中位 6 步、最大 11 步（`max_steps=200`、`tol=1e-8`、damping 0.01） |
| 位置残差 | 中位 0.062 mm、最大 0.1397 mm（容差本身对应 `|p| < 0.1414 mm`） |
| 姿态残差 | 恒为 0（`roll` 直接赋给 ID5，`rot_err=|q4-roll|`） |
| `ERR1` / 无回包 | 0 / 0 |
| 失败帧 | 2 帧 `code=1`，残差 5.05 / 6.16 mm，距 pan 轴 **0.3503 / 0.3698 m**（可达球半径 0.3738 m，只剩 23.5 / 4.0 mm 余量） |

结论：

1. **视觉输出可以直接喂板端 IK**，协议、`seq`、数值范围全部对得上，不需要中间转换。
2. 那 2 帧失败都落在**外壳**（距球心 ≥ 0.35 m）。收敛帧的最远距离是 0.3658 m。
   → 上游 `clamp_reach(margin=0.030)`（把目标夹到 ≤0.3438 m）正好把这类点排除，"可达球内缩 30 mm"这条不是保守而是必要。
3. 旧 yaw 完全无害 —— 与源码一致（板端丢弃 pitch/yaw）。
4. 明细：`results/sim/board_oracle.csv`；往返 48.7 ms/帧是脚本固定等待（40 ms + 15 ms 计时粒度）造成的，**不是板子耗时**。

---

## 5.2 S3 实测结果（2026-11-01，真机，TEST_MANUAL_ENABLE）

| 指标 | 结果 |
|---|---|
| 时长 | 1500 帧 / 65.1 s = **23.0 fps**（POSE1 按 20 Hz 节流后发 1262 帧） |
| 回包 | **1261 / 1262**，`code=0` **100%**，`ERR1` **0** |
| IK | 迭代 平均 4.2 / 最大 7；位置残差 中位 0.081 mm / 最大 0.1414 mm |
| 手部检出 | 1184 / 1262 = **93.8%**（丢手 78 帧全程 hold 保持） |
| 输出范围 | x 0.159→0.270、y 0.040→0.080、z 0.200→0.355、roll −0.02→1.27 |
| 退出 | `ARM1 STOP STOPPED active=0 torque_off=1` |

### 四个踩坑（都是实测踩出来的，代码已改）

1. **使能必须在视觉栈就绪之后。** 先 `TEST_MANUAL_ENABLE` 再去加载 landmarker / 开摄像头要几秒，而看门狗只有 **500 ms**
   → 板子自己 `ARM_FROZEN`。正确顺序：摄像头+landmarker 就绪 → `CHECK` → `ENABLE` → 立即 `KEEP` → `STATE1` → 再 `KEEP` → 进循环。
2. **异常必须兜底 `ARM1 STOP`。** 实测踩到一次：使能之后某段检查抛 Python 异常，异常在 `try` 之外，
   臂被留在使能态。现在 `pose_link._LIVE` 记录当前链路，`__main__` 的 `except BaseException` 里必定 STOP。
3. **序号要接板端的 `pose_last_sequence`。** 板端跨复位保留它，新会话从 1 开始会连发 `ERR1 STALE_SEQUENCE`
   （实测因此被自我保护中止过两次）。启动时先 `STATE1` 取最后一个 seq，再往后接。
4. **从实测位姿过渡，不要从 HOME 硬跳。** 使能后 `STATE1` 拿到的就是实测末端（实测这次离 HOME 差 64°、
   x 0.159 / z 0.355）；用 `--blend 6` 秒线性过渡，否则第一帧就把臂甩到包络里。

### 待调（不是 bug，是参数）

包络命中 717 帧（57%）、上游 `reach` 夹紧 653 帧（52%）、y 顶到 +0.08 有 254 帧、z 顶到 0.20 长期贴着
→ 零位（AutoZero 期间触发了两次）、`pos.gain_*`、包络三者需要一起调。另外 `gripper` 已经在 0/0.5/1 之间变化，
但按设计**没有驱动 ID6**。

---

## 6. 阶段 0 明确不做

- 不提速（`cap` 保持 0.02，等 S3 通过再按 0.05 试）
- 不碰 `pitch → ID4`（当前固件丢弃 pitch，要恢复必须改固件）
- 不接夹爪 ID6 的实际下发（`gripper` 字段照发，板端只做预览）
- 不改 `BOOT.BIN`、不改 HLS、不训模型
