# B1 阶段总结（2026-10-17 起 → 2026-10-31 定稿）

## 一、目标
在 PC 上把「摄像头 → 手部 21 点 → 机械臂 POSE1 指令行」这条走线打通并标定，为后续
自训 21 点关键点模型（板端）提供**已验证的映射层与标定参数**。本轮不训练模型、不接板子。

## 二、交付物

| 文件 | 行数 | 作用 |
|---|---|---|
| `sw/pose_map.py` | 1176 | 主程序：摄像头/视频 → HandLandmarker → 几何 → POSE1 行 + 中文 HUD + 标定 |
| `sw/pose_sheet.py` | 163 | 离线批处理：对 HaGRID 图片跑检测、拼标注 contact sheet、出统计 json |
| `sw/axes_check.py` | 104 | 退化分析复现脚本（翻腕 vs 画面内转的分辨能力） |
| `sw/run_calib.bat` | 12 | 一键起标定 GUI（纯 ASCII，cmd 里不能有中文 echo） |
| `docs/研究-手部自由度测量逻辑-v2.md` | 93 | **最终控制逻辑定案**（观测量 ↔ 自由度、自动标定） |
| `docs/研究-翻腕与转腕的可分性.md` | — | 退化问题的几何证明与数值表 |
| `docs/B1-视觉走线-使用与标定.md` | 180 | 使用方法、键盘表、标定流程、坑 |
| `results/w3_pose/calib.json` | — | 现场标定参数（零位、符号、增益、死区、自动标定配置） |

## 三、最终控制逻辑（定案）

| 手部观测 | 机械臂自由度 | 换算 |
|---|---|---|
| 掌心横向 u | y 左右 | `y = y0 + 0.6·(u−u0)` |
| 掌心纵向 v | z 上下 | `z = z0 − 0.6·(v−v0)` |
| 手的大小 `hand_px` | x 前后 | `x = x0 − 0.5·(手长比−1)`，手长比 = `hand_px0/hand_px` |
| 画面内腕转角 θ | **roll**（唯一旋转自由度） | 死区 4° → 符号 → 增益 1.0 → **夹 ±90° 饱和** |
| 伸指数 5 / 0 | 夹爪 | 5→1.0、0→0.0、其它保持 |
| 位置反算 | yaw | `atan2(y, x−0.0612)`，手不给 |
| — | pitch ID4 | **固定 hold**（−0.0977 rad） |

**合计：3 平移 + 1 旋转 + 夹爪**（5 轴臂 + yaw 反算的必然结果）。

保护链：0.8% 死区 → 0.02 m/s 平移限速 / 0.15 rad/s 旋转限速 → 工作空间 ∩ 固件硬限
（x∈[0.12,0.45], y∈[−0.30,0.30], z∈[0.03,0.45], |roll|≤π, |pitch|≤1.4, gripper∈[0,1]）。
丢手 → `hold()` 冻结上一帧有效解，绝不下发部分解。

## 四、POSE1 协议（与主机 right_joycon 完全一致）

```
POSE1 <seq> <core> <x> <y> <z> <roll> <pitch> <yaw> <gripper>\n    数值 '%.9g'
```
回传 `RESULT1` 12 字段：CODES = CONVERGED / MAX_ITER / HARDWARE_ERROR / NUMERICAL_ERROR / JOINT_LIMIT。

## 五、走线验证结果

- MediaPipe VIDEO 模式 `detect_for_video` 16.7 ms/帧（≈60fps 上限）；端到端 28 fps（无窗口）、18–20 fps（带 HUD）
- HaGRID 验证：palm 伸指数 5（36/36）、fist 0（30/38）、no_gesture/其他 → 夹爪保持（行为正确）
- 摄像头：按**设备名**选（pygrabber + DirectShow），避开内置 ASUS，优先外接 GWI-1200
- 手势自动标定：握拳 2 次 → 张开手掌 → 5 帧平均采零位（演示前重设相对基准）

## 六、踩过的坑（已修）

1. `landmarker.close()` 在 mediapipe 1.1.1 固定卡 42 s → 限时关闭 + `os._exit()` 硬退
2. `wrap()` 会给 roll 制造 ±180 跳变 → 改成偏移量夹 ±90° 饱和
3. 握拳时掌宽/手长比涨到 1.0 → 宽度比融合要门控（伸指数 ≥4 才用）
4. 正掌姿势下「翻腕」与「画面内转腕」退化（同一根轴）→ 主因，已改由 θ 单通道驱动 roll
5. `hand_px0` 是前后映射的基准，缺失时自动用第一帧手长补；换姿势务必重采
6. calib.json 的字段含义变过（sign_v 等）→ 迁移时显式重写，不能靠 deep_merge

## 七、下一步：简易 3D 仿真（建议入口）

**目标**：不接板子，先把 POSE1 → IK → 关节角 → 可视化 这条链跑通，验证可达性与手感。

可直接复用的现成件：
- `fpga/diag_fw/firmware/ik_iteration.c` —— 与 PL 里 `xf_solver_lm_so101_0/1` 同一套 IK，一行不改
- `refs/.../host/right_joycon.py` —— `fk_pose()`（LeRobot ET 模型，tx/tz 已提取）、`encode()/parse_result()`
- HOME_Q = `[0.2,−0.3,0.4,−0.2,0.15]` → FK = 位置 `[0.2726,0.0429,0.2383]`、rpy `[0.15,−0.10,0.20]`
- 关节限位不在 `ik_solve` 内，在 `so101_q_to_counts()`（越界 code=4）与 `arm_target`（ARM_Q_LIMIT）
- `sw/pose_map.py --source <视频/摄像头>` 可当 POSE1 数据源，`--no-jsonl --quiet` 只要指令行

**建议仿真形态**：Python + OpenGL/pyrender 或 matplotlib 3D，读 POSE1 行 → IK → 画 5 连杆 + 夹爪，
叠加工作空间包络与关节限位告警；先做静态位姿回放（录像 jsonl 里有完整轨迹），再做实时。
