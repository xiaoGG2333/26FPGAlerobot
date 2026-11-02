# 调研：手势映射控制机械臂的开源项目（2026-10-16）

调研目的：下一轮要做"手部关键点 → 手势/旋转/翻转 → 驱动 SO101"的方案，先看别人怎么做的、能抄什么、什么抄不了。

网络限制说明：本机 `github.com` 直连被墙，本次通过 web_search + `cdn.jsdelivr.net`（jsDelivr CDN 可通）读 README；star 数/完整许可未能逐个核实，已标注。

---

## 一、结论（先看这个）

1. **这类项目很多，且已有 3 个就是 SO-ARM100/SO-101 + 手部关键点**（见 A 档），说明"手→关键点→臂"的路线是主流、可行。
2. **但没有一个跑在 FPGA/Zynq 上**，也没有一个用自己的小网做 21 点热图回归。它们清一色是 **MediaPipe / WiLoR + Python + (ROS2)**，跑在 PC / 树莓派 / GPU 上。
   → 它们的**推理栈对我们完全不可用**（WiLoR 是 3D 手重建大网，MediaPipe HandLandmarker 也需要大算力，Zynq-7100 的 A9 跑不动）。
   → 但它们的**"语义映射层"（手势→动作、平滑、标定、安全）**非常有参考价值，这部分是纯逻辑、与算力无关，可以直接照抄设计。
3. 最有价值的一条发现（安全设计）：**`imitrob/teleop_gesture_toolbox` 用"握拳=使能"做死手开关**——握拳时机械臂才跟随你的手，"张开手 = 机械臂停止跟随"。这正好对应主人要求的"张开=夹爪开、握拳=夹爪合、**其他手势夹爪不动**"的安全语义，且实现只需几行状态机。

---

## 二、A 档：同一条臂（SO-ARM100 / SO-101）+ 手部关键点

| 项目 | 关键做法 | 许可 | 可抄什么 |
|---|---|---|---|
| **ABMI-software/hand_controlLerobot** | webcam + WiLoR/MediaPipe → SO-101；`hand_teleop/tracking/` 内做 **Kalman 平滑**；`kinematics/` 可选 pinocchio FK/IK；`gripper_pose/` 夹爪位姿；**`config/gesture_mapping.yaml` 手势映射**；关节/速度限制 + 急停；YOLOv8 自动抓取；三种模式：直接关节控制 / 任务空间笛卡尔 / 捏合手势控制夹爪 | **MIT** ✅ | **手势映射配置文件化**、Kalman 平滑、安全限幅与急停、模式切换设计 |
| **Joeclinton1/hand-teleop** | webcam + **WiLoR(GPU)** → **直接输出关节位置，不做 IK**；内置 Kalman 平滑；后台线程低延迟；`--hand left/right` 选左右手；Apache-2.0。README **专门警告**：LeRobot 依赖的 `opencv-python-headless` 会破坏 `cv2.imshow`（**和我们 10-15 晚踩的坑一字不差**） | Apache-2.0 ✅ | "学一个手→关节角的映射、跳过 IK"这条路值得评估；**`--hand left/right` 说明左右手必须显式区分**（与我们"要识别左右手"的结论一致）；它的 headless 警告印证我们的坑 |
| **sowmiah08/hand_based_manipulation** | SO-ARM100 + RealSense 深度 + MediaPipe **食指尖**像素坐标 → 用深度反投影成 3D → 末端跟随；ROS2 Jazzy + LeRobot | 未核实 | "食指指尖 + 深度"这条路线（我们已决定不用单目深度，可作为对比参照） |
| **nickmeinhold/lidar-robot-arm** | iPhone LiDAR 追手臂 3 点 + Vision 手部姿态 → WebSocket → LeRobot 驱动 SO-100/101 | 未核实 | 遥操作数据链路分层（采集/传输/执行）的工程化拆分 |

## 三、B 档：手势 → 机械臂（不同臂，但映射层通用）

| 项目 | 关键做法 | 备注 |
|---|---|---|
| **imitrob/teleop_gesture_toolbox**（研究组，CHI23/IROS 论文） | Leap Motion / RealSense + ROS2；**`links.yaml` 定义"手势含义→机器人动作"**；"手势句子"（点选物体 + 手势 = 组合指令）；**Direct Teleoperation：握拳=机械臂跟随手，张开手=停止跟随**；**右手遥操、左手开合夹爪**；把传感器与场景基座做一次性标定（`saved_setups/`） | 语义映射层最成熟的一个 |
| **EbyGunner/camera_robot_teleoperation** | 普通 webcam 控两台 6-DOF 臂：**左右手各驱动一条臂，握拳闭合对应夹爪**；ROS2 Jazzy + MoveIt2 | 左右手语义的做法 |
| **wzaielamri/ros2_dual_arm_teleop** | webcam 控双臂 UR5e + Robotiq 夹爪（ROS2 Humble） | 同上 |
| **Nabil-Miri/mediapipe_dual_arm_control** | MediaPipe 手姿 + **MoveIt Servo** 平滑伺服控双臂 Panda | MoveIt Servo 是一种"连续伺服"平滑思路 |
| **gjcliff/FrankaTeleop** | MediaPipe + MoveIt Servo 控 Franka 7-DOF（ROS2 Iron） | 同上 |
| **pablofntdz/xarm-gesture-control** | MediaPipe + Intel RealSense 深度，"无标记"控 UFactory xArm6 | 深度相机路线 |
| **daniellowell/handpose-robotics** | MediaPipe handpose → uHand 机械手；**3D 关节角检测**；USB 串口 / UART 二进制协议两条路 | 关节角→舵机的落地细节 |
| **Mattral/Gesture_Arm** | 手势 + 语音 + **LSTM 时域稳定** + 几何 IK；左手控臂 pan/tilt、右手控别的 | **LSTM 时域稳定**印证"必须做时域滤波" |
| **Chansnow/handRobicArm**（中文） | 摄像头识别手势**位置 + 开合** → 串口 → 树莓派控三轴臂；用中指指尖与拇指指尖的 x 距离变化判开合映射夹爪 | 与我们最像的"土办法"，坐标映射可参考 |
| **CarterWells111/OneTouchReality**（中文） | MediaPipe 关键点 → UDP → Unity 重建虚拟手 → 碰撞检测 → TCP → Arduino 控实体手 | 中间层设计 |
| **D-Robotics/gesture_control** | C++ 手势检测/追踪/识别模块（Horizon 生态） | 嵌入式 C++ 实现参考 |
| **HiminDong/gesture-vrep**（中文） | 自采数据训分类器 → 控 V-REP 的 PUMA560 仿真 | 自采数据的流程 |
| **agilexrobotics/Agilex-College**（松灵官方） | "手势遥操机械臂：使用手势遥操作 Piper 机械臂末端六自由度位姿" | 厂商官方案例 |
| **基于 FPGA(PYNQ) 的手势识别机械臂**（极术社区） | 单目图像 + PYNQ 做手势分割/建模/形状特征提取，识别 8 种手势 → 舵机臂分拣 + 井字游戏 | **唯一的 FPGA 同类项目**，但它是经典形状特征 + 分类，**不是关键点回归**，算力需求比我们小 |

## 四、可借鉴的论文

- *Vision-Based Hand Shadowing for Robotic Manipulation via Inverse Kinematics*（arXiv 2603.11383）：单目/自视角 RGB-D + MediaPipe 21 点 → 深度反投影成 3D → 变换到机器人坐标系 → **阻尼最小二乘 IK**。与我方"21 点 → 目标位姿 → IK"最接近。
- imitrob 组的 "Gesture Sentence"（CHI23 / IROS）系列：手势组合成"句子"再执行，是"手势→动作"语义层的研究级参考。

---

## 五、能直接抄的 6 条设计（与算力无关，纯逻辑）

1. **手势含义配置化**：不要把手势→动作写死在代码里，用 `links.yaml` / `gesture_mapping.yaml` 这类配置文件（toolbox / hand_controlLerobot）。我们的"张开/闭合/其他"映射也应外置成表。
2. **"握拳=使能"死手开关**（toolbox 的 servoing）：只有在检测到"闭合"时才让机械臂跟随，检测到"张开"或其他手势就停止跟随/保持不动。这条直接满足主人"其他手势夹爪不动"的安全要求，实现只要一个状态机 + 计数。
3. **Kalman / EMA / LSTM 时域平滑必须有**（hand-teleop、hand_controlLerobot、Gesture_Arm 三个项目都做了）。角度平滑要在 sin/cos 分量上做，避免 ±180° 回绕。
4. **夹爪用几何量而非分类**：拇指尖-食指尖（或中指）距离 / 指尖相对掌心的伸展度 → 夹爪开合（Chansnow 中文项目、hand_controlLerobot 的 pinch 模式都是这么做的）。这与我方"伸指数"几何规则一致，且比分类器更连续可控。
5. **左右手必须显式区分**：hand-teleop 有 `--hand left/right`，EbyGunner 用左右手各控一臂。不用它，翻转/旋转的符号就无法确定（与我们此前结论一致）。
6. **安全层独立于视觉层**：关节/速度限制 + 急停 + 工作空间夹紧（hand_controlLerobot、我方计划书里的"安全框 + 迟滞"）。视觉抖动绝不允许直接驱动舵机。

## 六、对我们方案的直接影响

- **不改变**：推理必须是我们自己训的小网（21 点热图 + soft-argmax + 后处理几何），因为目标板上跑不了 MediaPipe/WiLoR。
- ★**已作废（主人 2026-10-16 明确否决）**：hand-teleop 的“跳过 IK、直接学手→关节角”路线**不采用**——本项目已经把逆解算核做好了（`fpga/diag_fw/firmware/ik_iteration.c` + PL HLS 单步核），必须复用它，不能另起一套学习式“手→关节角”映射。此条不得再作为备选重新提出。
- **新增待定决策**：是否采用 **“闭合=使能、张开=停止跟随”** 的死手开关模式（来自 toolbox 的成熟做法），它比原计划的“夹爪开合 + 其他手势不动”更强的安全语义，但会改变操作手感（用户必须一直握拳才能让臂跟随）——**需主人拍板**。
- **许可提示**：hand_controlLerobot（MIT）、hand-teleop（Apache-2.0）、LeRobot（Apache-2.0）、MediaPipe（Apache-2.0）都可用于本项目；**WiLoR 的模型/权重许可需另行核查**（此类 3D 手重建模型常带研究/非商用限制），toolbox 许可未核实。

---

## 七、本项目逆解核的实际接口（2026-10-16 读码确认，设计必须对齐）

位置：`fpga/diag_fw/firmware/ik_iteration.{h,c}`（另有 `main.c` 调用示例、`README.md` 说明）

```c
void ik_fk(const float q[5], double T[16]);                     // 5-DOF 正解
void ik_error(const double cur[16], const double tgt[16], float e[6]);
int  ik_solve(const double target[16], const float initial[5],  // ← target = 完整 4×4 位姿
              int max_steps, double tol, float damping, int sugihara,
              ik_step_fn step, ik_trace_fn trace, void *ctx, ik_result *result);
typedef int (*ik_step_fn)(void *, const float[5], const float[6], float, float[5]); // 一步 Δq
```

事实：
- **输入是完整 4×4 目标位姿**（位置 + 旋转），不是纯 3-DOF 位置 IK；误差是 6 维 `position_error` + `rotation_error`。
- **step_fn 是回调 = 调 PL 里的 HLS 单步核**；ARM 负责跑整个迭代循环（封顶 1000 步，tol 1e-10，damping 0.01，`sugihara` 选 0/1）。
- **支持热启动**（`initial[5]`）。
- 自述限制：`mask = all ones`、**无 line search、无关节限位强制**（“No servo control, line search or joint-limit enforcement”）。
- PL 寄存器只有 `CTRL(0x0)/LAMBDA(0x90)/STATUS(0xE8)/STATE_I(0xF8)` → **看不到 mask 寄存器**（需确认 PL 核是否支持掩码）。
- 验收线：**板上单解 <5ms**（`fpga/diag_fw/README.md`，`measure_board_ms.py` + `board_us_median<5000`）。

由此定的视觉侧输出契约（写入《网络结构清单》v2.0 待办）：
```
关键点模型输出：21×2 坐标 + 左右手 + （热图）
后处理（PL/PS 均可）产出：
    末端目标位置 (x, y)   ← 掌心/腕部点位经标定映射，z 仍为固定深度平面
    手旋转角 θ_hand       ← 腕0→中指根9 向量（画面内旋转）
    翻转 flip             ← 掌面手性（左右手已由模型给出，符号无歧义）
    手势 gesture          ← 伸指数（5/3/2/0 → 张开/OK/剪刀/闭合）
映射层（新）构造 4×4 目标位姿 T → 调 ik_solve(热启动) → q[5] → SYNC_WRITE
```

**必须由主人拍板的 3 个接口问题**：
1. 目标位姿的**旋转分量怎么给**：只给工具轴 roll（θ_hand + flip），还是给完整 R？mask=all ones 时 5-DOF 追 6-DOF 会做最小二乘折中；若 PL 核能支持 mask，则应把不可达分量踢出误差。
2. **腕部 ID5 是否跟随**：原计划书定的是“ID5 固定 ≈ 2037 不跟随”；现在要手旋转/翻转，就必须重新定义 ID5 的角色。
3. **关节限位谁负责**：IK 核不强制限位 → 必须在映射层把目标夹进安全框（计划书已有“安全框 + 遲滞”，需写入契约）。
