# 证据分层与验证边界

## 实机证据按版本登记（2026-11-02）

以下数据来自操作者在协作对话中贴出的串口日志。保留能力标记和采样值，便于在原记录中定位。

| 平台与功能 | 实机观察 | 能支持的结论 |
|---|---|---|
| RK7020 双逆解 IP | CORE0、CORE1 均通过参考输入、零误差和重复输入，串口返回 OVERALL PASS: 6 cases + repeatability + cross-core agreement. | 两个实际 IP 在该板端测试集上的输出符合预期 |
| RK7100 热控版本 | HELLO 含 BOARD_RK7100_F_V1 和 CHIP_THERMAL1_V1，读到 chip_c=31.97、fan_pct=50、rpm=2130，ID1 至 ID6 均返回 OK | 已在 RK7100 实板运行固件，读回片温、风扇转速和六轴舵机反馈 |
| RK7100 Xbox 保持与停止 | seq=106 时显示 XBOX_HOLD、max_goal_gap=3 counts，Ctrl+C 后收到 ARM1 STOP STOPPED active=0 torque_off=1 | 现场联调记录覆盖保持、位置反馈和显式停止 |
| RK7100 限位重试版本与 2025.2 迁移候选 | 对应离线报告记录 board_execution=NOT_RUN | 这两份报告只描述各自构建的验证范围。归档该版本实测时，应另存带版本标记的 HELLO、测试输入、期望值与实测输出 |

RK7020 双核测试和 RK7100 热控、执行反馈应分别引用。若需要声称 RK7100 某版的双核数值一致性或求解加速比，应提供该板该版的对应测量。手势识别尚未进入实机验证，列为后续扩展。

2026-11-01从实际文件读取的快照，哈希与摘录在evidence_index.json。源文件更新后需重新评估，不能悄悄更新哈希后继续沿用旧结论。

| 范围 | 文件证据 | 可支持的结论 |
|---|---|---|
| 2024.2限位重试 | reports/bound_retry_validation.json | 离线与构建PASS；板端NOT_RUN |
| 数值对照 | reports/bound_retry_native.json | 1878例；原成功1485例保留，新增成功7例来自合成集 |
| 2025.2候选 | repro_2025_2/reports/migration_validation.json | 构建与离线通过；formal_baseline_switched=false；板端NOT_RUN |
| 候选构建过程 | repro_2025_2/reports/build_status.json | 各阶段退出码和时间；不能代替实物验收 |
| 发布记录 | reports/github_publication_bound_retry_20261031.json | 本地记录发布仓库与提交、remote_verified=true；本次未联网复核远端 |
| 映射 | firmware/servo_mapping.h | 本机零点/符号/范围；物理alignment未认证 |

数值分组：历史969例保持890成功、79失败；日志近似初值659例保持357成功、302失败；合成250例由238成功变为245成功，仍5例失败。近似初值不等于精确复现原始硬件求解。

2024.2：125MHz，WNS +0.416ns、WHS +0.056ns。2025.2：125MHz，WNS +0.382ns、WHS +0.024ns；LUT32718/277400(11.79%)、FF38263/554800(6.90%)、BRAM4/755(0.53%)、DSP190/2020(9.41%)。资源与时序结果不能直接推导完整IK加速倍数。

尚未完成：上述候选最新BOOT的板端/实物验收、统一条件下ARM软件与ARM+PL基准、本Skill的独立实物复用评估。手势识别、视觉抓取和运行时智能体均仅后续扩展，不写入已验证能力。

验证边界分五层：文件一致性→编译/数值离线→RTL→板端计算→实物执行。check_evidence.py只负责第一层与报告摘录，输出MATCH不是功能PASS。
