# 调研：Vitis AI DPU 上板路线可行性（桌面调研，2026-11-01）

> 目的：闭环《阶段1-视觉模型训练-计划书》§11-③ 的待拍板项——"上板路线先按 Vitis AI DPU 走？"
> 性质：纯桌面调研（官方文档 + 上游仓库 + 社区数据源），未动板子、未改任何现有文件。
> 结论会推翻计划书 §7.2 的"首选 DPU"排序，**是否改计划书由主人拍板**。

---

## 0. 结论摘要（先看这个）

| 计划书原假设 | 调研结果 |
|---|---|
| "Vitis AI DPU 官方支持 Zynq 7000" | **半对。官方支持已于 2021-07 终止**（PG338 v3.3 修订记录原文："对 Zynq-7000 器件的支持已终止。请联系您当地的销售代表"）。最后官方组合 = **Vitis AI 1.4/1.4.1 + Vivado/Vitis/PetaLinux 2021.1 + DPU IP 3.3**。板端工程现为 2024.2，差 3 年 |
| "PS 侧调用简单" | **对已验证的裸机固件不成立。DPU 官方只支持 Linux**（PetaLinux/Yocto/Ubuntu + 内核驱动 + VART）。官方 FAQ 原文："As of this release, **no support exists for bare-metal or FreeRTOS**"；且 DPU 指令编译器闭源、指令集不公开 → 自研裸机 runtime 属逆向工程，不现实 |
| "DPU + IK 核资源共存需 M3 实测，是唯一可能推翻方案的风险" | **资源反而是唯一没有风险的项**（见 §3）：B1152 折算到 XC7Z100 约 16% LUT / 11% DSP，IK 双核约 10%/10%，共存绰绰有余 |
| "MobileNetV3-Small 为目标结构" | **三条候选路线全部容不下 hard-swish/SE**（见 §5.2）：VAI 1.4.1 的 DPU IP 3.3 无 HardSwish ALU（VAI 2.5/IP 4.0 才加入，而 2.5 又不支持 7000）；FINN/Brevitas 与 hls4ml 同样偏好 ReLU 族 → **模型结构应改为纯 ReLU 化设计，这个决定现在就能做，且三路线通用** |

**一句话**：DPU 在 Zynq-7100 上"能用，但只能以 2021.1 工具链 + 板上跑 Linux"的形态用。真正的决策点不是"驱动验证"，而是两个更上游的问题：
① **接受板上跑 Linux 吗？**（接受 → DPU 路线存活；坚持裸机 → DPU 基本出局）
② 接受工具链冻结在 5 年前的 Vitis AI 1.4.1 吗？

若两问任一答"否"，主路线应转为 **FINN（或 hls4ml）定制小网**——与现有"裸机固件 + Vivado 2024.2"完全兼容，且我们的模型（≤1.5M 参数 / ≤300M MACs）正落在 FINN 的甜区。

---

## 1. 问题一：版本支持矩阵（证据链）

时间线（全部有来源，见 §7）：

| 时间 | 事件 | 含义 |
|---|---|---|
| 2019-12 | Vitis AI 1.0 FAQ："Vitis AI will support Zynq-7000 and ZU+ embedded platforms such as ZCU102, ZCU104, Ultra96, **Zedboard**" | Zynq-7000 曾是官方一等公民 |
| 2021-07-22 | **PG338 v3.3 修订记录："对 Zynq-7000 器件的支持已终止。请联系您当地的销售代表"** | 官方终止点。IP 3.3 ↔ Vitis AI v1.3/v1.4 ↔ Vivado 2020.2/2021.1 |
| 2022-06 | Vitis AI 2.0 起，DPUCZDX8G（IP 3.4/4.0/4.1）目标只剩 ZU+ MPSoC / Kria K26（官方兼容性表） | 7000 从 TRD/预编译镜像消失 |
| 2023-06 | Vitis AI 3.5（最后一个经典 DPU 版本）：FAQ"支持的器件家族"= ZU+ MPSoC、Versal、Kria、Alveo，**无 Zynq-7000**；兼容 Vivado/PetaLinux **2023.1** | 3.5 也不救 7000 |
| 2024~2026 | AMD Vitis AI 官网："The latest release of Vitis AI provides General Access support for two device series: Versal AI Edge / Gen 2"，DPU 归入 "legacy releases" | DPU 整条线已停止演进 |
| 现状 | DPU-PYNQ 仓库已归档（archived），master（VAI 2.5/PYNQ 3.0）板卡列表清一色 ZU+，无 Zynq-7000 板 | 官方连旁路示例也放弃了 7000 |

**矛盾点如实记录**：AMD 的 DPU IP 产品页（2025/2026 仍在更新）依然写"可集成到部分 Zynq 7000 SoC"，架构列表还是 v2 时代的 B512/B1024/B2304/B3136——该页面是历史遗留，与 PG338 v4.1（标题已改为"适用于 Zynq UltraScale+ MPSoC"）和 Vitis AI 3.5 FAQ 冲突，**不能当作现行支持的证据**。

**可用组合的唯一官方答案**：

```
Vitis AI 1.4 / 1.4.1  +  DPU IP 3.3  +  Vivado/Vitis/PetaLinux 2021.1  +  Linux kernel 5.10
```

附带的工程现实（待核实项，见 §6）：
- PetaLinux 2021.1 官方宿主是 Ubuntu 18.04/20.04，现有构建机是 22.04 → 大概率要开 VM
- Vitis AI 1.4.1 以 docker 镜像分发（xilinx/vitis-ai:1.4.x）→ 国内网络下 Docker Hub 可达性要验证
- 我们板子是定制 RK-ZYNQ7100-F，无厂家 PetaLinux BSP（W1 时期就在等厂家 BSP，从未落实）→ device-tree/ rootfs 要自己做，好在 `hardware/rk7100_ps.tcl` 有完整 PS 配置可导出 XSA 给 PetaLinux

## 2. 问题二：OS 形态（本次调研最重要的发现）

官方 FAQ（Vitis AI 3.5，原文）：

> "Is it possible to deploy the DPUCZ without using Linux? — We do have proof-of-concept beta support for both Green Hills Integrity and Blackberry QNX... **As of this release, no support exists for bare-metal or FreeRTOS**; however, **Zynq UltraScale+** family members do support asymmetric multiprocessing..."
>
> "the Vitis AI tool DPU instruction compiler is **not provided as open source**, and the **instruction set for the DPUCZ is not publicly documented**."

三条硬含义：

1. **DPU 必须跑 Linux**（PetaLinux/Yocto/Ubuntu + UIO 内核驱动 + VART 用户态运行时）。
2. 官方 AMP 逃生门（Linux 管 DPU、裸机管应用）**只给 ZU+**（A53+R5 架构）。Zynq-7000 没有 R5；A9-A9 双核拆分（CPU0 Linux + CPU1 裸机）在 UG585/OpenAMP 层面技术可行，但 **Vitis AI 对这种组合零官方支持**，属自研集成。
3. "自己写裸机 DPU 驱动"被官方明确堵死：指令集不公开、编译器闭源。VART 的核心调度逻辑虽在 GitHub（src/runtime_src），但它深度绑定 Linux（mmap/UIO/dlopen/pthread），移植裸机 = 重造运行时，且 xmodel 指令 blob 的行为只能黑盒试。

**而我们现在板上跑的、刚通过 S3 真机验收的，是裸机固件**（忙等 pose_server、500ms 看门狗、50Hz 舵机节拍、UART1/EMIO 直驱 ST3215）。于是 DPU 路线必然带来一次"控制链路形态"的重构，三种可选形态：

| 形态 | 做法 | 对已验证资产的破坏 | 难度/风险 |
|---|---|---|---|
| **F1 全 Linux 迁移** | 控制固件移植为 PetaLinux 用户态程序（C 源码全在手：`arm_control.c`/`ik_iteration.c`/`st3215.c` 等）；50Hz 舵机环 + 500ms 看门狗在 Linux 上用 SCHED_FIFO 线程可达成；忙等 UART 改 termios/UIO | **大**：整个已验收控制链要在 Linux 下重新回归（时序、看门狗、热保护、ERR1 语义） | 中。工作量大但每一步都有成熟做法；这其实就是 W1 最初的 PetaLinux 路线图（`docs/w1_d1_petalinux_checklist.md` 本来就是为此写的） |
| **F2 A9-A9 AMP** | CPU0 = Linux（DPU+摄像头+视觉）；CPU1 = 现有裸机控制固件几乎不动；OpenAMP/rpmsg 或共享内存+IPI 传 POSE1 | **小**：控制固件保真 | 高。内存/中断分区、核间协议全自研，无官方组合支持，排障孤立无援 |
| **F3 放弃 DPU** | FINN/hls4ml 定制加速器直接进 2024.2 工程，裸机固件不动，推理程序也是裸机（寄存器接口简单、自己生成的 RTL 自己全懂） | **零** | 中。工具流要自建，但无版本锁、无 OS 依赖 |

## 3. 问题三：资源预估（好消息，数字很扎实）

XC7Z100（-2）：277.4K LUT / 2020 DSP / 755×36Kb BRAM（26.5Mb）/ 无 URAM（URAM 是 UltraScale 专属，DPU 的 URAM 选项在 7000 上不可用，只能走 BRAM 配置）。

| 组件 | LUT | DSP | BRAM | 来源 |
|---|---|---|---|---|
| IK 双核（xf_solver_lm_so101 ×2 @100MHz） | ~27K（≈10%） | 198（≈10%） | ~8×36Kb（≈1%） | 7020 实测报告折算：DSP 198/220、LUT 50.55%×53.2K、BRAM 5.71%×140（《移植及修改进度》§5 初始资源报告） |
| DPU B1152 单核（144MHz，参考值） | ~44.6K（≈16%） | ~220（≈11%） | ~110×36Kb（≈15%） | 论文实测@XC7Z020：83.9% LUT / 100% DSP / 78.9% BRAM（CSDN 转引，量级与 PG338 一致） |
| DPU 最低配（B512 级） | ≥38.5K | 更少 | 更少 | 社区实测："DPU IP 最低配置至少需要 38457 LUT"（7010 装不下、7020 可以） |
| 风扇/温控/XADC/UART EMIO 等现有逻辑 | 少量 | ~0 | 少量 | 现有工程 |
| **合计（B1152 + 双 IK）** | **~72K / 277K ≈ 26%** | **~420 / 2020 ≈ 21%** | **~120 / 755 ≈ 16%** | 推算 |

结论：**资源共存无风险**，甚至还有余量上 B1600 或双 DPU 核。BRAM 若紧张可用低 BRAM 配置（RAM Usage 选项）。计划书里"DPU 小核通常占 ~1000 LUT 级别 DSP 的一部分"这句含糊表述应更正为上表数字。
（注意：以上 DPU 数字是 IP 3.3/v2 时代社区与论文数据，精确值要在 PoC 时用目标工具链跑 report_utilization 复核——但量级结论不会翻。）

**比资源更重要的一个确认**：上游仓库 `rk7100_so101_bound_retry_20261031/` 里有完整的 **`hardware/`（build.tcl、design_1.tcl BD 脚本、rk7100_ps.tcl PS 配置、XDC）+ `sources/`（IK 核 HLS 算法源码 + 测试向量）+ `hls_config.cfg`（7100 目标）+ `scripts/build_all.ps1` 一键构建**，README 明确"使用本机 AMD 2024.2 工具链，依次 HLS 综合与 IP 打包 → hardware/build.tcl → Vitis 平台 → build.py"。
→ **整个 bitstream 是可重建的**（本地 `refs/` 只拷了 firmware+host，是本地缺口不是上游缺口）。这意味着"用 Vivado 2021.1 重建整个工程再加 DPU"没有源码级障碍，只有 HLS/BD 脚本跨 3 年版本的移植回归成本。上游另有一个 `rk7100_so101_native2025_2_20261101_candidate` 目录（命名暗示正用 2025.x 工具链重建），进一步佐证工程跨版本可重建。

## 4. 问题四：IP 获取与许可

- DPU IP 随 Vitis AI 发布（GitHub v1.4 分支含 IP 仓库），**免费**，无单独购买项（Vitis AI 3.5 FAQ："As of the 3.5 release all components are free"；更早版本仅 Optimizer 需免费申请许可，我们用不到）。
- 许可形态：End User License Agreement（产品页），RTL 加密交付（不支持 hw_emu 仿真——PG338 明确"DPU 的 RTL 代码已加密"，**无法在仿真里验证，只能上板**，这会拉长 PoC 周期）。
- 1.4.1 时代的获取渠道（2026 年现状）：GitHub `Xilinx/Vitis-AI` v1.4 分支/tag 仍在；docker 镜像 `xilinx/vitis-ai:1.4.x` 在 Docker Hub（国内可达性待验证）；AMD 官网历史版本下载需登录。

## 5. 对阶段 1 计划书的具体影响

### 5.1 里程碑层面

| 里程碑 | 影响 |
|---|---|
| M0 数据管线 / M1 PC ONNX 基线 | **不受任何路线影响，照做**（float 模型三路线通用） |
| M2 正式训练 | 基本不受影响，但**模型结构要先改**（见 5.2），且训练前应先定路线（量化约束不同） |
| M3 量化 | **分叉点**：DPU → vai_q/vai_c（1.4.1 docker，INT8 per-tensor 时代工具，无 QAT 支持，掉点风险比计划书预期高）；FINN → Brevitas QAT（量化感知，精度上限更高）；hls4ml → 自带定点流 |
| M4 上板 | **原估 5~8 天作废**。DPU 路线 = 工具链降级重建 + PetaLinux 定制 + 控制固件迁移/AMP + 全链路回归，是**周级到月级**的工程；FINN 路线 = 工具流搭建 + 加速器集成进现有 2024.2 工程，也不止 5~8 天但无 OS 迁移 |
| M5 A/B | 不变 |

### 5.2 模型结构（可以现在就定的事）

MobileNetV3-Small 的 hard-swish 与 SE 模块：
- DPU 路线：HardSwish/HardSigmoid 是 **IP 4.0（VAI 2.5）才加入 ALU** 的（V4.0 changelog 明确），而 VAI 2.5 不支持 7000；7000 能用的 IP 3.3 没有 → hard-swish 会整层踢回 CPU 或直接不支持。SE 的 sigmoid+mul 同样存疑。
- FINN 路线：Brevitas 对 ReLU 族支持最好，hard-swish 无现成量化支持。
- hls4ml 路线：计划书自己就写了"模型结构受限"。

→ **三条路线殊途同归：把 backbone 换成纯 ReLU 结构**（MobileNetV2 去掉 hard-swish 的变体 / 全 ReLU 的小型自定义 conv 网 / ShuffleNetV2-ReLU）。这不是损失——我们本来就要为 INT8 友好放弃 heatmap 改直接回归（计划书 §5.3），再加一条"放弃 exotic 激活函数"完全同向。21 关键点直接回归 + 纯 ReLU + ≤1.5M 参数的约束下，结构选择空间仍然充足。

## 6. 待核实清单（本轮没查实的，动手前补）

1. PetaLinux 2021.1 官方宿主 OS 列表（18.04/20.04 记忆待证）与在 2026 年 Ubuntu 24.04 VM 里安装的可行性
2. `xilinx/vitis-ai:1.4.1` docker 镜像在本机网络的拉取可行性
3. GitHub `Xilinx/Vitis-AI` v1.4 分支里 Zynq-7000 的 DPU TRD/IP 仓库完整流程（arch json、B800/B1152 的 7000 配置模板）
4. FINN 当前版本对 Zynq-7000（xc7z100）的支持状态、要求的 Vivado 版本、以及 21 点回归头（非分类/检测输出）在 FINN 里的支持程度——**若走 F3 这是下一个要做的桌面调研**
5. 上游 `native2025_2` candidate 包详情（工具链版本、是否已含新硬件改动）
6. PG338 v3.3 PDF 原版资源表（docs.amd.com 是 JS 应用抓不到，老 xilinx.com PDF 直链已 301 失效；需要从 AMD 登录区或镜像拿）
7. 厂家（RIGUKE）是否提供 RK-ZYNQ7100-F 的 PetaLinux BSP——W1 时就在等，至今未落实，Linux 路线绕不开

## 7. 建议的决策树与下一步

```
主人拍板①：板端最终形态接受 Linux 吗？
├─ 接受 Linux（回到 W1 原路线图）
│   ├─ 拍板②：接受工具链冻结在 Vitis AI 1.4.1 / Vivado 2021.1？
│   │   ├─ 接受 → 路线1（DPU）：先做 §6.1-6.3 核实 → 2021.1 重建工程 PoC（IK 核 HLS 移植回归）→ PetaLinux 定制 → 控制固件迁移
│   │   └─ 不接受 → 路线3（FINN）：Linux 与否皆可，FINN 工具链较新；先做 §6.4 调研
│   └─ （AMP 形态 F2 仅在"Linux 用于视觉、控制必须保裸机硬实时"时考虑，自研风险高）
└─ 坚持裸机（现有已验收形态不动）
    └─ DPU 出局 → 路线3（FINN/hls4ml 定制加速器，Vivado 2024.2 原生集成）
```

无论走哪条，**以下三件事不变、可立即开工**（与计划书 §12 一致）：
1. `label_auto.py` 给 calib_rec.mp4 打标（蒸馏数据管线与部署路线无关）
2. sanity 模型训练（结构改用纯 ReLU 变体，见 §5.2）
3. `--model-kind nn` 开关骨架

若主人倾向裸机/FINN，建议追加一个 1~2 天的 FINN 桌面调研 + 最小 PoC（Brevitas 量化一个玩具网 → FINN 构建 → 资源/延迟报告），与 DPU 路线的核实项并行不冲突。

## 8. 来源

官方：
- [Vitis AI 3.5 IP and Tool Version Compatibility](https://xilinx.github.io/Vitis-AI/3.5/html/docs/reference/version_compatibility.html)（VAI↔IP↔Vivado 对照表）
- [Vitis AI 3.5 FAQ](https://xilinx.github.io/Vitis-AI/3.5/html/docs/reference/faq.html)（bare-metal 不支持、指令集不公开、支持器件家族、组件免费）
- [Vitis AI 3.0 兼容性表](https://xilinx.github.io/Vitis-AI/3.0/html/docs/reference/version_compatibility.html)
- [PG338 v4.1 修订历史（docs.amd.com 中文）](https://docs.amd.com/r/zh-CN/pg338-dpu/%E4%BF%AE%E8%AE%A2%E5%8E%86%E5%8F%B2)（"对 Zynq-7000 器件的支持已终止"，2021-07-22 v3.3 行）
- [AMD Vitis AI 官网](https://www.amd.com/zh-cn/developer/resources/vitis-ai.html)（最新 GA 仅 Versal AI Edge；DPU 归 legacy）
- [AMD DPU IP 产品页](https://www.amd.com/zh-tw/products/adaptive-socs-and-fpgas/intellectual-property/dpu.html)（历史遗留的 Zynq-7000 表述，与现行文档冲突，已辨析）
- [DPU-PYNQ README](https://github.com/Xilinx/DPU-PYNQ)（已归档；VAI 2.5；板卡全为 ZU+）
- Vitis AI 1.0 时代 FAQ（Gitee 镜像）：[doc/faq.md](https://gitee.com/zhong-kai/Vitis-AI/blob/master/doc/faq.md)（"support Zynq-7000 ... Zedboard"）

社区/第三方：
- [DPU TRD V4.0 中文流程（kanhao100）](https://blog.csdn.net/kanhao100/article/details/146571371)（V4.0=VAI 2.5 才加入 HardSwish/HardSigmoid ALU 的 changelog）
- [Vitis AI 基本认知（CSDN）](https://blog.csdn.net/DongDong314/article/details/140701830)（FAQ 中文转述：无裸机/FreeRTOS 支持）
- [高效推理设计实践（CSDN）](https://m.blog.csdn.net/beta5/article/details/151268402)（B1152@XC7Z020 实测资源：83.9% LUT / 100% DSP / 78.9% BRAM，144MHz 166GOPs）
- [亦梦云烟博客](https://blog.csdn.net/u010580016/category_7686878.html)（DPU 最低配置 ≥38457 LUT，7010 不够/7020 可用）
- GitHub topic 检索到的社区项目："DPU IP integration with APU using Zynq-7000"（2021，证明 1.4.1 时代 7000+DPU 有民间落地）

本仓库/上游仓库：
- `refs/26FPGAlerobot/移植及修改进度.md`（7020 初始资源报告：双核 DSP 198/220、LUT 50.55%、BRAM 5.71%；工程目录结构）
- 上游 [rk7100_so101_bound_retry_20261031](https://github.com/xiaoGG2333/26FPGAlerobot/tree/main/rk7100_so101_bound_retry_20261031)（hardware/、sources/、hls_config.cfg、build 脚本；README："Vivado 2024.2 工程，xc7z100ffg900-2，AMD 2024.2 工具链"）
