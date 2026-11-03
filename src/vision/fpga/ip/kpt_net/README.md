# kpt_net —— 阶段1 手部关键点网 IP

规格见 `docs/IP规格-阶段1-关键点网-冻结.md`。

当前阶段：**V2.5 已通过** —— HLS 结构版（NMAC=64 / NCH=32 并行）与功能模型
**逐位一致**（热图 0 LSB、presence/handed 完全相同）；功能模型与 onnxruntime int8 差 0.88 LSB。
代码层面综合前置条件已清完：无 `std::vector`、无 `double`、无动态内存。
V3 综合**尚未运行**。工具是本机已装的 **Vivado HLS 2018.3**
（`D:\FPGAivado\Vivado8.3inivado_hls.bat`，lic 覆盖到 2037）。

## 目录

```
data/    kpt_layers.h / kpt_params.h / kpt_weights.bin   ← 由 tools/gen_ip_header.py 自动生成
src/     kpt_net.h / kpt_net.cpp   ← 定点功能模型（算法基准）
         kpt_hls.cpp               ← HLS 结构版（并行/pragma/顶层 kpt_net_top）
tb/      tb_kpt_net.cpp            ← V1：功能模型 vs onnxruntime int8
tb/      tb_kpt_hls.cpp            ← V2：结构版 vs 功能模型，要求逐位一致
hls.tcl         Vivado HLS 2018.3 综合脚本（未跑过）
hls_vitis.tcl   同上的 Vitis 版（将来迁移用）
build.sh 一条命令编三个可执行文件
build/   编译产物
```

## 重新生成层表与权重

```
D:/Anaconda3/envs/train-pi/python.exe -u tools/gen_ip_header.py \
    --onnx results/stage1/self_run5_192/int8/best_hm_int8.onnx
```

## 编译功能模型

```
cd fpga/ip/kpt_net
./build.sh              # 需要 g++（本机 MinGW 14.2 已装）
```

## 对拍（必过门槛）

```
# V1：功能模型 vs onnxruntime int8
tools/check_cpp_ip.py --n 6          # 期望热图平均 ≤1.0 LSB

# V2：HLS 结构版 vs 功能模型
tools/check_hls_model.py --n 6       # 期望【最大差 0】
```

历史问题记录（都修过了，别再犯）：

| 症状 | 原因 |
|---|---|
| 整网全饱和 | 输入是 HWC，内部按 CHW 索引 |
| 边界一圈全错 | 量化域零填充用了 0，应该是 x_zp |
| 残差后全乱 | Add 的 a/b 支路搞反，且漏了各自 zp |
| GAP/FC 全饱和 | GAP 没减输入 zp；pres/hand 两个头是并列的，不是串联 |
| 只有 hm 对、pres 错 | 同上 |
| HLS 版热图对、pres/hand 错 | FC 隐藏层边读 `FC_IN` 边写 `FC_IN`，后面的输出用的是被踩过的输入；改成先写 `FC_HID` 再整体搬回 |
| HLS 版整网偏亮/饱和 | `b_off/m_off/s_off` 是**各自段内的索引**（功能模型写的是 `P.b + L.b_off`），不是 blob 绝对字节偏移，漏加段基址 |
| 综合出空电路（Latency 0 / DSP 0 / 端口全悬空） | 顶层指针绑成 m_axi 后传给了非内联子函数 → 顶层必须 `#pragma HLS INLINE recursive`；内联后才报出下面两条真错误 |
| `SYNCHK 200-41 unsupported pointer reinterpretation` | 在 m_axi 的 `uint8_t*` 上 `reinterpret_cast<int32_t*>`；改成 `rd32()` 逐字节拼小端 int32 |
| `SYNCHK 200-43/200-61 non-static pointer / unknown size` | `cur`/`nxt`/`skip` 指针乒乓；改成整数偏移，全部写成 `sc[base + i]` |
| csim 报 `expected unqualified-id before numeric constant` | 局部常量取名 `NT`，撞了 HLS csim 命令行预定义的 `-DNT=...` 宏 |

## 存储/定点契约（V2.6）

| 缓冲 | 位置 | 大小 |
|---|---|---|
| 激活（A/B 乒乓 + SKIP） | **外部 DDR**，`kpt_net_entry` 的 `scratch` 指针 | `KPT_SCRATCH_BYTES` = 2×884736 + 147456 = **1.83 MB** |
| 逐点层权重块 WT[64][cin] | 片上 BRAM | ≤ 32 KB |
| 逐点/逐通道的 bias/mult/shift 块 BT/MT/ST | 片上 BRAM | 3×64×4 B = 768 B |
| 逐通道权重块 WD[32][3][3] | 片上 BRAM | 288 B |
| FC 缓冲 FC_IN/FC_HID[256] | 片上 BRAM | 512 B |
| 三路输出 HM_BUF[3024] / FC_P[1] / FC_H[2] | 片上 BRAM | 3027 B |

所有系数都是整数：`am`/`bm` 是残差和 GAP 的 **Q20** 定点系数（GAP 的 ÷(h·w) 已折进 `am`），
所以整条数据通路没有浮点，综合不会引入浮点 IP。

## HLS 结构版（kpt_hls.cpp）做了什么

| 层型 | 并行度 | 结构 |
|---|---|---|
| 逐点 1×1（26 层，占总 MAC 84%） | **NMAC=64** 输出通道并行 | 权重块先搬进 64 个 bank 的 BRAM（WT[64][cin]），像素循环 II=1 |
| 逐通道 3×3（13 层） | **NCH=32** 通道并行 | 权重块 WD[32][3][3]，每输出行读 3 行输入 |
| 首层 3×3 s2（1 层） | 24 路输出通道并行 | 3×3 全展开直算 |
| 残差/GAP/FC | 流水化标量 | 计算量可忽略 |

**激活放 DDR，权重和窗口放 BRAM**（这是 V2 的结构决策）：最大单层激活 96×96×96=864 KB
放不进 BRAM，但 DDR 完全无所谓——每帧激活读写 ~15-20 MB，30fps 约 500 MB/s。
真正稀缺的资源是 DSP：NMAC=64 约 64 片 DSP48E1（占 2020 片的 3%），
30fps 需要 1.76M 周期 = **14.1 ms/帧**（预算 33.3 ms）。
phase-2 优化是把 逐点+逐通道 合成行带流水，激活读量降 3 倍（现在是每行读 3 次）。

## 下一步

1. **V3 综合**：`vivado_hls -f hls.tcl -l hls.log`，看 125 MHz 收不收敛、
   资源与估算（DSP ~64、BRAM ~40 个 36K、LUT 待测）是否对得上。
   **判定成功的两个标志**：hls.log 里不再有 `SYNCHK 200-77 has no outputs`，
   csynth.rpt 里 DSP48 接近 64、Latency 不为 0。
2. **如果 II 达不到 1**：把 WT 分块改成循环次序 (ci 外层 / 像素内层) 的权重驻留版，
   或者降低 NMAC 到 32 换频率。日志里现有的两条
   `cannot completely unroll a loop with a variable trip count` 就是这个问题。
3. **phase-2 行带流水**：把 block 内三层合成一条流水线，激活不再落 DDR。
4. 预处理（ROI 方块 + INTER_AREA 192）定点化并与 cv2 对拍。

## 调试用：两路逐层跟踪

```
./build/tb_hlstrace.exe data/kpt_weights.bin build/input.bin
```
前 52 行是功能模型（`-DKPT_TRACE`，打印的是**本层之前**的缓冲），
后 52 行是 HLS 结构版（`-DKPT_HLS_TRACE`，打印的是**本层输出**）。
对比时要错一位对齐：HLS 的 L(i) ↔ 功能模型的 L(i+1)。
本次就是靠它把“热图已经逐位一致，只有 pres/hand 错”定位到 FC 读写踩踏。
