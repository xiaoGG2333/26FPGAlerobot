# HLS 落地与排错

以 Vivado HLS 2018.3（7 系列够用）+ 一张 52 层 int8 小网为背景。每条都是真实踩过的。

## A. 空设计：`has no outputs`、Latency 0、全端口 dangling

```
WARNING: [SYNCHK 200-77] The top function kpt_net_entry (src/top.cpp:319) has no outputs.
```

紧接着会刷几百条 `RTGEN 206-101 Port ..._AWVALID has no fanin or fanout and is left dangling`，那些都是**后果**，不是原因。

**根因**：顶层把已经绑定到 `m_axi` 的指针传给了**一个没有内联的子函数**。HLS 认为该子函数无副作用，把整个调用剪掉——包括无条件的第一层、输入转置、所有输出写。于是 `Detail: Loop: N/A`（一个循环都没调度），DSP=0、Latency=0。

**修法**：顶层加

```c
#pragma HLS INLINE recursive
```

**然后它会变成报错**，暴露真正不合法的写法，通常是这三条：

| 报错 | 原因 | 修法 |
| --- | --- | --- |
| `SYNCHK 200-41 unsupported pointer reinterpretation` | 对 `m_axi uint8_t*` 做 `reinterpret_cast<Struct*>` | 写 `rd32(p, off)` 逐字节拼 int32 |
| `SYNCHK 200-43 use or assignment of a non-static pointer` | 指针做 ping-pong / 指针算术存进变量 | 全改成整数偏移 `sc[base + i]` |
| `SYNCHK 200-61 memory access ... unknown size` | 同上，指针指向未知大小数组 | 同上 |

推论：**输出一定在顶层写**。把 `hm/pres/hand` 的最终写放在 `extern "C"` 顶层函数体内，子模块只写 on-chip 数组（`HM_BUF` / `FC_P` / `FC_H`），顶层再搬出去。

## B. 接口

- `#pragma HLS INTERFACE m_axi port=w offset=slave bundle=gmem_w depth=N` 之后，HLS 会**额外**生成一个同名 `s_axilite` 偏移寄存器，日志里那句 `Setting interface mode on port .../hm to s_axilite & ap_none` 是**正常**的，不是重复绑定。
- 端口重复 pragma 不是空设计的原因（曾经怀疑过，改完症状一模一样）。
- **AXI master 数量是硬预算**：Zynq-7000 的 HP/ACP 端口有限，一张网用 4 个（权重 / 输入 / 激活 scratch / 输出）就到顶。想把 weight/bias/mult/shift 拆成 4 个端口会更清楚，但 7 个 master 不可接受，而且同一 bundle 上混元素宽度不安全。用**单个权重 blob + `rd32` 拼**。

## C. 循环与流水线

- 内层 `cin` 是运行期值 → `XFORM 203-503 cannot completely unroll a loop with a variable trip count`，且 II 上不去。把 `cin` 变成编译期常量。
- `#pragma HLS UNROLL off` 在 2018.3 是**硬错误**（`HLS 200-70 unknown option 'off'`）。想禁止展开只有两条路：改代码结构，或用下面这种「黑盒边界」。
- **volatile 黑盒边界是个陷阱**：

```c
volatile int nblk_v = (cout + NMAC - 1) / NMAC;   // 让 HLS 看不出常量
const int nblk = nblk_v;
for (int b = 0; b < nblk; ++b) { ... }
```

它确实能治「常量版本被强制全展开导致代码爆炸」，但立刻把下一个问题换成 `SCHED 204-65 Loop contains subloop(s) not being unrolled or flattened` 和 `SCHED 204-68 II Violation`——包在最外层的 `PIPELINE II=1` 永远满足不了。**凡是外层要流水、内层要摊平的循环，边界都不要做成不透明。**

- 正确解法是**生成期展开**：代码生成器为每个 co-block 直接吐一条字面量参数的 kernel 调用，根本没有「块循环」存在，也就不需要任何屏蔽技巧。
- 一个静态上限的权重量化表 `static int8_t WT[NMAC][CIN_MAX]` + `ARRAY_PARTITION complete dim=1`，`CIN_MAX` 别开太大（512 深度里实际只用 16~432，多余深度会拖慢 RTL 生成）。

## D. 结构选择：层表循环 vs 直通生成

| | 层表循环（一个 while 走 52 行表） | 直通生成（52 条字面量调用） |
| --- | --- | --- |
| 代码体积 | 小 | 大 |
| 前端耗时 | 预综合 2 分钟 | 预综合 2 小时+ |
| trip count | 运行期，需处理 unroll 告警 | 全编译期，告警消失 |
| **DSP** | **一套 NMAC 阵列复用** | **每层各一套 → 26xNMAC，超预算** |
| 结论 | **可部署** | 只当数值交叉验证 |

判定依据：硬件综合报告里的 DSP 数。直通生成即使功能全对，资源也上不了板。

## E. 大函数惩罚

全内联 52 层之后，架构综合耗时 2 小时 22 分、RTL 生成再 3 小时以上。判断「在跑」还是「死了」：

- 在跑：单核 CPU 涨到 ~93-95%，内存平稳或已回落（某阶段结束时从 3.3 GB 掉到 660 MB）。
- 死了/爆炸：CPU 不动、内存往 GB 级涨。
- 阶段时钟只认 `Finished xxx [cpu 2:12:58, elapsed 2:22:54]` 这种行；`SCHED ... Final II = 1` 不代表阶段。

`csynth.rpt` **只在 `csynth_design` 全结束时写**，中途没有 DSP/II 可读。

想快速拿单层基线：单独建一个只含一层（比如最重的那层 1x1 扩张卷积）的工程，前端 1 分钟、RTL 十几分钟，报告就能给你 DSP/BRAM/II 的真实量级，再按层数外推。

## F. 时钟与调度告警的真正含义

```
SCHED 204-70 Cannot meet target clock period from "sub" to "select" (16.76 ns)
SCHED 204-70 ... from bus read on port gmem_act to bus write on port gmem_act (14 ns)
```

8 ns 目标下 16.76 ns 组合路径 = **时钟根本没收敛**。这不是「再优化一下」的级别，直接降 `NMAC`（64→32 让路径减半）或把 requant 的乘法拆成两级流水。

## G. 工程习惯

- `set_part` / `create_clock` **必须在 `open_solution` 之后**，否则 `HLS 200-71 no active solution`，4 秒就死。
- 功能模型（`std::vector`）**不要**放进 HLS 工程的文件列表，会破坏解析；它只参加 g++ 的 V1/V2 闸门。
- csim 用的测试程序不能接收 `argv`（`csim_design` 会 `exit(2)`），路径写死并按 cwd 逐级重试（cwd 在 `prj/solution1/csim/build`）。
- csim 命令行会定义 `NT` 之类的宏，局部变量同名会 `SIM 211-100`；合成流程不定义，所以这个坑只在 csim 出现（改名即可）。
- 批处理脚本用纯 ASCII（Windows cmd 按 GBK 解析，中文注释会生成乱码命令）；HLS 单线程，同时跑两三个综合几乎不抢 CPU。
- `build.sh` 要把**所有**被脚本调用的测试程序都编一遍；只重编 trace 版会让闸门继续用旧二进制，给出「修完还是错」的假象。

## H. 存储层次

- 权重（几百 KB int8）进 BRAM 没问题；**最大中间张量进不去**（96x96x96 ≈ 864 KB 已经是单层上限，全图 ping-pong 要 ~1.7 MB，把 3.3 MB BRAM 吃光）。
- 可行方案：激活放 **DDR**（`m_axi` scratch + ping-pong 偏移 + skip 快照偏移，全部由生成器算好写成常量），片上只留权重量化表 + 3 行窗口。
- 进一步优化是**行带流式**：每次算 ~12 行输出、带 2 行 halo，把扩张块的中间张量压到 ~130 KB/缓冲；或把扩张→深度→投影三段按行融合，激活流量掉 3 倍。这些都是二期，先让基本结构跑通。
- 深度卷积按行做窗口，stride=1 时相邻输出行会重复读同一批行，代价约 3 倍——先接受，等报告说话。
