# tiny-net-hls

**平台说明**　本项目在 RK-ZYNQ7100-F 上运行 ARM 裸机固件，不依赖 PYNQ 软件环境。据项目组说明，技能流程已在 PYNQ 板卡上做过验证；该验证仅涉及技能流程，整套 SO101 系统的实物验证平台为 RK-ZYNQ7100-F。

把一个 PyTorch 小网做成 **int8 + FPGA/HLS 可部署 IP** 的技能包。核心是「软件先于硬件」的 V0–V5 验证阶梯：在没有工具链、没有板子的情况下，用位精确的参考模型把 RTL 之前的算术风险全部消掉。

## 什么时候用

- 训练好的小网要做 int8 量化上板
- 要把 CNN 写成 HLS / RTL
- C 仿真、csim、RTL 和 golden 对不上
- HLS 综合出空设计、Latency 0、资源 0、时序不收敛
- 判断一张网能不能塞进目标器件（MAC / DSP / BRAM / fps）
- 小网训练总是过不了精度门，怀疑结构但其实缺数据

## 使用和安装

在仓库内从 [skill.md](skill.md) 开始。安装到支持技能包的工具时，复制整个 tiny-net-hls 目录，并将安装副本中的 skill.md 改名为 SKILL.md。

## 内容

```
skill.md                     主流程：预算 → 量化 → V1..V5 → 判据
references/
  qdq-math.md                int8 QDQ 定点语义（uint8 激活、requant、add/GAP、ReLU6）
  hls-pitfalls.md            HLS 空设计、INLINE recursive、volatile 陷阱、结构选择
  verification.md            双胞胎法、逐层 trace 对齐、测量异常识别
  data-and-model.md          小网训练：数据域 > 结构、负样本、尺度统计、指标拆分
```

## 设计取舍

- 主文件只放**流程与完成判据**，深材料全部用指针推到 `references/`，按需加载。
- 所有数值都是实测值（不是拍脑袋），并标注了它们是在什么条件下测的——换场景时要重测。
- 判据写成可检查的形式（`≤1.0 LSB`、`0 LSB`、`SYNCHK 200-77 = 0`），避免「大概对了吧」。

## 来源

整理自一个 Zynq7020/7100 手部遥操作项目的实战过程：52 层 int8 关键点网（112.9M MACs）、int8 PTQ、numpy 定点参考、C++ 功能模型、HLS 结构双胞胎、Vivado HLS 2018.3 综合排错，以及配套的域内数据蒸馏训练。所有「真实案例」都是当时的日志与测量。
