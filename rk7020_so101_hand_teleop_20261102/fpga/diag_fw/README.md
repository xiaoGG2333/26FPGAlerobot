# fpga/diag_fw/ · 板上 IK 单解耗时精确测量固件

> 目的:精确测量"板上 IK 单解耗时"(ARM 调 PL 逆解核单步 Δq 的完整迭代),排除串口往返。
> 判定:D1 验收标准为板上单解 <5ms。
> 红线:本目录是**原固件的打点副本**,原仓库与最新包 BOOT.BIN 不受任何影响。

## 内容

- `firmware/` — 原最新包固件副本,已应用打点(pose_uart.inc 追加 `TIME1 <seq> <board_us>` 行,RESULT1 格式不变)
- `apply_board_timing.py` — 打点应用脚本(幂等,可重跑)
- `measure_board_ms.py` — 主机侧配合脚本,解析 TIME1 行并判定

## 构建(需构建机:Vitis 2024.2 + 板级 BSP,与原仓库 build.py 同环境)

1. 按原包 `build.py` 的流程编译 `firmware/`,只换固件源码目录为本目录的 `firmware/`(打点版)
2. 生成的 ELF 与**原最新包 bitstream** 组合出新的 BOOT.BIN(bit 不动,只换 ARM ELF)
3. 临时 SD 卡冷启动(与演示卡分开),确认 HELLO 正常

## 测量

```bash
cd fpga/diag_fw
python measure_board_ms.py --port COM16 --count 12 --out ../../results/w1_diagnostics/<时间戳>/board_ik
```

- 判定:`board_us_median < 5000` → PASS(<5ms)
- 结果落 `results/` 并注明固件版本(打点版,基于 20261011 包)与条件

## 回退

打点版只用于诊断;演示统一用原最新包 BOOT.BIN。若打点版行为异常,直接用原包恢复,并把异常记录进 results。
