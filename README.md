# 26FPGAlerobot

RK-ZYNQ7100-F（xc7z100ffg900-2）与 SO101 右臂的 FPGA 逆运动学硬件加速工程。ARM 负责完整迭代、轨迹及舵机控制，双 PL IP 负责单步求解，PL 时钟 125 MHz。

## 最新 2025.2 候选版本

目录：[rk7100_so101_native2025_2_20261101_candidate](rk7100_so101_native2025_2_20261101_candidate/)。

- 使用 Vivado/Vitis 2025.2，从源码创建 HLS IP、Vivado 工程、位流/XSA、平台/BSP、FSBL 和 ARM 应用，生成配套 BOOT.BIN。
- 保留 XYZ 位置优先加 ID5、自由俯仰、限位感知与附近初值重试；保留五轴协调、编码器映射、夹爪、温控和扭矩策略。
- 修复主机协议兼容性：HELLO 包含 `BOUND_RETRY_IK_20261031_V1` 时，接受累计最多360次迭代（200+80+80）；旧固件仍检查200。避免合法重试结果被误判为 `Invalid result`。
- Xbox 平移配置上限5 cm/s；正常五轴速度参数200～2000计数/秒；左推Y+、右推Y−；LT关闭夹爪、RT打开夹爪。

## 构建与运行

先阅读[迁移及验收指南](rk7100_so101_native2025_2_20261101_candidate/migration_guide.md)。

Windows 安装匹配的 Vivado/Vitis 2025.2、Zynq-7000器件支持和许可证，使用Python 3。将新版本目录复制到短路径（例如 `D:/fpga2025`，根路径不超过44字符且不含空格），配置 `config/toolchain.json`，运行：

```text
python scripts/build_all.py
```

工程不依赖旧版XPR或预先存在的BSP。固定保留的 `Vitis_Libraries_2024.2` 是数学库源代码，不代表使用2024.2工具。Ubuntu主机依赖见 `host/requirements.txt` 和 `config/host_requirements_lock.txt`，沿用独立venv。

候选启动文件：[board/boot/BOOT.BIN](rk7100_so101_native2025_2_20261101_candidate/board/boot/BOOT.BIN)。使用同一目录的配套位流与固件；不要混用不同发布版本。主机协议修复本身无需再次更换BOOT。

## 验证与限制

| 检查 | 结果 |
|---|---|
| 2025.2 HLS C仿真及649笔RTL协同仿真 | 通过 |
| 1878个软件逆解对比 | 无新增退化；不等于真实板端速度或精度测量 |
| 29项主机协议、夹爪与失败处理回归 | 通过 |
| 新增累计迭代协议测试 | Windows、Ubuntu通过 |
| 轨迹、阻尼、风扇及温控回归 | 通过 |
| 125 MHz最终实现 | WNS +0.382 ns，WHS +0.024 ns |
| FPGA资源 | LUT 11.79%，FF 6.90%，DSP 9.41%，BRAM 0.53% |
| 新平台/BSP、ARM编译、BOOT打包 | 通过 |
| Ubuntu包同步与协议修复同步 | 已完成，未自动启动运动 |
| 完整实机验收 | 尚未完成，不以旧版本或零散运行日志代替 |

`MAX_ITER`表示本次未求得合格解，不是几何不可达的全局证明。正常拒绝目标保留上次接受目标与扭矩；格式、序号及非有限数值检查仍保留。模型与实物的绝对精度需要独立验证。

[构建验证报告](rk7100_so101_native2025_2_20261101_candidate/reports/migration_validation.json)｜[源码与适配校验](rk7100_so101_native2025_2_20261101_candidate/reports/runtime_parity.json)｜[寄存器兼容性](rk7100_so101_native2025_2_20261101_candidate/reports/register_abi.json)

## 旧版与文件完整性

保留 [2024.2 bounded-retry版本](rk7100_so101_bound_retry_20261031/BOUND_RETRY_GUIDE.md) 作为原样对照和回退；本次没有删除或覆盖该目录。2025.2候选版本通过完整实机验收后，再作为后续正式工作基线。

新目录的发布文件按 `manifests/release_sha256.json` 校验；主机兼容性适配单独记录在 `manifests/migration_compatibility_sha256.json`，ARM/PL与舵机逻辑未因此修改。Git关闭自动换行转换以保持文件字节。构建缓存、虚拟环境和SSH凭据不上传。

保留原有第三方许可证声明，本次不替上游或项目重新指定许可证。协作及历史见[移植及修改进度.md](移植及修改进度.md)。
