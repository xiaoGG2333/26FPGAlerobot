# W1 D1 · PetaLinux 构建环境验证清单

> 目的:第一天就暴露 PetaLinux 构建风险(PLAN §0.4 高等级风险)。
> 执行位置:**构建机**(Ubuntu 22.04 + PetaLinux 2024.2,主人已确认就绪)。
> 本机(Windows)只放清单与记录;厂家 BSP 路径待主人提供。

## 1. 前置确认

- [ ] `source /opt/pkg/petalinux/2024.2/settings.sh` 或等价路径可 source
- [ ] `petalinux-config --version` 输出 2024.2
- [ ] 厂家 BSP 文件(RIGUKE RK-ZYNQ7020-F 配套 .bsp)路径已确认:______(待主人填写)
- [ ] SD 卡 ≥8GB,已分区(FAT32 BOOT + ext4 rootfs)

## 2. 最小镜像构建

```bash
mkdir -p ~/petalinux_w1 && cd ~/petalinux_w1
petalinux-create -t project -s <BSP路径> -n min_check
cd min_check
petalinux-build                    # 最小 rootfs,先不动任何配置
```

## 3. 打包与冷启动

```bash
petalinux-package --boot --fsbl zynq_fsbl.elf --fpga system.bit --u-boot
# 拷贝 BOOT.BIN + image.ub 到 SD FAT32 分区,rootfs 解到 ext4 分区
```

## 4. 验收

- [ ] 板卡插 SD 冷启动,串口(115200 8N1)出现 U-Boot → kernel → **login 提示**
- [ ] 登录(默认 root)后 `uname -a` 正常
- [ ] 记录:构建耗时、BSP 名称版本、任何报错原文 → 写入 `results/w1_diagnostics/petalinux_bootstrap_<ts>.md`

## 5. 失败应对(按顺序)

1. BSP 版本与 PetaLinux 2024.2 不匹配 → 找厂家要 2024.2 配套 BSP
2. 构建报错 → 记录原文,优先查 BSP 自带 README/厂家长群
3. 仍不通 → 启用备选:厂家预编译镜像 + 只改应用层(NCNN 等静态链接上传)
4. 最后评估:裸机 FreeRTOS + 轻量推理(TFLM)保命变体(需主人同意)

## 6. 后续(通过后,D2-3 才做)

- [ ] 在 min_check 基础上加 V4L2/VDMA 摄像头驱动与采集例程,再做视觉 bitstream 联调
