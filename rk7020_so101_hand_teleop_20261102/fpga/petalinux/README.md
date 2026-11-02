# fpga/petalinux/ · PetaLinux 构建产物与脚本(W1 D2-3 起使用)

本目录存放视觉版 Linux 镜像的构建脚本与配置。

- 当前状态:W1 D1,先按 `docs/w1_d1_petalinux_checklist.md` 验证最小镜像(在构建机执行)
- 待厂家 BSP 路径确认后,把构建脚本(.sh)、device-tree、内核/rootfs 配置落本目录
- 视觉版 bitstream(采集时序 + VDMA + 单逆解核 + 急停 GPIO)在 `fpga/bd/` 下管理
- 所有实测(构建耗时、镜像大小、启动日志)写入 `results/`
