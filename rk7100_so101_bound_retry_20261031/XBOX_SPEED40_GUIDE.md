# Xbox平移速度上限4 cm/s（2026-10-28）

只修改host/right_config.json和host/right_joycon.py。
新增Xbox专用配置xbox_max_translation_m_s=0.04；其他输入max_translation_m_s=0.02和旋转配置保持原值。
默认命令行速度仍为0.02m/s，显式添加 --translation-speed 0.04 以使用4cm/s。也可指定0.03等较低值。
Xbox命令行速度大于0.04m/s会被拒绝；4cm/s是目标平移速度上限，摇杆偏转小于满量程时会更慢。
逆解、舵机速度/轨迹、温控、编码器映射和限位、扭矩策略都未改动。BOOT与20261019位置优先版完全相同。
无需更新SD卡，不需要重编译FPGA。主机测试使用模拟设备，未驱动实物。

将完整包复制到Ubuntu下载目录，然后：

```bash
source ~/venvs/joycon-rk7020/bin/activate
python -m zipfile -e "$HOME/下载/rk7100_so101_xbox40_20261028.zip" "$HOME/"
cd ~/rk7100_so101_xbox40_20261028
python -u host/xbox_control.py --port /dev/ttyUSB1 --xbox-device /dev/input/event6 --xbox-right-axes rxry --test-drive --manual-control --translation-speed 0.04 --hz 5 --csv results/xbox40.csv
```

端口和手柄event编号按实际设备修改，仍按原流程Enter启用、Ctrl+C停止。
若要恢复2cm/s，将命令行参数改成 --translation-speed 0.02。
目标速度上限提高不保证实物等比例提速；仍受原有S曲线和执行能力约束。
