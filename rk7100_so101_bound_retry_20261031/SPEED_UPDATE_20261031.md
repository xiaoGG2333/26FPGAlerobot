# Xbox 5cm/s + 五轴速度200..2000计数/秒（2026-10-31）

Xbox平移配置上限0.05m/s，默认仍0.02m/s；左推Y+、右推Y-保持。
五轴在线轨迹的舵机速度寄存器限幅统一200..2000计数/秒；轨迹速度上限2000。
加速度和加加速度不变。200是速度寄存器参数下限，不强迫实际关节始终以200运动：起停和小位移仍受轨迹约束。
初始化/保持位置写入使用原有参数，ID6夹爪速度保持50。逆解、编码器限位、温控、故障扭矩策略不变。

请备份SD原BOOT.BIN，断电后用本包boot/BOOT.BIN替换SD启动分区根目录文件再重启。
HELLO应包含SERVO_SPEED200_2000_20261031_V1；只同步Ubuntu文件不能更新板端固件。

Ubuntu沿用venv：
```bash
source ~/venvs/joycon-rk7020/bin/activate
cd ~/rk7100_so101_xbox50_servo200_2000_20261031
python -u host/xbox_control.py --port /dev/ttyUSB1 --xbox-device /dev/input/event6 --xbox-right-axes rxry --test-drive --manual-control --translation-speed 0.05 --hz 10 --gripper-control --gripper-config host/gripper_config.json --csv results/drive.csv
```
Enter按原流程启用运动，Ctrl+C停止并关闭扭矩。测试和同步未自动启动运动。
