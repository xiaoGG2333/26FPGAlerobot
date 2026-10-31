# 五轴执行速度上限 1500 计数/秒

ID1/ID5：1000→1500；ID2/ID3/ID4：1000→1500。轨迹最大速度、速度寄存器上限及RAM配置校验同步更新。
保留加速度、加加速度、速度下限、轨迹时钟及规划方式、编码器限位、逆解失败保持、温度保护、扭矩策略和ID6夹爪速度。
Xbox最大20cm/s及左推Y+右推Y-均已包含。
1500计数/秒约131.84度/秒，是上限，不代表每次运动都达到该速度；短距离仍受加速度和加加速度约束。

## 更新
1. 停止当前控制程序，支撑机械臂，开发板和舵机断电。
2. 备份SD卡原BOOT.BIN，将本包 boot/BOOT.BIN 复制到SD卡启动分区根目录，替换旧BOOT.BIN，安全弹出。
3. 复制压缩包到Ubuntu下载目录并解压，进入新目录。
4. 开发板从SD重新启动，串口HELLO应包含 SERVO_SPEED1500_20261030_V1。新版本仍按Enter才启用运动，Ctrl+C停止。

```bash
source ~/venvs/joycon-rk7020/bin/activate
python -m zipfile -e "$HOME/下载/rk7100_so101_servo1500_20261030.zip" "$HOME/"
cd ~/rk7100_so101_servo1500_20261030
python -u host/xbox_control.py --port /dev/ttyUSB1 --xbox-device /dev/input/event6 --xbox-right-axes rxry --test-drive --manual-control --translation-speed 0.20 --hz 10 --gripper-control --gripper-config host/gripper_config.json --csv results/servo1500.csv
```

构建与离线测试通过，未驱动实物。旧BOOT和旧包仍保留，可回退。
