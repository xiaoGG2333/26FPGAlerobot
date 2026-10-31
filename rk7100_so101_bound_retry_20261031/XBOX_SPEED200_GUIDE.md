# Xbox 平移速度上限 20 cm/s

仅修改 host/right_joycon.py 和 host/right_config.json 的 Xbox 平移速度配置上限，从 0.04 改为 0.20 m/s。默认值仍为 0.02 m/s；ID5转速及舵机速度、轨迹、限位、温控、逆解失败处理不变。
无需更新 SD 卡。可将上述两个文件覆盖到 Ubuntu 当前工程的 host 目录，沿用原命令并修改 --translation-speed 0.20。
该参数是末端目标更新速度上限，不保证实物一定达到20cm/s，仍受关节速度、加速度、轨迹和可达性约束。

完整包使用：
```bash
source ~/venvs/joycon-rk7020/bin/activate
python -m zipfile -e "$HOME/下载/rk7100_so101_xbox200_20261030.zip" "$HOME/"
cd ~/rk7100_so101_xbox200_20261030
python -u host/xbox_control.py --port /dev/ttyUSB1 --xbox-device /dev/input/event6 --xbox-right-axes rxry --test-drive --manual-control --translation-speed 0.20 --hz 10 --gripper-control --gripper-config host/gripper_config.json --csv results/manual_tuning.csv
```
按原流程 Enter 启用运动，Ctrl+C 停止。此次验证为离线测试，未驱动机械臂。
