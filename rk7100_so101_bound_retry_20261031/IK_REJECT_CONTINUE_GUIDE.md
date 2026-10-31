# 逆解失败继续运行版（2026-10-30）

逆解失败：拒绝此次目标，继续执行上一次有效目标，到位后保持扭矩；打印原因，撤销未执行的输入累计，等待下一次目标。
手动控制原本已有此行为；本次补齐非手动模式的旧退出分支。
板端实际停止、温度保护、通信冻结、主动停止和程序退出处理不变。
算法、角度映射、编码器范围、速度、轨迹和夹爪控制不变。

只需替换 Ubuntu 现有工程的 host/right_joycon.py。无需更新 SD 卡，不用重新烧录。
也可以解压本完整包后进入新目录，沿用原运行命令。

```bash
source ~/venvs/joycon-rk7020/bin/activate
python -m zipfile -e "$HOME/下载/rk7100_so101_ik_reject_continue_20261030.zip" "$HOME/"
cd ~/rk7100_so101_ik_reject_continue_20261030
python -u host/xbox_control.py --port /dev/ttyUSB1 --xbox-device /dev/input/event6 --xbox-right-axes rxry --test-drive --manual-control --translation-speed 0.04 --hz 5 --gripper-control --gripper-config host/gripper_config.json --csv results/gripper_drive.csv
```

Enter 按原流程启用运动；Ctrl+C 仍会发送 STOP 并关闭扭矩。
测试仅使用模拟串口和控制器，没有驱动机械臂。
