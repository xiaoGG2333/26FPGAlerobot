# SO101 Xbox 温度过滤、速度100～600版

本次仅在 limits1000_20261011 版本上修改两项：

- 五轴温度反馈沿用55℃阈值。每轴单独计时，连续反馈大于55℃超过1500毫秒时停止并关闭扭矩；不超过1500毫秒的高值不触发温度停止。反馈恢复到55℃或以下，计时清零。读取失败不视为连续高温，沿用原通信故障冻结逻辑。
- 当前五轴同步运动速度参数区间100～600：最大行程轴600，其余轴按原比例计算后最低100。原同步发送与到位保持扭矩逻辑不变。单轴点动、标定工具保持原设置。

实测编码器范围、目标端点10计数余量、模型关节角范围、零点、方向、135mm运动学模型、Xbox输入方向和输入速度、扭矩上限、故障保持与恢复逻辑均保持上一版。温度过滤不代表已经确认高温反馈是假值。

## 更新

开发板断电，备份旧SD卡BOOT.BIN，将本包boot/BOOT.BIN复制到SD启动分区根目录，安全弹出后插回开发板上电。Ubuntu脚本也使用本包：

```bash
source ~/venvs/joycon-rk7020/bin/activate
python -m zipfile -e "$HOME/下载/rk7020_so101_xbox_temp1500_speed100_600_20261011.zip" "$HOME/"
cd ~/rk7020_so101_xbox_temp1500_speed100_600_20261011
python -u host/xbox_control.py --port /dev/ttyUSB1 --xbox-device /dev/input/event6 --xbox-right-axes rxry --test-drive --manual-control --hz 5 --csv results/xbox_drive.csv
```

HELLO应包含LIMITS600_TEMP1500_20261011_V1；启动READY应显示SPEED_PARAMETER_MIN=100 SPEED_PARAMETER_MAX=600 TEMP_LIMIT_C=55 TEMP_FILTER_MS=1500。设备路径可能随USB重连改变。Enter开启控制，Ctrl+C停止并关闭扭矩，其他操作与上一版相同。

## 验证

ARM编译与BOOT.BIN生成通过。模拟总线验证1500毫秒边界、1501毫秒停止、低温重置、每轴独立计时、同步速度100～600及原故障保持逻辑。12,363次编码器往返换算与五轴边界检查通过，34项Python回归测试通过。未进行实物运动测试。
