# LT/RT独立夹爪控制（2026-10-30）

已增加独立ID6命令与驱动。LT按住关闭，RT按住开启；同时按下或全部松开时，在状态切换时读取一次实际位置并锁定保持，不追随外力漂移，不继续执行旧开合累计。
扳机按下阈值为20%，采用固定可配置开合速度。右摇杆仍为Z及ID5，左摇杆仍为X/Y。
只识别独立ABS_Z/ABS_RZ扳机轴，和ABS_RX/ABS_RY右摇杆；不把复用摇杆轴当作扳机。
夹爪命令与POSE1求解分离。正常会话中五轴逆解失败不会拒绝夹爪开合。
会话启动仍需通过原五轴预检；通信/心跳/芯片异常会按原会话策略冻结，六轴明确停止统一关闭扭矩。

## 当前状态：端点已采集，尚未进行运动验证

host/gripper_config.json的verified=true，闭合目标2047、张开目标3486。只有显式添加--gripper-control才启用。
没有--gripper-control时沿用原操作，不新增ID6运动或使能命令；原有六轴停止流程保留。
使用--gripper-control但端点未确认时，在打开串口前拒绝启动。不要为跳过校验随意填写端点或将verified改为true。
自然闭合2037、张开3496已各采集5组一致读数；两端向内各留10计数余量。未执行自动开合或抓取测试。
标定配置还会与ID6当前EEPROM位置范围做只读核验。不会改写EEPROM或零点。

## 停止、冻结与扭矩

启动：读取范围/位置/状态，设置RAM扭矩上限，将当前位置写为保持目标，确认使能成功后进入正常控制。
尚未完成启动时失败：尝试关闭ID6扭矩；整个主机启动失败按原流程执行ARM1 STOP。
正常运行通信异常、心跳超时：保存上一有效目标，不发送关扭矩命令，冻结新目标并报错。
显式恢复：重新读取位置、温度和扭矩状态，清除累计，从实际位置保持；必须释放LT/RT才能恢复。
ID6温度>55℃持续>1500ms：沿用会话温度停止策略，六轴关闭扭矩。
芯片温度策略、五轴编码器范围和所有逆解/轨迹/速度设置不变。
正常Ctrl+C/程序退出或ARM1 STOP：关闭六轴扭矩，并在能确认关闭时恢复保存的ID6 RAM扭矩上限。
通信失败时无法保证舵机实际扭矩状态；软件只保证不主动发送关扭矩命令。
夹爪保持开度不等于恒力抓取；夹持效果需标定后实测。

## 更新和使用

PL位流与位置优先版相同；ARM和主机有改动，需要更新SD BOOT.BIN及主机文件。旧启动文件和压缩包保留。
新标记：GRIPPER_LT_RT_20261030_V1。

Ubuntu解压新包：
```bash
source ~/venvs/joycon-rk7020/bin/activate
python -m zipfile -e "$HOME/下载/rk7100_so101_gripper_calibrated_20261030.zip" "$HOME/"
cd ~/rk7100_so101_gripper_calibrated_20261030
```
不启用夹爪时继续五轴控制，沿用原命令，不添加--gripper-control。
端点已采集，准备显式启用夹爪时可使用：
```bash
python -u host/xbox_control.py --port /dev/ttyUSB1 --xbox-device /dev/input/event6 --xbox-right-axes rxry --test-drive --manual-control --translation-speed 0.04 --hz 5 --gripper-control --gripper-config host/gripper_config.json --csv results/gripper_drive.csv
```
状态输出GRIPPER actual/goal为真实ID6反馈及目标；原CSV/counts中的第6项仍为POSE预览字段，不代表独立夹爪目标。
默认开合目标变化率100计数/s、舵机协议速度50、RAM扭矩上限200。允许范围分别1..400、1..100、1..500，实际扭矩上限不提高到原有RAM上限以上。
配置和目标只写RAM，断电不保留；限位/方向标定JSON保存在电脑上。

固件编译和离线测试已通过；端点经实物只读采集，未执行自动开合或抓取测试。
