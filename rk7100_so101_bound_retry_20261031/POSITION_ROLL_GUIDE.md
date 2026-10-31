# 位置优先逆解更新（2026-10-19）

本版只修改逆解任务：XYZ位置 + ID5腕旋转关节角；末端俯仰、偏航不再作为独立约束。
仍控制原来的腕旋转轴心参考点，未改成爪尖。底座会为满足XYZ自然调整方向。

从实际编码器关节角开始迭代，采用阻尼最小范数增量、每次迭代最大0.08rad更新及下降验收，优先保持局部连续。
这是局部近邻选择，不承诺全局关节变化最小。0.08rad是内部数值更新上限，不是机械臂运动幅度限位。
迭代中限制候选关节角，最终仍使用原有编码器/关节限位审核及整组目标接受机制。

未更改：舵机驱动、五轴S曲线协调、速度配置、零点与方向映射、编码器范围、温控及扭矩/冻结/停止策略。
也未检查或修改前述轨迹执行节拍及额外实物反馈记录。

PS仍完成迭代，PL仍完成单步矩阵求解，125MHz不变。PL增加任务矩阵选择，AXI地址及寄存器数量不变。
正lambda保留原全姿态自测；负lambda表示新XYZ+ID5模式，内部使用其绝对值作为阻尼。
主机与新BOOT必须一起更新，启动标记 POSITION_ROLL_IK_20261019_V1。
原boot、boot_opt125、boot_no_workspace_20261019和旧包均保留。

## 使用

开发板断电后，备份SD卡BOOT.BIN，再替换为此包boot/BOOT.BIN。新BOOT同时含匹配的PL位流和ARM程序。
将压缩包复制到Ubuntu“下载”目录，执行：

```bash
source ~/venvs/joycon-rk7020/bin/activate
python -m zipfile -e "$HOME/下载/rk7100_so101_position_roll_20261019.zip" "$HOME/"
cd ~/rk7100_so101_position_roll_20261019
python -u host/xbox_control.py --port /dev/ttyUSB1 --xbox-device /dev/input/event6 --xbox-right-axes rxry --test-drive --manual-control --hz 5 --csv results/position_roll.csv
```

端口和手柄event编号按实际设备修改。先确认启动包含新标记；进入运动仍沿用原有Enter流程。
Xbox左摇杆X/Y、右摇杆上下Z、右摇杆左右ID5方向不变。
现在移动XYZ时夹爪允许自动俯仰；右摇杆左右仍独立控制腕旋转。
正常温度不周期输出；只有异常温度状态按旧策略反馈。

CSV字段数量不变。target_roll现在明确表示ID5关节角，target_pitch/target_yaw是保留兼容字段，不是求解约束。
rotation_error_rad现在表示ID5任务残差，不能与旧版完整姿态旋转误差直接比较。
STATE1线路格式不变，主机用其q[4]作为ID5参考，避免自由俯仰越过90度造成Euler roll跳变。

仍可能存在真正超出总臂长或关节范围的目标，新任务不保证所有目标可达。
无碰撞检测；本版构建验证不驱动实物。日志回放是数学回归，不等同于实物已经验证。

## 验证与回退

新任务和旧全姿态任务均执行HLS C仿真及RTL联合仿真；最终构建状态见validation.json。
独立数值参考日志回放可运行 tests/run_position_roll_log_regression.py（需要本机C编译器）。
受保护文件的SHA256核对见reports/position_roll_scope.json。
若回退，请同时换回旧包host和对应旧BOOT，避免混用任务语义。
