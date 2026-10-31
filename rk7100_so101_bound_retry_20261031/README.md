# Current update: OPT125_SCURVE_20261018_V1

See [Chinese operating guide](使用指南_OPT125.md), [design details](OPT125_GUIDE.md), and reports/optimization_validation.json. New output is boot_opt125/; original boot/ is retained. Sections below describe the prior 100MHz baseline.

# RK-ZYNQ7100-F / SO101 Xbox 工程

本目录 D:\VivadoProjects\RK7020_So_101 是后续工作的唯一工程目录，目标器件为 xc7z100ffg900-2。目录名中的7020沿用用户指定名称，工程本身针对7100。

迁移基线：rk7020_so101_xbox_temp1500_speed100_600_20261011。保持两核、100MHz、地址0x40000000/0x40010000，以及原算法、数据类型、完整迭代、Xbox输入、五轴同步控制、零点方向、编码器端点、100..600速度参数、55℃连续超过1.5秒温度保护和原故障/扭矩策略。仅适配PS/DDR/MIO、封装引脚、BSP/FSBL与板卡提示。

## 工程入口

- build/rk7100_so101.xpr：Vivado 2024.2工程。
- hardware/：PS配置、Block Design和UART1引脚约束。
- sources/：原HLS算法和测试向量；hls_config.cfg指定7100目标。
- third_party/：本地Vitis Libraries 2024.2头文件。
- firmware/：ARM裸机应用。
- host/：Ubuntu Xbox控制和诊断脚本，保持原版。
- build/vitis_workspace/platform：新XSA生成的Vitis平台。
- boot/：7100启动文件，BOOT.BIN仅用于新板。
- reports/、logs/：验证与构建记录。
- reference/：原始来源、厂商PS参考和迁移前源码SHA256。

## 硬件连接

DEBUG USB接电脑/虚拟机，115200 8N1；7100上为UART0 MIO14 RX / MIO15 TX。Windows当前COM16，Ubuntu枚举路径需检查。
舵机使用PS UART1经EMIO，1Mbps：J8第3脚(AK16) TX→适配器RX，J8第4脚(AJ16) RX←适配器TX，J8第1脚GND共地。BANK10/VCC_ADJ2按3.3V使用；断电核对接口方向和电压，适配器跳线沿用已验证的位置。舵机保留原独立12V电源。
板载风扇由PL独立运行：25kHz、正常50%，>=65℃自动100%，降至60℃恢复50%；有效温度采样中断1秒或读数异常时全速。软件在>=75℃或传感器异常时拒绝新IK目标并冻结五轴新控制，不关闭舵机扭矩、保留最后有效目标。冷却至65℃以下后按Enter人工恢复，降温本身不会恢复运动。风扇转速按厂商每转2脉冲换算并反馈。

## SD与Ubuntu更新

先支撑机械臂，停止旧控制程序，断开舵机电源后更新。备份旧SD的BOOT.BIN，将本工程boot/BOOT.BIN复制到SD启动分区根目录。7100板SW1两位OFF为SD启动（ON=0，两位OFF对应SD启动），断电切换后重新上电。旧7020的BOOT.BIN不能用于此板。

将packages/rk7100_so101_thermal_20261016.zip复制到Ubuntu下载目录，逐行执行：

```bash
source ~/venvs/joycon-rk7020/bin/activate
python -m zipfile -e "$HOME/下载/rk7100_so101_thermal_20261016.zip" "$HOME/"
cd ~/rk7100_so101_thermal_20261016
python -m serial.tools.list_ports -v
python host/xbox_input.py --list
python scripts/verify_board.py --port /dev/ttyUSB1 --servos --out results/board_readonly.json
```

先只验证计算（此命令不启用舵机运动）：

```bash
python -u host/xbox_control.py --port /dev/ttyUSB1 --xbox-device /dev/input/event6 --xbox-right-axes rxry --hz 5 --csv results/xbox_preview.csv
```

启动串口应包含BOARD_RK7100_F_V1 CHIP_THERMAL1_V1。原协议中的RK7020兼容名称及LIMITS600_TEMP1500_20261011_V1保留，保证原上位机匹配行为不变。启动时原单步/双核完整迭代自测仍会执行。检查自测和SERVO1只读反馈成功后，才进行实物控制。

现场准备完成后，使用原控制命令：

```bash
python -u host/xbox_control.py --port /dev/ttyUSB1 --xbox-device /dev/input/event6 --xbox-right-axes rxry --test-drive --manual-control --hz 5 --csv results/xbox_drive.csv
```

Enter开启控制、Ctrl+C明确停止并关闭扭矩，原断连冻结/恢复流程不变。设备路径可能重新编号。左右摇杆映射与7020版一致，夹爪ID6仍沿用原预览逻辑。

## 重新构建

使用本机AMD 2024.2工具链。依次进行HLS综合与IP打包、Vivado hardware/build.tcl、Vitis scripts/build_platform.py、python build.py。不要使用旧7020的BSP或FSBL。scripts/check_source_parity.py可检查本次迁移的严格源码差异。

一键构建入口：PowerShell执行 `./scripts/build_all.ps1`；仅重编ARM应用和BOOT执行 `./scripts/build_all.ps1 -Stage Boot`。已有Vivado工程使用rebuild_hardware.tcl重新构建，保留当前Block Design。

具体已通过项目和未执行的上板检查以reports/validation.json为准。主机仿真、编译和时序通过不能替代7100实物验证。

## 温度与风扇显示

串口新增独立THERMAL1帧，原RESULT1、STATE1格式不变。Ubuntu坐标输出前会显示类似（仅格式示例，不是实测）：

```
CHIP: 42.50 C | fan_pwm=50% rpm=2700 | NORMAL
seq=... CONVERGED ...
q_deg: ...
```

CSV末尾追加chip_temp_c、fan_pwm_pct、fan_rpm、chip_thermal_state、chip_sensor_ok。
NORMAL=50%；FULL=100%；HOT_HOLD=拒绝新IK目标且保持舵机目标/扭矩；SENSOR_FAULT=全速且禁止新的五轴控制。初始未取得有效冷却温度时，固件不运行启动计算自测，会持续输出温度状态。

片上温度和舵机温度独立：原舵机55℃连续超过1.5秒关闭扭矩逻辑没有修改。片上75℃冻结是新增保护，不使用舵机的1.5秒延迟。片上温控不改变已经接受的舵机目标；舵机会保持或继续完成该目标。

必须同时更新SD的BOOT.BIN与Ubuntu控制包。板载风扇需已正确连接并有12V供电，开机先确认实际转动和RPM反馈。开机等待约3秒后，RPM持续为0应检查连接，不继续实物联调。控制逻辑不能弥补风扇断电、散热器脱落等硬件故障；若全速后温度仍升高，应停止负载并检查散热。此版尚未实测板上温度/转速。

依据AMD UG480的XADC连续扫描、16次平均和温度换算实现：
https://docs.amd.com/r/en-US/ug480_7Series_XADC/Instantiating-the-XADC
新增RTL测试：scripts/run_thermal_sim.ps1、scripts/run_xadc_sim.ps1；后者用45/70/80/55℃激励验证实际XADC原语及DRP读取。舵机通信和运动测试未自动执行。

