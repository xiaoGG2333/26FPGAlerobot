# 视觉输入与小型网络

| 环节 | 入口 | 输出 |
|---|---|---|
| 摄像头预览 | [sw/pose_map.py](sw/pose_map.py) | 掌心移动、手部表观大小和腕旋转映射成 POSE1，记录到本目录 results/w3_pose/ |
| 数值仿真 | [test_kin.py](../../sim/vision/test_kin.py)、[verify_channels.py](../../sim/vision/verify_channels.py)、[live_sim.py](../../sim/vision/live_sim.py) | 运动学自检、五通道检查、三维线框预览 |
| 板端串口 | [pose2arm.py](../../board/vision/pose2arm.py)、[pose_link.py](../../board/vision/pose_link.py) | 检查 RK7100 能力标记，分别使用 POSE1 和 GRIP1 控制五轴与夹爪 |
| 小型 CNN | [检测网络](model/detect/)、[分类网络](model/classify/)、[冻结权重](model/frozen/)、[关键点训练工具](tools/) | 独立进行模型训练、推理和评估 |
| 量化 | [模型工具](model/tools/)、[INT8 实验](results/w1_int8/)、[技能包](../../skill/tiny-net-hls/README.md) | 量化尺度、ONNX 对照和 HLS 验证方法 |

### 离线和摄像头预览

以下命令在仓库根目录执行。在独立 Python 环境中，运动学自检需要 NumPy，通道检查另需 OpenCV。摄像头需要带 GUI 的 OpenCV 和提供 Tasks API 的 MediaPipe。打开串口时还需要 pySerial。训练使用 PyTorch；训练数据需另行准备。

```bash
python -m pip install numpy opencv-contrib-python mediapipe pyserial
python sim/vision/test_kin.py
python sim/vision/verify_channels.py --rate
```

将 [MediaPipe Hand Landmarker 模型](https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task) 保存到下列命令指定的位置：

```bash
python src/vision/sw/pose_map.py --source 0 --model ./hand_landmarker.task --no-window --max-frames 60
```

去掉 `--no-window` 可查看图像；用 `--source` 选择相机编号或视频文件。真机联调前应检查零位、坐标方向和串口版本。当前版本中性手势和丢手时可能延续上一夹爪开闭方向。分开验证五轴时使用 `--no-grip`。相机与实物跟踪效果须在使用现场确认。

### 复现范围

整理由[视觉分支固定提交](https://github.com/xiaoGG2333/26FPGAlerobot/tree/0dbaedff3310657b16b7ed6c8b7b16077f2296db/rk7020_so101_hand_teleop_20261102)提供的代码。训练及历史实验脚本中以 `model/`、`tools/`、`results/` 开头的命令，以本目录为工作目录。完整训练集、采集视频与历史实验的外部输入需按对应脚本准备。原始分支仍可查阅实验记录。

当前摄像头遥操作使用主机 MediaPipe。小型 CNN 的冻结权重、INT8 实验和技能包属于独立的模型开发路径。RK7100 比赛工程的 PL 加速器用于逆运动学；本次整理没有将视觉网络写入该位流。`fpga/diag_fw/` 是原分支的诊断代码，独立于比赛固件构建。

### 关键点网络 HLS 实验材料

[IP 源码、权重和测试台](fpga/ip/kpt_net/README.md) · [冻结规格](../../report/vision/kpt_ip_spec.md) · [INT8 参考实现](tools/int8_ref.py)。这些材料来自视觉分支提交 `2f721b5`，对应独立 IP 实验。随附日志是原实验记录。
