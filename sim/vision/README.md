# sim/ —— SO-101 5 轴臂 Python 仿真

## 现状（2026-10-31）

| 文件 | 作用 | 状态 |
|---|---|---|
| `kin.py` | FK / 雅可比 / DLS IK / 关节表 / counts | 完成，自检全绿 |
| `test_kin.py` | 运动学自检 | 完成 |
| `live_sim.py` | 手 -> POSE1 -> IK -> OpenCV 线框 3D 臂（实时 + 离线回放） | 已跑通，实时 30 fps |
| `verify_channels.py` | 5 个视觉通道的增益/线性区间/串扰体检 + 接逆解核前的逐帧体检 | 已跑通（见 docs/接口-视觉5通道-到逆解核-确认.md） |

用法见 `docs/仿真-使用说明.md`。

## 对齐关系（谁是权威）

| 本项目实现 | 权威来源 | 说明 |
|---|---|---|
| `kin.fk()` / `kin.fk_parts()` | `fpga/diag_fw/firmware/ik_iteration.c:ik_fk` | 逐字移植，ET 模型 tx/tz 相同 |
| `kin.ik_error()` | 同上 `ik_error` | 保留 `frame='target'` 分支可与固件逐字对账 |
| `kin.jacobian()` / `kin.solve()` | 板端 HLS IP `xf_solver_lm_so101_0/1` | IP 源码不在库内，这里给等价 DLS |
| `kin.JOINTS` / `q_to_counts` / `gripper_to_counts` | `servo_mapping.h` | zero/sign/限位/counts-per-rad 全同值 |

**IK 迭代格式**（板端在 IP 里，这里的等价实现）：
```
δq = Jᵀ (J Jᵀ + λI)⁻¹ e        λ = damping          (wampler)
                              λ = energy + damping (sugihara)
```
`J` 是 6×5 世界系几何雅可比；旋转误差必须用世界系 `rotvec(R_t R_cᵀ)`，
否则位置块与旋转块符号相反，迭代会振荡（已踩过）。

## 自检结果（2026-10-31）

```
[1] FK(HOME_Q) = pos [0.27265 0.04286 0.23829] rpy [0.15 −0.10 0.20]   偏差 <5e-5  ✅
[2] 固件 SCENARIO0/1 复现：16 / 37 步收敛，q 与固件打印一致              ✅
[3] 在线形态（2mm 小步+热启动）150 帧：1.23 ms/次，平均 2.8 步，全收敛   ✅
[4] 姿态不变只平移 2mm → code=1 MAX_ITER（5 轴固有，非 bug）              ✅
[5] HOME_Q → counts [1991 1960 2154 1856 1939]；夹爪 0/0.5/1 → 1975/2771/3567 ✅
```

## 关键结论（必须记住）

1. **5 轴臂不能任意摆姿态**：目标必须落在可达流形上，也就是
   `yaw = atan2(y, x−0.0612)` 由位置反算（原摇杆链就是这么做的）。
   违反这个条件（例如只平移 2mm 却要求姿态不变）会得到 `code=1 MAX_ITER`，
   这与固件行为一致，不是仿真 bug。
2. `solve()` 失败时返回**最后一步迭代值**（`code=1`），与固件 `ik_solve` 相同。
   上层应据此决定“保留上一帧有效解”还是“显示当前最佳解”。
3. `counts_to_q` 往返误差 6.6e-4 rad ≈ 0.038°，来自 counts 量化（1 count ≈ 0.088°）。

## 用法

```python
import kin
T = kin.fk(kin.HOME_Q)                       # 4x4 末端位姿
Tt = kin.make_target(x, y, z, roll, pitch, yaw)   # 按固件 RzRyRx 顺序造目标
r = kin.solve(Tt, kin.HOME_Q, max_steps=100, tolerance=1e-8, damping=0.01)
r['code'], r['q'], r['pos'], r['rot'], r['iterations'], r['path']
kin.q_to_counts(r['q'])                      # -> (counts, 越界关节下标)
kin.gripper_to_counts(0.5)                   # -> 2771
kin.limit_violations(r['q'])                 # -> ['wrist_roll']
```

## 3D 仿真（`live_sim.py`）要点

- 视图方案 A：同一个 OpenCV 窗口左右分屏（左摄像头 + 手部 HUD，右自绘线框臂），**零 3D 依赖**。
- 投影是自写的球面轨道相机 + 针孔模型（`Cam` / `Scene`），线框按深度压暗。
- `--replay <jsonl>` 不开摄像头、不加载 landmarker，先验证「POSE1 -> IK -> 渲染」。
- 渲染分三层（静态层缓存 / 动态层 / ASCII 文字），实测 3D 渲染 2.7 ms/帧；
  瓶颈是 landmarker 16 ms。中文标签只画一次，动态数值走 `cv2.putText`。
- IK 失败（`code=1 MAX_ITER`）时**沿用上一帧有效解**，画面标黄提示；目标位姿另用半透明 ghost 画出，可直观看偏差。
- 夹爪几何是示意：末端系 x = 腕 roll 轴/接近方向，两指沿 y 分开，开度 = `0.006 + 0.020*gripper`。

## 等接的部分

- 接板子：把 POSE1 真正下发（本轮按任务边界不做）
- 夹爪 ID6 驱动（同上，未接）

