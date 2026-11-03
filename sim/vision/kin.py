"""SO-101 5 轴臂运动学（Python 版，与板端固件对齐）

对齐来源：
  fpga/diag_fw/firmware/ik_iteration.c   -> fk() / ik_error() 逐字移植
  fpga/diag_fw/firmware/servo_mapping.h  -> 关节表 zero/sign/限位、q<->counts
  refs/.../host/right_joycon.py:fk_pose  -> 同一 ET 模型（tx/tz），已交叉验证

坐标约定（与固件一致）：
  基座在原点；x 向前、y 向左、z 向上；单位 m/rad。
  FK 的几何 Jacobian 与 ik_error 的旋转误差同在目标系下表达。

IK 迭代格式（板端在 HLS IP 里，这里给等价实现）：
  δq = J^T (J J^T + λI)^-1 e          λ = damping            (wampler)
                                      λ = energy + damping   (sugihara)
"""

from __future__ import annotations

import math

import numpy as np

# ---------------------------------------------------------------- 常量（逐字来自固件）
PI = math.pi
TX = (0.0612, 0.02943, 0.02798, 0.13504, 0.0593)
TZ = (0.0598, 0.05504, 0.11270, 0.00519, 0.00996)

SO101_COUNTS_PER_RAD = 651.8986469  # 4096 / 2π，与固件同值

# name, id, zero, sign, count_min, count_max, q_min, q_max   (servo_mapping.h)
JOINTS = (
    dict(name='shoulder_pan',  id=1, zero=2121, sign=-1, cmin=1164, cmax=2964,
         q_min=-1.27780599631, q_max=1.45267980613),
    dict(name='shoulder_lift', id=2, zero=2156, sign=+1, cmin=828, cmax=3197,
         q_min=-2.02178667843, q_max=1.58153419231),
    dict(name='elbow_flex',    id=3, zero=1893, sign=+1, cmin=876, cmax=3073,
         q_min=-1.54471865340, q_max=1.79475752183),
    dict(name='wrist_flex',    id=4, zero=1986, sign=+1, cmin=874, cmax=3180,
         q_min=-1.69044682825, q_max=1.81623325286),
    dict(name='wrist_roll',    id=5, zero=2037, sign=-1, cmin=170, cmax=3966,
         q_min=-2.94370913195, q_max=2.84860232310),
)
Q_MIN = np.array([j['q_min'] for j in JOINTS])
Q_MAX = np.array([j['q_max'] for j in JOINTS])

HOME_Q = np.array([0.2, -0.3, 0.4, -0.2, 0.15])
GRIPPER_ID = 6
GRIPPER_COUNTS = (1975, 3567)          # 0.0 .. 1.0

# 固件硬范围（姿态目标层）
FW_LIMIT = {'x': (-0.5, 0.6), 'y': (-0.6, 0.6), 'z': (-0.3, 0.7),
            'roll': (-math.pi, math.pi), 'pitch': (-1.4, 1.4),
            'gripper': (0.0, 1.0)}


def wrap_pi(a):
    v = math.fmod(a + PI, 2.0 * PI)
    if v < 0:
        v += 2.0 * PI
    return v - PI


# ---------------------------------------------------------------- FK（逐字移植 ik_fk）
def fk(q):
    """返回 4x4 齐次变换（末端位姿）。"""
    R = np.eye(3)
    p = np.zeros(3)
    for i, ang in enumerate(q):
        c, s = math.cos(ang), math.sin(ang)
        p = p + R[:, 0] * TX[i] + R[:, 2] * TZ[i]
        if i == 0:
            A = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        elif i == 4:
            A = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
        else:
            A = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
        R = R @ A
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def fk_parts(q):
    """返回 (关节轴原点列表, 关节轴方向列表, 末端 4x4)，供画图与 Jacobian 用。"""
    R = np.eye(3)
    p = np.zeros(3)
    origins, axes = [], []
    for i, ang in enumerate(q):
        c, s = math.cos(ang), math.sin(ang)
        p = p + R[:, 0] * TX[i] + R[:, 2] * TZ[i]
        origins.append(p.copy())
        axes.append(R[:, 2 if i == 0 else (0 if i == 4 else 1)].copy())  # 0 绕 z、4 绕 x、其余绕 y
        if i == 0:
            A = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        elif i == 4:
            A = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
        else:
            A = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
        R = R @ A
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return origins, axes, T


def rpy(T):
    """返回 (roll, pitch, yaw)，与固件 Rz(yaw)Ry(pitch)Rx(roll) 分解一致。"""
    R = T[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) < 1e-9:
        roll, yaw = 0.0, math.atan2(-R[0, 1], R[1, 1])
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def make_target(x, y, z, roll, pitch, yaw):
    """按固件顺序 T = T(xyz) · Rz(yaw) · Ry(pitch) · Rx(roll) 造目标位姿。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], float)
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = (x, y, z)
    return T


# ---------------------------------------------------------------- 误差（逐字移植 ik_error）
def ik_error(current, target, frame='world'):
    """6 维误差：前 3 位置（世界系），后 3 旋转 axis*angle。

    frame='world'  旋转误差 = rotvec(R_t R_c^T)，与 Jacobian 的 z 轴同为世界系
                   （求解必须用这个，符号才与位置块一致）
    frame='target' 旋转误差 = rotvec(R_t^T R_c)，与固件 ik_error 逐字一致
                   两种写法的旋转角大小相同，故 rot 残差指标仍与固件可比
    """
    e = np.zeros(6)
    e[:3] = target[:3, 3] - current[:3, 3]
    R = (target[:3, :3] @ current[:3, :3].T if frame == 'world'
         else target[:3, :3].T @ current[:3, :3])
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = float(np.linalg.norm(v))
    tr = float(np.trace(R))
    if n < 1e-6:                                   # 保留固件的近零/近 π 分支
        e[3:] = 0.0 if tr > 0 else (0.5 * PI * (np.diag(R) + 1.0))
    else:
        e[3:] = math.atan2(n, tr - 1.0) * v / n
    return e


def jacobian(q):
    """6x5 几何 Jacobian（世界系）：dq -> d(末端位姿)；误差导数 = -J。"""
    origins, axes, T = fk_parts(q)
    pe = T[:3, 3]
    J = np.zeros((6, 5))
    for i in range(5):
        J[:3, i] = np.cross(axes[i], pe - origins[i])
        J[3:, i] = axes[i]
    return J


# ---------------------------------------------------------------- 求解（对齐 ik_solve）
def solve(target, initial=None, max_steps=200, tolerance=1e-8, damping=0.01,
          sugihara=False, use_limits=False, stall=0):
    """阻尼最小二乘迭代求解。返回 dict(code, q, iterations, energy, pos, rot, path)。

    code: 0 收敛 / 1 迭代用尽 / 3 数值错误（与固件同号）。
    path: 每步的 q 列表，供仿真画轨迹。
    stall: >0 时启用停滞退出——连续 stall 步能量几乎不再下降就直接返回 code=1。
           目标不可达时固件也会跑满 max_steps，仿真每帧白烧 30+ ms，这里提前收。
           默认 0 = 不启用（保持与固件逐字对齐的行为，自检不受影响）。
    """
    q = np.array(HOME_Q if initial is None else initial, float).copy()
    path = [q.copy()]
    info = dict(code=3, q=q, iterations=0, energy=float('inf'), pos=float('inf'),
                rot=float('inf'), path=path)
    if max_steps < 0 or not (tolerance > 0) or not (damping > 0):
        return info
    eye = np.eye(6)
    best, stuck = float('inf'), 0
    for it in range(max_steps + 1):
        cur = fk(q)
        e = ik_error(cur, target)
        p2 = float(e[:3] @ e[:3])
        r2 = float(e[3:] @ e[3:])
        energy = 0.5 * (p2 + r2)
        info.update(iterations=it, energy=energy, pos=math.sqrt(p2), rot=math.sqrt(r2))
        if energy < tolerance:
            info['code'] = 0
            info['q'] = np.array([wrap_pi(v) for v in q])
            path.append(info['q'].copy())
            return info
        if it == max_steps:
            info['code'] = 1                       # MAX_ITER：与固件一样返回最后一步迭代值
            info['q'] = np.array([wrap_pi(v) for v in q])
            return info
        if stall:                                  # 停滞检测：不可达目标不必跑满
            if best < float('inf') and energy > best * (1.0 - 1e-9):
                stuck += 1
            else:
                stuck = 0
            best = min(best, energy)
            if stuck >= stall:
                info['code'] = 1
                info['q'] = np.array([wrap_pi(v) for v in q])
                return info
        lam = (energy + damping) if sugihara else damping
        J = jacobian(q)
        try:
            dq = J.T @ np.linalg.solve(J @ J.T + lam * eye, e)
        except np.linalg.LinAlgError:
            info['code'] = 3
            return info
        if not np.all(np.isfinite(dq)):
            info['code'] = 3
            return info
        q = q + dq
        if use_limits:                              # 求解层默认不限位（与固件一致）
            q = np.clip(q, Q_MIN, Q_MAX)
        path.append(q.copy())
    info['code'] = 1
    info['q'] = np.array([wrap_pi(v) for v in q])
    return info


# ---------------------------------------------------------------- 关节角 <-> counts
def q_to_counts(q):
    """返回 (counts, 出错关节下标或 None)，与 so101_q_to_counts 同语义。"""
    counts = []
    for i, j in enumerate(JOINTS):
        v = float(q[i])
        if not math.isfinite(v) or v < j['q_min'] or v > j['q_max']:
            return None, i
        c = int(round(j['zero'] + j['sign'] * v * SO101_COUNTS_PER_RAD))
        if c < j['cmin'] or c > j['cmax']:
            return None, i
        counts.append(c)
    return counts, None


def counts_to_q(counts):
    return np.array([j['sign'] * (counts[i] - j['zero']) / SO101_COUNTS_PER_RAD
                     for i, j in enumerate(JOINTS)])


def gripper_to_counts(g):
    lo, hi = GRIPPER_COUNTS
    return int(round(lo + max(0.0, min(1.0, g)) * (hi - lo)))


def limit_violations(q):
    """返回越界的关节名列表（供仿真告警）。"""
    out = []
    for i, j in enumerate(JOINTS):
        if q[i] < j['q_min'] - 1e-9 or q[i] > j['q_max'] + 1e-9:
            out.append(j['name'])
    return out
