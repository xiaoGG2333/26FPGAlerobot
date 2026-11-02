"""数值验证：单目下 '翻腕(旋前/旋后 σ)' 与 '转画面内(ψ)' 是否可区分。
用简化手模型 + 与 pose_map 完全相同的几何，算各候选特征对三种运动的敏感度。
"""
import math
import numpy as np

# ---- 简化手（相机坐标：x 右, y 上, z 指向场景深处；掌心朝相机 => 法线 [0,0,-1]）----
def hand_points():
    p = np.zeros((21, 3))
    p[0] = [0, 0, 0]                                     # 腕
    mcp_y = {'5': 0.090, '9': 0.095, '13': 0.090, '17': 0.080}
    mcp_x = {'5': 0.030, '9': 0.010, '13': -0.010, '17': -0.030}
    for k, idx in (('5', 5), ('9', 9), ('13', 13), ('17', 17)):
        p[idx] = [mcp_x[k], mcp_y[k], 0.0]
    # 指节/指尖：沿 +y 延伸，带一点 z=0（掌心平面）
    tip = {'5': (6, 7, 8), '9': (10, 11, 12), '13': (14, 15, 16), '17': (18, 19, 20)}
    for k, (a, b, c) in tip.items():
        base = p[int(k)]
        p[a] = base + [0, 0.030, 0]
        p[b] = base + [0, 0.055, 0]
        p[c] = base + [0, 0.075, 0]
    p[1] = [-0.030, 0.010, 0]
    p[2] = [-0.050, 0.040, 0]
    p[3] = [-0.060, 0.060, 0]
    p[4] = [-0.065, 0.075, 0]
    return p


def rot(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def features(p):
    """p: 21x3。返回 (θ, φ, ψ, asin(x_z), MCP线角, 法线-指轴差角)。"""
    p2 = p.copy()
    d2 = p2[9] - p2[0]
    theta = math.atan2(d2[1], d2[0])
    n = unit(np.cross(p[5] - p[0], p[17] - p[0]))
    if n[2] > 0:                                          # 统一到掌心朝相机
        n = -n
    tw = math.atan2(p[9][1] - p[0][1], p[9][0] - p[0][0])
    ca, sa = math.cos(-tw), math.sin(-tw)
    nx = ca * n[0] - sa * n[1]
    ny = sa * n[0] + ca * n[1]
    nz = abs(float(n[2])) + 1e-9
    phi = math.atan2(nx, nz)
    psi = math.atan2(ny, nz)
    xh = unit(p[9] - p[0])
    axis_tilt = math.asin(max(-1.0, min(1.0, xh[2])))
    mcp_vec = p[17] - p[5]
    mcp_ang = math.atan2(mcp_vec[1], mcp_vec[0])
    n_az = math.atan2(n[1], n[0]) - tw                      # 法线方位（去掉画面内旋转）
    n_az = math.atan2(math.sin(n_az), math.cos(n_az))
    return np.array([theta, phi, psi, axis_tilt, mcp_ang, n_az])


NAMES = ['θ 画面内', 'φ 法线去旋', 'ψ', 'asin(x_z)指轴', 'MCP线角', '法线方位-θ']

FOREARM = np.array([0, 0, 1.0])     # 前臂轴（从肘指向腕，掌心朝相机时≈光轴）
FLEX = np.array([1.0, 0, 0])        # 屈伸轴（横贯掌面）
INPLANE = np.array([0, 0, 1.0])     # 画面内旋转轴（= 相机光轴）

base = hand_points()


def sens(tilt_deg):
    """tilt: 整只手绕屈伸轴倾斜，让掌面偏离光轴。返回 3 个运动对 6 个特征的敏感度(度/度)。"""
    Rt = rot(FLEX, math.radians(tilt_deg))
    f_axis = Rt @ FOREARM        # 前臂轴随姿势一起转
    h = math.radians(0.5)
    out = {}
    for tag, axis, pts in (('翻腕σ', f_axis, None), ('屈伸β', Rt @ FLEX, None),
                           ('转画面ψ', INPLANE, None)):
        up = features((rot(axis, h) @ base.T).T)
        dn = features((rot(axis, -h) @ base.T).T)
        out[tag] = np.degrees((up - dn) / (2 * h))
    return out


print(f'{"姿势":>10} | ' + ' | '.join(f'{n:>12}' for n in NAMES))
for tilt in (0, 15, 30, 45, 60):
    s = sens(tilt)
    print(f'{"掌倾%d°" % tilt:>10} |')
    for tag in ('翻腕σ', '屈伸β', '转画面ψ'):
        print(f'{tag:>9} | ' + ' | '.join(f'{v:12.2f}' for v in s[tag]))

print('\n关键：看 "翻腕σ" 与 "转画面ψ" 两行的差别（差=可区分，几乎相同=退化）')
for tilt in (0, 15, 30, 45, 60):
    s = sens(tilt)
    a, b = s['翻腕σ'], s['转画面ψ']
    d = np.abs(a - b).max()
    # 只取两通道 (θ, asin(x_z) 或 φ) 组成 2x2，看条件数
    M = np.array([[s['翻腕σ'][0], s['屈伸β'][0]],
                  [s['翻腕σ'][1], s['屈伸β'][1]]])
    cn = np.linalg.cond(M) if np.linalg.det(M) != 0 else float('inf')
    print(f'  掌倾 {tilt:2d}°: max|翻腕-转画面| = {d:6.2f} 度/度   2x2 条件数(θ,φ) = {cn:8.2f}')
