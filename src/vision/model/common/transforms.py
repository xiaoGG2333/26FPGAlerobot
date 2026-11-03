"""公共工具: 数据增强、指标、配置。

增强仅用: 几何(仿射裁剪/平移/缩放)、光度(亮度/对比度)、运动模糊、水平翻转。
不用: 遮挡/剪切(演示场景无)、颜色抖动过强(与演示光照偏差大)。
"""
import math
import random

import cv2
import numpy as np

IMG_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMG_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def normalize(img):
    x = img.astype(np.float32) / 255.0
    return (x - IMG_MEAN) / IMG_STD


def photometric(img, rng):
    """亮度/对比度小扰动 + 高斯噪声(轻微)。"""
    a = float(rng.uniform(0.75, 1.25))          # 对比度
    b = float(rng.uniform(-0.2, 0.2))           # 亮度
    x = np.clip(img.astype(np.float32) * a + b * 255.0, 0, 255)
    if rng.random() < 0.3:
        x = x + float(rng.gauss(0, 3.0))        # 轻噪声
    return np.clip(x, 0, 255).astype(np.uint8)


def motion_blur(img, rng):
    """模拟快速挥动(演示场景核心增强)。"""
    k = rng.choice([3, 5, 7, 9])
    angle = float(rng.uniform(0, 360))
    m = cv2.getRotationMatrix2D((k / 2, k / 2), angle, 1)
    kernel = np.diag(np.ones(k, dtype=np.float32)) / k
    kernel = cv2.warpAffine(kernel, m, (k, k))
    return cv2.filter2D(img, -1, kernel)


def crop_origin(rng, dim, crop, center=None, keep_center=False, margin_frac=0.05, random=True):
    """裁剪起点。keep_center=True 时保证中心 center 落在裁剪窗口内(留 margin_frac 边距)。

    用于修复"中心裁剪把标注中心裁掉 → 标签不可达"的管线 bug。
    """
    hi = max(0, dim - crop)
    lo = 0
    if keep_center and center is not None:
        m = margin_frac * crop
        lo = max(lo, int(math.ceil(center - crop + m)))
        hi = min(hi, int(math.floor(center - m)))
        if lo > hi:                      # 中心本就在画面外 → 保底：尽量居中放置
            return int(min(max(round(center - crop / 2), 0), max(0, dim - crop)))
    return rng.randint(lo, hi) if random else (lo + hi) // 2


def geom_params(h, w, size, augment, rng, center=None, keep_center=False, zoom_min=0.88):
    """几何变换参数: 短边缩放 -> 裁剪窗口 -> 翻转。返回 (scale, (nh,nw), (y0,x0,ch,cw), flip)。

    center: 归一化 (cu, cv) 标注中心；keep_center=True 时裁剪窗口保证包含它（训练时用，
            避免标签被裁出画面导致目标不可达）。

    裁剪策略（2026-10-14 修订）: **训练窗口与评估/部署对齐**。
      - 评估/部署: 短边缩到 size 后取 size×size 中心裁剪
      - 训练: 同尺寸的 k×k 方窗 (k=size×U[zoom_min,1])，随机位置 + 翻转
        zoom_min 默认 0.88（历史行为）；调小 => 多尺度增强（手在 144 输入里显得更大，
        针对小品手定位），但注意 k 不可能 > size（短边已是 size，视野已用满）
    旧版训练取"近全图非方窗"(192×144) 再压成 144×144 → 视野与纵横比都与评估不一致
    （同一只手在 val 里看上去约 1.33 倍高），是系统性训练/评估失配。
    """
    scale = size / min(h, w)
    nh, nw = int(round(h * scale)), int(round(w * scale))
    if augment:
        k = int(round(size * rng.uniform(zoom_min, 1.0)))
        k = max(8, min(k, nh, nw, size))
        ch = cw = k
        cy = cx = None
        if center is not None:
            cx, cy = center[0] * nw, center[1] * nh
        y0 = crop_origin(rng, nh, ch, cy, keep_center)
        x0 = crop_origin(rng, nw, cw, cx, keep_center)
        flip = rng.random() < 0.5
    else:
        ch, cw, flip = size, size, False
        y0, x0 = (nh - size) // 2, (nw - size) // 2
    return scale, (nh, nw), (y0, x0, ch, cw), flip


def apply_geom(img, geom, size):
    """应用几何变换: 裁剪 -> resize -> 可选翻转。"""
    _, _, (y0, x0, ch, cw), flip = geom
    img = img[y0:y0 + ch, x0:x0 + cw]
    if img.shape[:2] != (size, size):
        img = cv2.resize(img, (size, size), interpolation=cv2.INTER_AREA)
    if flip:
        img = img[:, ::-1]
    return img


def geom_oob(h, w, size, rng, center, d_lo=0.05, d_hi=0.30, zoom=(0.88, 1.0)):
    """构造「标注中心落在窗口之外」的裁剪窗口（生成"目标手不在窗口里"负样本）。

    与正样本训练同一条规则：短边缩到 size 后取 k x k 窗口，k = size*U[zoom]。
    窗口沿某一轴平移，把标注中心推到窗口外 d in [d_lo,d_hi]*k（另一轴随机取位，与正样本一致）。
    这样窗口里保留"手被画面边缘裁掉/几乎看不见"的真实形态，而不是纯空场景。

    k 取 size 时只有约 9% 的正样本推得出去（HaGRID 标注中心集中在画面中部）；
    跟随训练用的缩放抖动后可用率显著提高，且窗口尺度与正样本分布一致。
    本帧推不出去时返回 None，由调用方回退。
    """
    scale = size / min(h, w)
    nh, nw = max(size, int(round(h * scale))), max(size, int(round(w * scale)))
    k = int(round(size * rng.uniform(zoom[0], zoom[1])))
    k = max(8, min(k, nh, nw, size))
    cx, cy = center[0] * nw, center[1] * nh
    opts = []
    for axis, dim, c in ((0, nw, cx), (1, nh, cy)):
        if dim < k:
            continue
        d = float(rng.uniform(d_lo, d_hi)) * k
        q = int(round(c - d)) - k                 # 窗口放在中心"小坐标侧"
        if q >= 0:
            opts.append((axis, q))
        q = int(round(c + d))                     # 窗口放在中心"大坐标侧"
        if q <= dim - k:
            opts.append((axis, q))
    if not opts:
        return None
    axis, q = opts[rng.randrange(len(opts))]
    other = nh if axis == 0 else nw
    o = crop_origin(rng, other, k)                 # 另一轴随机位置（正样本训练同口径）
    x0, y0 = (q, o) if axis == 0 else (o, q)
    return scale, (nh, nw), (y0, x0, k, k), False


OOB_ZOOM_STAGES = ((0.88, 1.0), (0.75, 0.90))     # 先按正样本同款缩放；再不行就放宽窗口尺度


def oob_feasible(h, w, size, center, seed=0, tries=4, d_lo=0.05, d_hi=0.30):
    """本图能否造出"中心在窗口外"的窗口（用于数据集预筛，不读像素）。

    与 load_jpeg_oob 用同一套两段策略，保证"预筛通过"≈"实际能造出来"。
    """
    rng = random.Random(seed)
    for zoom in OOB_ZOOM_STAGES:
        for _ in range(tries):
            if geom_oob(h, w, size, rng, center, d_lo, d_hi, zoom) is not None:
                return True
    return False


def load_jpeg_oob(path, size, augment, rng, center, d_lo=0.05, d_hi=0.30, tries=8):
    """读图并生成"目标中心在窗口外"的负样本。推不出去时返回 (None, None)。

    单次随机抽取可能失败（缩放抖动/推的方向是随机的），所以重抽几何若干次；
    图像只解码一次。
    """
    img = cv2.imread(str(path))
    if img is None:
        raise ValueError(f'无法读取 {path}')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    h, w = img.shape[:2]
    geom = None
    for zoom in OOB_ZOOM_STAGES:                # 两段：先正样本同款尺度，再放宽
        for _ in range(max(1, tries)):
            geom = geom_oob(h, w, size, rng, center, d_lo, d_hi, zoom)
            if geom is not None:
                break
        if geom is not None:
            break
    if geom is None:
        return None, None
    _, (nh, nw), _, _ = geom
    img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    img = apply_geom(img, geom, size)
    if rng.random() < 0.5:                     # 左右翻转扩大多样性（conf=0，无坐标要映射）
        img = img[:, ::-1]
    if augment and rng.random() < 0.5:         # 与域内负样本一致：只加光度抖动
        img = photometric(img, rng)
    return img, geom


def map_point(u_norm, v_norm, geom):
    """原图归一化坐标 -> 变换后 size×size 图上的归一化坐标。"""
    scale, (nh, nw), (y0, x0, ch, cw), flip = geom
    pu, pv = u_norm * nw, v_norm * nh
    pu, pv = pu - x0, pv - y0
    if flip:
        pu = cw - pu
    return pu / cw, pv / ch


def load_jpeg(path, size, augment, rng, center=None, keep_center=False, zoom_min=0.88):
    """读图 -> 短边缩放 -> 裁剪/翻转 -> 光度/模糊增强。返回 (np.uint8 HxWx3)。"""
    img = cv2.imread(str(path))
    if img is None:
        raise ValueError(f'无法读取 {path}')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    h, w = img.shape[:2]
    geom = geom_params(h, w, size, augment, rng, center, keep_center, zoom_min)
    scale, (nh, nw), _, _ = geom
    img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    img = apply_geom(img, geom, size)
    if augment:
        if rng.random() < 0.5:
            img = photometric(img, rng)
        if rng.random() < 0.4:
            img = motion_blur(img, rng)
    return img


def load_jpeg_geom(path, size, augment, rng, center=None, keep_center=False, zoom_min=0.88):
    """同 load_jpeg, 额外返回几何参数(供坐标同步映射)。

    center/keep_center: 见 geom_params —— 训练时传标注中心 + keep_center=True，
    保证裁剪不会把标注中心裁出画面（修复不可达标签）。
    """
    img = cv2.imread(str(path))
    if img is None:
        raise ValueError(f'无法读取 {path}')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    h, w = img.shape[:2]
    geom = geom_params(h, w, size, augment, rng, center, keep_center, zoom_min)
    scale, (nh, nw), _, _ = geom
    img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    img = apply_geom(img, geom, size)
    if augment:
        if rng.random() < 0.5:
            img = photometric(img, rng)
        if rng.random() < 0.4:
            img = motion_blur(img, rng)
    return img, geom


def image_size(path):
    """只读文件头取 (w, h)，不解码整图（JPEG/PNG）。失败返回 None。

    用于数据集预筛（如判断某图能否构造"中心在窗口外"的负样本）；
    本环境没装 pillow，单独为读尺寸引入依赖不值，用 20 行解析代替。
    """
    try:
        with open(str(path), 'rb') as f:
            sig = f.read(2)
            if sig == b'\xff\xd8':                    # JPEG: 找 SOF 段
                while True:
                    b = f.read(1)
                    if not b:
                        return None
                    if b != b'\xff':
                        continue
                    while b == b'\xff':
                        b = f.read(1)
                    m = b[0]
                    if m in (0xd8, 0x01) or 0xd0 <= m <= 0xd7:
                        continue
                    ln = int.from_bytes(f.read(2), 'big')
                    if 0xc0 <= m <= 0xcf and m not in (0xc4, 0xc8, 0xcc):
                        f.read(1)                       # precision
                        h = int.from_bytes(f.read(2), 'big')
                        w = int.from_bytes(f.read(2), 'big')
                        return w, h
                    f.seek(ln - 2, 1)
            if sig == b'\x89P':                        # PNG: IHDR 紧跟签名
                f.read(12)
                w = int.from_bytes(f.read(4), 'big')
                h = int.from_bytes(f.read(4), 'big')
                return w, h
    except Exception:
        return None
    return None


def bbox_center_px(bbox, w, h):
    """归一化 bbox [x,y,w,h] -> 中心点像素 (u, v)。"""
    bx, by, bw, bh = bbox
    return (bx + bw / 2) * w, (by + bh / 2) * h


def center_error_stats(preds, gts, img_sizes):
    """中心点误差(像素): 还原到原始分辨率。返回 dict。"""
    errs = []
    for (u, v), (gu, gv), (w, h) in zip(preds, gts, img_sizes):
        errs.append(math.hypot(u * w - gu * w, v * h - gv * h))
    errs = np.array(errs)
    return {'mean_px': float(errs.mean()), 'median_px': float(np.median(errs)),
            'p90_px': float(np.percentile(errs, 90)), 'max_px': float(errs.max())}
