"""V1/V2 两级对拍：C++ 功能模型 vs ORT int8（V1），HLS 结构版 vs 功能模型（V2）。

用法：
  python -u tools/check_hls_model.py            # 默认 6 帧
  python -u tools/check_hls_model.py --n 10

V2 要求逐位一致（最大差 0），这是 HLS 综合前必须过的最后一道软件门槛。
"""
import argparse
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
IP = ROOT / 'fpga/ip/kpt_net'
sys.path.insert(0, str(ROOT / 'tools'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cache', default='results/stage1/cache_cam480_v3')
    ap.add_argument('--n', type=int, default=6)
    ap.add_argument('--stride', type=int, default=60)
    a = ap.parse_args()

    cache = ROOT / a.cache
    wbin = IP / 'data/kpt_weights.bin'
    build = IP / 'build'
    build.mkdir(exist_ok=True)
    exe = build / 'tb_kpt_hls.exe'

    src = [IP / 'tb/tb_kpt_hls.cpp', IP / 'src/kpt_net.cpp', IP / 'src/kpt_hls.cpp']
    cmd = ['g++', '-O2', '-std=c++17', '-Wno-unknown-pragmas',
           '-I', str(IP / 'data'), '-I', str(IP / 'src'),
           *[str(p) for p in src], '-o', str(exe)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print('编译失败：\n' + (r.stderr or r.stdout))
        return 1
    print('[编译] 通过')

    imgs = np.load(cache / 'imgs.npy', mmap_mode='r')
    idx = list(range(0, min(len(imgs), a.n * a.stride), a.stride))[:a.n]
    inp = build / 'input.bin'
    bad = 0
    for k in idx:
        im = cv2.resize(np.asarray(imgs[k]), (192, 192), interpolation=cv2.INTER_AREA)
        im.tofile(inp)
        p = subprocess.run([str(exe), str(wbin), str(inp)], capture_output=True, text=True)
        head = p.stdout.strip().splitlines()
        tag = 'OK ' if p.returncode == 0 else 'FAIL'
        if p.returncode != 0:
            bad += 1
        print(f'  #{k:4d} [{tag}] ' + ' | '.join(head[:3]))

    print(f'\n[V2 结论] {len(idx) - bad}/{len(idx)} 帧逐位一致'
          + ('' if bad == 0 else '  ← 还有不一致，不能进 HLS 综合'))
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
