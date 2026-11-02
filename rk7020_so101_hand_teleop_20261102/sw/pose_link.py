# -*- coding: utf-8 -*-
"""POSE1 串口链路：与 rk7100 板端固件通信（阶段 0）。

只做协议，不懂视觉。依据 firmware/pose_uart.inc + arm_uart.inc：

- `ARM1 xxx` 的回包是多行，以单独一行 `ARM1 END` 结束
- `POSE1 ...` 的回包是单行 `RESULT1 ...` 或 `ERR1 ...`（没有 END）
- 看门狗 500 ms：持续发 POSE1 即等于喂狗（arm_target 内部会调 arm_keep）
- `active=0` 时 POSE1 只解 IK 不回舵机 —— 可以用作纯 IK 预言机
- POSE1 的 gripper 字段只被 COUNT1 预览，不会驱动 ID6
"""

from __future__ import annotations

import threading
import time
from collections import deque

# HELLO 里必须有这些标记，否则 SD 卡里的 BOOT.BIN / bit / app 不是同一套
MARKERS = (
    'BOARD_RK7100_F_V1',
    'OPT125_SCURVE_20261018_V1',
    'POSITION_ROLL_IK_20261019_V1',
    'SERVO_SPEED200_2000_20261031_V1',
    'BOUND_RETRY_IK_20261031_V1',
)


def encode_pose1(seq, core, pose):
    """按固件 token 规则拼 POSE1：10 个 token，7 个 %.9g 数值。"""
    v = (pose['x'], pose['y'], pose['z'], pose['roll'], pose['pitch'], pose['yaw'],
         pose['gripper'])
    for i, t in enumerate(v):
        if not (t == t and abs(t) != float('inf')):
            raise ValueError(f'POSE1 第 {i} 个数不是有限值: {t!r}')
    return 'POSE1 %d %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g' % (int(seq), int(core), *v)


def parse_result(line):
    """RESULT1 <seq> <code> <iters> <pos> <rot> q0..q4 <grip> -> dict 或 None"""
    t = line.split()
    if len(t) < 12 or t[0] != 'RESULT1':
        return None
    try:
        return {'seq': int(t[1]), 'code': int(t[2]), 'iters': int(t[3]),
                'pos': float(t[4]), 'rot': float(t[5]),
                'q': [float(x) for x in t[6:11]], 'gripper': float(t[11])}
    except (ValueError, IndexError):
        return None


def parse_err(line):
    """ERR1 <seq> <REASON> -> (seq, reason) 或 None"""
    t = line.split()
    if len(t) < 3 or t[0] != 'ERR1':
        return None
    try:
        return int(t[1]), ' '.join(t[2:])
    except ValueError:
        return None


# 当前打开的链路（供异常出口兑底关力矩用）
_LIVE = None


class BoardLink:
    """线程安全的 POSE1 链路：主线程写，守护线程读。"""

    def __init__(self, port, baud=115200, verbose=True):
        self.port, self.baud, self.verbose = port, baud, verbose
        self.ser = None
        self._buf = deque()
        self._lock = threading.Lock()
        self._alive = False
        self._th = None
        self.last_result = None
        self.last_err = None
        self._wlock = threading.Lock()
        self._ka = None
        self._ka_on = False
        self._grip_on = False
        self._grip_dir = ''
        self._last_pose_t = 0.0
        self._last_grip_t = 0.0
        self.counts = {'pose': 0, 'result': 0, 'err1': 0, 'other': 0, 'code0': 0,
                       'grip': 0, 'grip_fail': 0}

    # ---------- 生命周期 ----------
    def open(self):
        import serial                                    # pyserial 只在这里需要
        self.ser = serial.Serial(self.port, self.baud, timeout=0.02, write_timeout=0.6)
        try:                                             # 别让 DTR/RTS 电平去动板子
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:
            pass
        self.ser.reset_input_buffer()
        self._alive = True
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()
        global _LIVE
        _LIVE = self
        return self

    def _reader(self):
        pend = b''
        while self._alive:
            try:
                chunk = self.ser.read(512)
            except Exception:
                break
            if not chunk:
                continue
            pend += chunk
            while b'\n' in pend:
                raw, pend = pend.split(b'\n', 1)
                s = raw.decode('ascii', 'replace').strip()
                if s:
                    with self._lock:
                        self._buf.append(s)

    def close(self):
        self._alive = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    # ---------- 收 ----------
    def _write(self, b):
        with self._wlock:
            self.ser.write(b)

    def keepalive_start(self, period=0.2):
        """后台定时发 ARM1 KEEP + 夹爪保活：视觉循环卡顿也不会触发板端 500 ms 看门狗。"""
        self._ka_on = True
        if self._ka is not None:
            return

        def _run():
            # 只在主循环「卡住」时才补发，正常 20Hz 运行时一条都不多发（115200 带宽有限）
            while self._alive and self._ka_on:
                try:
                    now = time.monotonic()
                    if now - self._last_pose_t > 0.18:
                        self._write(b'ARM1 KEEP' + bytes([10]))
                        self._last_pose_t = now
                    if self._grip_on and self._grip_dir and                             now - self._last_grip_t > 0.18:
                        self._write(('GRIP1 INPUT ' + self._grip_dir).encode('ascii')
                                    + bytes([10]))
                        self._last_grip_t = now
                except Exception:
                    pass
                time.sleep(0.05)

        self._ka = threading.Thread(target=_run, daemon=True)
        self._ka.start()

    def keepalive_stop(self):
        self._ka_on = False

    def grip_set(self, direction):
        """只写不等回包（主循环用，避免阻塞 0.6 s 把两个看门狗都饿死）。"""
        self._grip_dir = direction
        self._grip_on = True
        self._last_grip_t = time.monotonic()
        self._write(('GRIP1 INPUT ' + direction).encode('ascii') + bytes([10]))

    def poll(self):
        with self._lock:
            items = list(self._buf)
            self._buf.clear()
        return items

    def drain(self, hold=0.0):
        """把缓冲区里的回包分类记账，返回本次新到的 RESULT1 列表。"""
        out = []
        for line in self.poll():
            r = parse_result(line)
            if r is not None:
                self.counts['result'] += 1
                if r['code'] == 0:
                    self.counts['code0'] += 1
                self.last_result = r
                out.append(r)
                continue
            e = parse_err(line)
            if e is not None:
                self.counts['err1'] += 1
                self.last_err = e
                if self.verbose:
                    print(f'  [板] ERR1 {e[0]} {e[1]}')
                continue
            if line.startswith('GRIP1'):
                self.counts['grip'] += 1
                self.last_grip = line
                rc = None
                for tok in line.split():
                    if tok.startswith('rc='):
                        rc = tok[3:]
                if rc is not None and rc != '0':
                    self.counts['grip_fail'] += 1
                    if self.verbose:
                        print('  [夹爪] ' + line[:90])
                continue
            self.counts['other'] += 1
            if self.verbose and not line.startswith('ARM1 ALIVE'):
                print(f'  [板] {line}')
        if hold:
            time.sleep(hold)
        return out

    # ---------- ARM1 命令 ----------
    def arm(self, cmd, timeout=2.5):
        """发一条 ARM1 命令，收到 'ARM1 END' 或超时为止，返回回包行列表。"""
        self.poll()                                      # 丢掉陈旧的回包
        self._write((cmd + chr(10)).encode('ascii'))
        t0 = time.time()
        got = []
        while time.time() - t0 < timeout:
            for line in self.poll():
                got.append(line)
                if line == 'ARM1 END':
                    return got
            time.sleep(0.004)
        return got

    def hello(self, timeout=2.0):
        got = self.arm('HELLO', timeout)
        text = ' '.join(got)
        ok = ('POSE1 READY' in text) and all(m in text for m in MARKERS)
        return ok, text

    def state(self, timeout=2.5):
        got = self.arm('ARM1 STATE', timeout)
        kv = {}
        for line in got:
            for tok in line.split():
                if '=' in tok:
                    k, v = tok.split('=', 1)
                    kv[k] = v
        return kv

    def check(self, manual=True):
        got = self.arm('ARM1 TEST_MANUAL_CHECK' if manual else 'ARM1 CHECK', 3.0)
        text = ' '.join(got)
        return text.startswith('ARM1 CHECK_OK') or 'ARM1 CHECK_OK' in text, text

    def enable(self, manual=True):
        got = self.arm('ARM1 TEST_MANUAL_ENABLE' if manual else 'ARM1 ENABLE', 4.0)
        text = ' '.join(got)
        return 'ARM1 READY' in text, text

    def stop(self):
        return self.arm('ARM1 STOP', 3.0)

    def freeze(self):
        return self.arm('ARM1 FREEZE', 2.0)

    def resume(self):
        return self.arm('ARM1 RESUME', 2.0)

    def keep(self):
        return self.arm('ARM1 KEEP', 1.0)

    # ---------- GRIP1 命令（夹爪 ID6：方向式，不是位置式） ----------
    def grip(self, *args, timeout=0.6):
        """发一条 GRIP1 命令，等一行以 GRIP1 开头的回包。

        回包形如 `GRIP1 STATE rc=0 active=1 frozen=0 valid=1 actual=.. goal=..`。
        返回 (ok, dict)，dict 含 rc/active/frozen/valid/actual/goal/direction/torque_off。
        """
        cmd = 'GRIP1' + ((' ' + ' '.join(str(a) for a in args)) if args else '')
        self._write((cmd + chr(10)).encode('ascii'))
        t0 = time.monotonic()
        got = None
        deferred = []                     # 非 GRIP1 的行（大多是 RESULT1）原样放回缓冲区
        while time.monotonic() - t0 < timeout:
            batch = self.poll()
            for line in batch:
                if got is None and line.startswith('GRIP1'):
                    got = line
                else:
                    deferred.append(line)
            if got:
                break
            time.sleep(0.003)
        if deferred:
            with self._lock:
                self._buf.extendleft(reversed(deferred))
        if got is None:
            return False, {'raw': '', 'rc': '-1'}
        kv = {'raw': got}
        for tok in got.split():
            if '=' in tok:
                k, v = tok.split('=', 1)
                kv[k] = v
        try:
            ok = int(kv.get('rc', '-1')) == 0
        except ValueError:
            ok = False
        return ok, kv

    def grip_config(self, closed=1975, opened=3567, rate=200, speed=60, cap=300):
        """端点/速率/速度/扭矩上限。固件要求 closed/opened 在 0~4095 且相差 >=64。"""
        return self.grip('CONFIG', closed, opened, rate, speed, cap)

    def grip_enable(self):
        return self.grip('ENABLE')

    def grip_input(self, direction):
        """direction: 'OPEN' / 'CLOSE' / 'HOLD'。500 ms 内必须重发，否则 GP_WATCHDOG 冻结。"""
        return self.grip('INPUT', direction)

    def grip_resume(self):
        return self.grip('RESUME')

    def grip_stop(self):
        return self.grip('STOP')

    def grip_state(self):
        return self.grip('STATE')

    # ---------- POSE1 ----------
    def state1(self, timeout=1.5):
        """STATE1 回单行：STATE1 <seq> q0..q4 x y z roll pitch yaw <last_seq>"""
        self.poll()
        self._write(b'STATE1 1' + bytes([10]))
        t0 = time.time()
        while time.time() - t0 < timeout:
            for line in self.poll():
                if line.startswith('STATE1 '):
                    return line.split()
            time.sleep(0.005)
        return []

    def last_sequence(self):
        """板端当前已接受的最后一个 seq；新会话必须从它之后开始。"""
        t = self.state1()
        try:
            return int(t[-1])
        except (ValueError, IndexError):
            return 0

    def reset(self, seq):
        """RESET1：pose_q 复位到 home（仅 active=0 时允许），并占掉 seq。"""
        self.poll()
        self._write(('RESET1 %d' % int(seq)).encode('ascii') + bytes([10]))
        time.sleep(0.15)
        return self.poll()

    def pose(self, seq, pose, core=0):
        self._write((encode_pose1(seq, core, pose) + chr(10)).encode('ascii'))
        self._last_pose_t = time.monotonic()
        self.counts['pose'] += 1
