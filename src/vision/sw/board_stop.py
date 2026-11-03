# -*- coding: utf-8 -*-
"""强杀 pose2arm 后恢复用：给板子补发 ARM1 STOP（关扭矩）。用完即删。"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT.parents[1] / 'board' / 'vision'))
import pose_link                                                  # noqa: E402

link = pose_link.BoardLink('COM18', verbose=False)
link.open()
try:
    print('GRIP1 STOP:', link.grip_stop())
    print('ARM1 STOP:', link.arm('ARM1 STOP'))
    print('STATE:', link.state())
finally:
    link.close()
