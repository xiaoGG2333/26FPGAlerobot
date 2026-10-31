"""Reproducible controller/driver and calibration tests; no serial port opened."""
from pathlib import Path
import argparse
from datetime import datetime, timezone
import json
import os
import shutil
import subprocess
import sys
import tempfile

package=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--gcc',help='Native GCC executable; not ARM cross compiler')
args=parser.parse_args()
gcc=args.gcc or shutil.which('gcc')
if not gcc and Path(r'D:\Software\minGW\mingw64\bin\gcc.exe').exists():gcc=r'D:\Software\minGW\mingw64\bin\gcc.exe'
if not gcc:parser.error('Native GCC required; pass --gcc. No FPGA/motor hardware is used.')
report={'environment':'native software tests only','date_utc':datetime.now(timezone.utc).isoformat(),
        'hardware_used':False,'serial_port_opened':False,'checks':[],'status':'FAIL'}
try:
    with tempfile.TemporaryDirectory(prefix='rk7020_arm_test_') as folder:
        executable=Path(folder)/('test_arm_control.exe' if os.name=='nt' else 'test_arm_control')
        commands=[
            [gcc,'-std=c99','-O2','-Wall','-Wextra','-Werror',str(package/'tests/test_arm_control.c'),
             *[str(package/'firmware'/name) for name in ('arm_control.c','st3215.c','motor_control.c')],
             '-lm','-o',str(executable)],
            [str(executable)],
            [sys.executable,str(package/'tests/test_calibration.py')]]
        for command in commands:
            run=subprocess.run(command,capture_output=True,text=True,timeout=60)
            report['checks'].append({'command':command,'exit_code':run.returncode,'output':run.stdout+run.stderr})
            print(run.stdout+run.stderr,end='',flush=True)
            run.check_returncode()
    report['status']='PASS'
finally:
    (package/'tests/unit_results.json').write_text(json.dumps(report,indent=2),encoding='utf8')
