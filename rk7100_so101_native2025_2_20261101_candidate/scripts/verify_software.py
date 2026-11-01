"""Existing offline regressions only; no serial or motor access."""
from pathlib import Path
import subprocess,sys,json,os,shutil
r=Path(__file__).resolve().parents[1]
gcc=json.loads((r/'config/toolchain.json').read_text()).get('native_gcc') or shutil.which('gcc')
out=r/'build/native';out.mkdir(parents=True,exist_ok=True)
checks=[]
commands=[('bound_guard_compile',[gcc,'-std=c99','-O2','-Wall','-Wextra','-Werror','-I'+str(r/'firmware'),r/'tests/test_bound_retry.c',r/'firmware/ik_iteration.c','-lm','-o',out/'bound_guard.exe']),('bound_guard',[out/'bound_guard.exe']),('bound_comparison',[sys.executable,r/'tests/run_bound_retry_comparison.py']),('host_protocol',[sys.executable,r/'tests/run_position_roll_host_checks.py']),('gripper_host',[sys.executable,r/'tests/test_gripper_host.py']),('failure_continue',[sys.executable,r/'tests/test_ik_failure_continue.py'])]
for name,cmd in commands:
 run=subprocess.run(list(map(str,cmd)),cwd=r,capture_output=True,text=True)
 (r/'reports'/(name+'.txt')).write_text(run.stdout+run.stderr)
 checks.append({'name':name,'exit_code':run.returncode});print(name,run.returncode,flush=True)
 if run.returncode:break
(r/'reports/software_validation.json').write_text(json.dumps({'checks':checks,'hardware_used':False,'status':'PASS' if len(checks)==len(commands) and all(x['exit_code']==0 for x in checks) else 'FAIL'},indent=2))
if any(x['exit_code'] for x in checks):raise SystemExit(1)
