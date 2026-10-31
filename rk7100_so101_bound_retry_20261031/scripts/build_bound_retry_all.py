from pathlib import Path
import subprocess,sys,time,os
r=Path(__file__).resolve().parents[1];v=Path(r'D:\Software\vivado2024.2');cfg=r/'hls_bound_retry.cfg';work=r/'hls_bound_retry'
steps=[('bound_hls_synth',[v/'Vitis/2024.2/bin/v++.bat','-c','--mode','hls','--config',cfg,'--work_dir',work]),('bound_hls_cosim',[v/'Vitis/2024.2/bin/vitis-run.bat','--mode','hls','--cosim','--config',cfg,'--work_dir',work]),('bound_hls_package',[v/'Vitis/2024.2/bin/vitis-run.bat','--mode','hls','--package','--config',cfg,'--work_dir',work]),('bound_vivado',[v/'Vivado/2024.2/bin/vivado.bat','-mode','batch','-source',r/'scripts/build_bound_retry.tcl','-log',r/'logs/bound_vivado.log','-journal',r/'logs/bound_vivado.jou']),('bound_boot',[sys.executable,r/'build.py','--hardware',r/'build_bound_retry/hardware_export','--output',r/'boot_bound_retry_20261031'])]
task_tmp=r/'tmp_bound_retry';task_tmp.mkdir(exist_ok=True)
os.environ['TEMP']=os.environ['TMP']=str(task_tmp)
if os.name=='nt':os.environ['LOCALAPPDATA']=str(Path.home()/'AppData/Local')
cwd=r/'build_tools_bound_retry';cwd.mkdir(exist_ok=True)
for name,args in steps:
 print('START',name,time.strftime('%H:%M:%S'),flush=True)
 with (r/'logs'/(name+'_console.log')).open('w',encoding='utf-8') as stream:rc=subprocess.run([str(a) for a in args],cwd=cwd,stdout=stream,stderr=subprocess.STDOUT).returncode
 print('END',name,'code',rc,time.strftime('%H:%M:%S'),flush=True)
 if rc:raise SystemExit(rc)
print('BOUND_RETRY_BUILD_COMPLETE',flush=True)
