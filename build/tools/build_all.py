"""2025.2 clean build; never opens serial/JTAG or starts motion."""
from pathlib import Path
import argparse,subprocess,sys,os,json,hashlib,datetime
r=Path(__file__).resolve().parents[1]
if os.name=='nt' and (len(str(r))>44 or ' ' in str(r)):
 raise SystemExit('Use a short project root (<=44 characters, no spaces), e.g. D:/fpga2025. Vivado nested IP paths have Windows length limits.')
p=argparse.ArgumentParser();p.add_argument('--from-stage',default='check');p.add_argument('--through',default='boot');a=p.parse_args()
c=json.loads((r/'config/toolchain.json').read_text());amd=Path(c['amd_root']);v=amd/'Vitis';vivado=amd/'Vivado/bin/vivado.bat'
for folder in ['build','logs','reports','build/tmp']:(r/folder).mkdir(parents=True,exist_ok=True)
env=os.environ.copy();env['TEMP']=env['TMP']=str(r/'build/tmp');env['LOCALAPPDATA']=str(Path.home()/'AppData/Local')
# Avoid inherited older AMD setup; every top-level program is absolute.
for key in ['XILINX_VITIS','XILINX_VIVADO','XILINX_HLS'] : env.pop(key,None)
env['PATH']=os.pathsep.join(x for x in env.get('PATH','').split(os.pathsep) if '2024.2' not in x)
cfg=r/'build/hls.cfg'
cfg.write_text('part='+c['part']+'\n[hls]\nflow_target=vivado\nsyn.top=xf_solver_lm_so101\n'+
 'syn.file='+str(r/'sources/xf_solver_lm_so101.cpp').replace('\\','/')+'\n'+
 'tb.file='+str(r/'sources/geometry_regression.cpp').replace('\\','/')+'\n'+
 ''.join(kind+'='+str(r/('sources/xf_solver_lm_so101.cpp' if kind.startswith('syn') else 'sources/geometry_regression.cpp')).replace('\\','/')+','+' '.join('-I'+str(r/('third_party/Vitis_Libraries_2024.2/solver/'+level+'/include')).replace('\\','/') for level in ['L1','L2'])+'\n' for kind in ['syn.file_cflags','tb.file_cflags'])+
 'clock=8ns\npackage.output.format=ip_catalog\npackage.output.syn=false\ncosim.tool=xsim\ncosim.rtl=verilog\n')
base=[v/'bin/vitis-run.bat','--mode','hls']
tail=['--config',cfg,'--work_dir',r/'build/hls']
steps=[('check',[sys.executable,r/'scripts/check_parity.py']),('tools',[sys.executable,r/'scripts/check_tools.py']),('csim',base+['--csim']+tail),('synth',[v/'bin/v++.bat','-c','--mode','hls']+tail),('cosim',base+['--cosim']+tail),('package',base+['--package']+tail),('abi',[sys.executable,r/'scripts/check_abi.py']),('hardware',[vivado,'-mode','batch','-source',r/'scripts/build_hardware.tcl','-log',r/'logs/vivado.log','-journal',r/'logs/vivado.jou']),('bdcheck',[sys.executable,r/'scripts/check_bd.py']),('platform',[v/'bin/vitis.bat','-s',r/'scripts/build_platform.py']),('boot',[sys.executable,r/'scripts/build_boot.py'])]
names=[s[0] for s in steps]
if a.from_stage not in names or a.through not in names:p.error('Unknown stage: '+','.join(names))
report={'toolchain':c,'board_execution':'NOT_RUN','motion_performed':False,'stages':{}}
rp=r/'reports/build_status.json'
if rp.exists():report['stages']=json.loads(rp.read_text()).get('stages',{})
for name,args in steps[names.index(a.from_stage):names.index(a.through)+1]:
 print('START',name,flush=True)
 with (r/'logs'/(name+'.log')).open('w',encoding='utf-8') as log:
  result=subprocess.run([str(x) for x in args],cwd=r/'build',env=env,stdout=log,stderr=subprocess.STDOUT)
 report['stages'][name]={'exit_code':result.returncode,'log':'logs/'+name+'.log','utc':datetime.datetime.now(datetime.timezone.utc).isoformat()};rp.write_text(json.dumps(report,indent=2))
 print('END',name,result.returncode,flush=True)
 if result.returncode:raise SystemExit(result.returncode)
print('BUILD_STAGES_COMPLETE; board validation remains separate',flush=True)
