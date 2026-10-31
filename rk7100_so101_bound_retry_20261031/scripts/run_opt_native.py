from pathlib import Path
import subprocess,os,json
r=Path(__file__).resolve().parents[1];gcc='D:/Software/minGW/mingw64/bin/gcc.exe'
out=r/'build/opt_tests';out.mkdir(parents=True,exist_ok=True)
checks=[]
for name,files in [('curve',['test_curve.c']),('control',['test_opt_control.c']),('chip',['test_opt_chip.c']),('adaptive',['test_adaptive.c'])]:
 cmd=[gcc,'-std=c99','-O2','-Wall','-Wextra','-Werror','-I'+str(r/'firmware'),*[str(r/'tests'/x) for x in files]]
 if name=='control':cmd += [str(r/'firmware'/x) for x in ['arm_control.c','st3215.c','motor_control.c']]
 if name=='adaptive':cmd += [str(r/'firmware/ik_iteration.c')]
 exe=out/(name+'.exe');cmd+=['-lm','-o',str(exe)]
 subprocess.run(cmd,check=True);result=subprocess.run([str(exe)],capture_output=True,text=True)
 print(result.stdout+result.stderr);checks.append({'name':name,'code':result.returncode,'output':result.stdout+result.stderr});result.check_returncode()
(r/'reports/opt_native.json').write_text(json.dumps(checks,indent=2))
