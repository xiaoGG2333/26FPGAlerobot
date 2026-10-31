"""Offline replay through independent finite-difference PL task reference. No UART."""
from pathlib import Path
import csv,ctypes as c,json,math,os,shutil,subprocess,collections
r=Path(__file__).resolve().parents[1]
out=r/'build/position_roll_tests';out.mkdir(parents=True,exist_ok=True)
gcc=shutil.which('gcc') or 'D:/Software/minGW/mingw64/bin/gcc.exe'
libpath=out/('position_roll.dll' if os.name=='nt' else 'position_roll.so')
cmd=[gcc,'-shared','-std=c99','-O2','-Wall','-Wextra','-Werror','-I'+str(r/'firmware'),str(r/'firmware/ik_iteration.c'),str(r/'tests/position_roll_reference.c'),'-lm','-o',str(libpath)]
if os.name!='nt':cmd.insert(2,'-fPIC')
subprocess.run(cmd,check=True)
lib=c.CDLL(str(libpath));F=c.c_float*5;D=c.c_double*3
class Result(c.Structure):
    _fields_=[('code',c.c_int),('iterations',c.c_int),('q',F),('energy',c.c_double),('position_error',c.c_double),('rotation_error',c.c_double)]
f=lib.ik_solve_position_roll
f.argtypes=[D,c.c_float,F,c.c_int,c.c_double,c.c_float,c.c_void_p,c.c_void_p,c.POINTER(Result)]
lo=[-1.27780599631,-2.02178667843,-1.5447186534,-1.69044682825,-2.94370913195]
hi=[1.45267980613,1.58153419231,1.79475752183,1.81623325286,2.8486023231]
counts=collections.Counter();iterations=[]
for row in csv.DictReader((r/'tests/position_roll_targets.csv').open()):
    res=Result();seed=F(*[float(row['seed'+str(i)]) for i in range(5)])
    rc=f(D(*[float(row[k]) for k in ['x','y','z']]),float(row['roll']),seed,200,1e-8,.01,c.cast(lib.numerical_step,c.c_void_p),None,c.byref(res))
    counts[(row['old_status'],rc)]+=1
    if row['old_status']=='CONVERGED':assert rc==0,row
    if rc==0:
        assert res.position_error<math.sqrt(2e-8) and res.rotation_error<1e-7
        for i in range(5):assert lo[i]-1e-7<=res.q[i]<=hi[i]+1e-7
        assert abs(res.q[4]-float(row['roll']))<2e-7
        iterations.append(res.iterations)
report={'status':'PASS','backend':'independent finite-difference task Jacobian; not physical hardware',
        'cases':[{'old_status':a,'new_code':b,'count':n} for (a,b),n in sorted(counts.items())],
        'success_iterations_min':min(iterations),'success_iterations_max':max(iterations),
        'motion_performed':False}
(r/'reports/position_roll_log_regression.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
