from pathlib import Path
import csv,ctypes as c,json,math,subprocess,collections,random,os,shutil
r=Path(__file__).resolve().parents[1];out=r/'build/bound_retry_tests';out.mkdir(parents=True,exist_ok=True)
gcc=shutil.which('gcc') or r'D:\Software\minGW\mingw64\bin\gcc.exe'
class Result(c.Structure):_fields_=[('code',c.c_int),('iterations',c.c_int),('q',c.c_float*5),('energy',c.c_double),('position_error',c.c_double),('rotation_error',c.c_double)]
F=c.c_float*5;E=c.c_float*6;D=c.c_double*3
CALL=c.CFUNCTYPE(c.c_int,c.c_void_p,c.POINTER(c.c_float),c.POINTER(c.c_float),c.c_float,c.POINTER(c.c_float))
libs=[]
for label,source in [('old',r/'reference/pre_bound_retry_20261031/firmware/ik_iteration.c'),('new',r/'firmware/ik_iteration.c')]:
 dll=out/(label+('.dll' if os.name=='nt' else '.so'));subprocess.run([gcc,'-shared',*(['-fPIC'] if os.name!='nt' else []),'-std=c99','-O2','-Wall','-Wextra','-Werror','-I'+str(r/'firmware'),str(source),str(r/'tests/position_roll_reference.c'),'-lm','-o',str(dll)],check=True)
 lib=c.CDLL(str(dll));fn=lib.ik_solve_position_roll if label=='old' else lib.ik_solve_position_roll_retry
 fn.argtypes=[D,c.c_float,F,c.c_int,c.c_double,c.c_float,CALL,c.c_void_p,c.POINTER(Result)]
 cb=CALL(('numerical_step',lib));libs.append((lib,fn,cb))
lo=[-1.27780599631,-2.02178667843,-1.5447186534,-1.69044682825,-2.94370913195];hi=[1.45267980613,1.58153419231,1.79475752183,1.81623325286,2.8486023231]
def solve(i,xyz,roll,seed):
 res=Result();libs[i][1](D(*xyz),roll,F(*seed),200,1e-8,.01,libs[i][2],None,c.byref(res))
 if not res.code:
  assert res.position_error<math.sqrt(2e-8) and res.rotation_error<1e-7
  for j in range(5):assert lo[j]-2e-7<=res.q[j]<=hi[j]+2e-7
 return res
counts=collections.Counter();improved=[];regressed=[];cases=[]
for row in csv.DictReader((r/'tests/position_roll_targets.csv').open()):cases.append(('historical',[float(row[n]) for n in ['x','y','z']],float(row['roll']),[float(row['seed'+str(j)]) for j in range(5)]))
rows=list(csv.DictReader((r/'tests/bound_retry_drive_reference.csv').open()))
for row in rows:
 if row['status'] not in ['CONVERGED','MAX_ITER','JOINT_LIMIT']:continue
 seed=[sign*(float(row['actual_count_id'+str(j+1)])-zero)/651.8986469044033 for j,sign,zero in [(0,-1,2121),(1,1,2156),(2,1,1893),(3,1,1986),(4,-1,2037)]]
 cases.append(('latest_log_approx_seed',[float(row[n]) for n in ['target_x','target_y','target_z']],float(row['target_roll']),seed))
# Deterministic reachable tasks with varied seeds, including boundaries.
random.seed(731)
for n in range(250):
 target=[random.uniform(lo[j]+.03,hi[j]-.03) for j in range(5)];T=(c.c_double*16)();libs[0][0].ik_fk(F(*target),T)
 seed=[min(hi[j],max(lo[j],target[j]+random.uniform(-.8,.8))) for j in range(5)]
 if n%3==0:seed[n%4]=lo[n%4] if n%2 else hi[n%4]
 cases.append(('reachable_synthetic',[T[3],T[7],T[11]],target[4],seed))
for index,(group,xyz,roll,seed) in enumerate(cases):
 a=solve(0,xyz,roll,seed);b=solve(1,xyz,roll,seed);counts[group,a.code,b.code]+=1
 item={'case':index,'group':group,'xyz':xyz,'roll':roll,'seed':seed,'old_code':a.code,'new_code':b.code,'new_error':b.position_error,'new_iterations':b.iterations}
 if a.code and not b.code:improved.append(item)
 if not a.code and b.code:regressed.append(item)
# Hardware failure is terminal; no retries. Invalid ID5 uses no hardware call.
calls=[]
@CALL
def fault(ctx,q,e,lam,delta):calls.append(1);return 1
res=Result();libs[1][1](D(.2,.1,.2),0,F(0,0,0,0,0),200,1e-8,.01,fault,None,c.byref(res));assert res.code==2 and len(calls)==1
calls.clear();libs[1][1](D(.2,.1,.2),4,F(0,0,0,0,0),200,1e-8,.01,fault,None,c.byref(res));assert res.code==4 and not calls
# Stationary residual forces bounded numerical retries without changing q.
@CALL
def stationary(ctx,q,e,lam,delta):
 calls.append(1)
 for j in range(5):delta[j]=0
 return 0
libs[1][1](D(3,0,0),0,F(0,0,0,0,0),200,1e-8,.01,stationary,None,c.byref(res));assert res.code==1 and len(calls)<=600
report={'backend':'native finite-difference Jacobian reference; not board execution','cases':len(cases),'transitions':[{'group':g,'old_code':a,'new_code':b,'count':n} for (g,a,b),n in sorted(counts.items())],'improved':improved,'regressed':regressed,'guard_tests':'PASS hardware abort, invalid roll no calls, bounded failed retries','motion_performed':False}
(r/'reports/bound_retry_native.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ['improved','regressed']},indent=2));print('IMPROVED',len(improved),'REGRESSED',len(regressed))
if regressed:raise SystemExit(1)
