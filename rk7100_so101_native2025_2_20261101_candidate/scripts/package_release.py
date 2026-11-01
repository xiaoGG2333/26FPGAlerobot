"""Package a validated build candidate; never programs a board."""
from pathlib import Path
import json,hashlib,zipfile,datetime,shutil
r=Path(__file__).resolve().parents[1]
status=json.loads((r/'reports/build_status.json').read_text())
required=['check','tools','csim','synth','cosim','package','abi','hardware','bdcheck','platform','boot']
bad=[n for n in required if status['stages'].get(n,{}).get('exit_code')!=0]
if bad:raise SystemExit('Build stages not passed: '+','.join(bad))
for name in ['runtime_parity','hardware_parity','register_abi','software_validation']:
 if json.loads((r/'reports'/(name+'.json')).read_text())['status']!='PASS':raise SystemExit('Validation failed: '+name)
if not (r/'board/boot/BOOT.BIN').is_file():raise SystemExit('BOOT.BIN missing')
for name in ['xf_solver_lm_so101_csynth.rpt','xf_solver_lm_so101_cosim.rpt']:
 found=list((r/'build/hls/hls').rglob(name))
 if not found:raise SystemExit('Report missing: '+name)
 shutil.copy2(found[0],r/'reports'/name)
folders=['firmware','sources','host','rtl','hardware','scripts','config','tests','third_party','reference','manifests','reports','board']
files=[]
for folder in folders:
 files.extend(p for p in (r/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ['.pyc','.exe','.dll','.so'])
files.extend(r/n for n in ['README.md','migration_guide.md','AGENTS.md'])
manifest={str(p.relative_to(r)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.name!='release_sha256.json'}
mf=r/'manifests/release_sha256.json';mf.write_text(json.dumps(manifest,indent=2));files=[p for p in files if p!=mf]+[mf]
out=r/'releases';out.mkdir(exist_ok=True)
name='rk7100_so101_native2025_2_20261101_candidate';archive=out/(name+'.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in sorted(set(files)):z.write(p,name+'/'+str(p.relative_to(r)).replace('\\','/'))
with zipfile.ZipFile(archive) as z:
 if z.testzip() is not None:raise RuntimeError('ZIP verification failed')
info={'file':archive.name,'bytes':archive.stat().st_size,'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'board_execution':'NOT_RUN','motion_performed':False,'status':'BUILD_VALIDATED_CANDIDATE'}
(out/'package_manifest.json').write_text(json.dumps(info,indent=2));print(json.dumps(info,indent=2))
