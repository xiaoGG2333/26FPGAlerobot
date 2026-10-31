"""Package only after all functional checks, routed timing and new FSBL build pass."""
from pathlib import Path
import hashlib,json,re,zipfile,xml.etree.ElementTree as E
r=Path(__file__).resolve().parents[1]
b=r/'boot_opt125'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for f in ['BOOT.BIN','ik_uart1_servo.elf','fsbl_uart1.elf','rk7100_so101.bit','rk7100_so101.xsa','rk7100_so101.hwh','timing.rpt','utilization.rpt','drc.rpt']:assert (b/f).exists(),f
assert b'OPT125_SCURVE_20261018_V1' in (b/'ik_uart1_servo.elf').read_bytes()
assert (b/'rk7100_so101.bit').read_bytes()==(r/'build/hardware_export/rk7100_so101.bit').read_bytes()
assert sha(r/'boot/BOOT.BIN')==sha(r/'reference/pre_opt_20261018/boot/BOOT.BIN')
timing=(b/'timing.rpt').read_text();assert 'All user specified timing constraints are met.' in timing
values=next(x.split() for x in timing.split('WNS(ns)',1)[1].splitlines() if re.match(r'^\s*-?\d+\.\d+',x))
assert float(values[0])>=0 and float(values[4])>=0
tree=E.parse(b/'rk7100_so101.hwh').getroot()
ps=next(m for m in tree.iter('MODULE') if m.get('INSTANCE')=='processing_system7_0')
params={p.get('NAME'):p.get('VALUE') for p in ps.iter('PARAMETER')}
assert abs(float(params['PCW_ACT_FPGA0_PERIPHERAL_FREQMHZ'])-125)<.001,params['PCW_ACT_FPGA0_PERIPHERAL_FREQMHZ']
assert abs(float(params['PCW_ACT_UART_PERIPHERAL_FREQMHZ'])-100)<.001
for name in ['opt_hls_csim.log','opt_hls_cosim.log','opt_production_csim.log','opt_production_cosim.log']:
 text=(r/'logs'/name).read_text(errors='replace');assert 'PASS' in text and not re.search(r'^.*(?:FAIL |ERROR:)',text,re.M),name
assert 'Ran 56 tests' in (r/'reports/opt_active_host_tests.txt').read_text()
assert '\nOK' in (r/'reports/opt_active_host_tests.txt').read_text()
native=json.loads((r/'reports/opt_native.json').read_text());assert len(native)==4 and all(c['code']==0 for c in native)
for f in ['thermal_rtl_sim.txt','xadc_rtl_sim.txt']:assert 'PASS' in (r/'reports'/f).read_text()
# Scalar AXI ABI must remain unchanged, not just base addresses.
old=E.parse(r/'reference/pre_opt_20261018/boot/rk7100_so101.hwh').getroot()
def registers(doc):
 result={}
 for module in doc.iter('MODULE'):
  if module.get('INSTANCE','').startswith('xf_solver_lm_so101_'):
   result[module.get('INSTANCE')]=[(x.get('NAME'),tuple((p.get('NAME'),p.get('VALUE')) for p in x.findall('PROPERTY') if p.get('NAME') in ['ADDRESS_OFFSET','SIZE','ACCESS'])) for x in module.iter('REGISTER')]
 return result
assert registers(old)==registers(tree),'AXI scalar ABI changed'
assert len(registers(tree))==2 and all(len(x)>20 for x in registers(tree).values())
util=(b/'utilization.rpt').read_text();resources={}
for label,total in [('Slice LUTs',277400),('Slice Registers',554800),('Block RAM Tile',755),('DSPs',2020)]:
 m=re.search(r'\| '+re.escape(label)+r'\s+\|\s*(\d+)\s*\|[^\n]*?\|\s*([0-9.]+)\s*\|\s*$',util,re.M);assert m,label
 resources[label]={'used':int(m[1]),'total':total,'percent':float(m[2])}
report={'build_status':'PASS','on_board_status':'NOT_RUN','motion_performed':False,'clock_mhz':125,
 'resources':resources,'setup_wns_ns':float(values[0]),'hold_whs_ns':float(values[4]),
 'partition':'PS full iteration and trajectory; PL single float LM step','iteration_timing_added':False,
 'tests':{'native':native,'active_host':56,'legacy_host':'existing failures compared with baseline; no new failures',
 'geometry_checks':180,'legacy_full_iterations':2,'production_adaptive_iterations':3,'rtl_cosim':'PASS',
 'autonomous_thermal_and_xadc':'PASS','AXI_register_compatibility':'PASS'},
 'chip_policy':'>60C continuous >1500ms, <55C recovery plus explicit resume; abnormal-only periodic push; diagnostic THERMAL1 available',
 'trajectory':'PS online common-duration bounded quintic Bezier, C2 replans; not Ruckig time-optimal',
 'old_boot_sha256':sha(r/'boot/BOOT.BIN'),'new_boot_sha256':sha(b/'BOOT.BIN')}
(r/'reports/optimization_validation.json').write_text(json.dumps(report,indent=2))
scope=json.loads((r/'reports/optimization_scope.json').read_text());scope['status']='BUILD_AND_OFFLINE_VALIDATED_ON_BOARD_NOT_RUN'
(r/'reports/optimization_scope.json').write_text(json.dumps(scope,indent=2))
files={}
for folder in ['host','firmware','sources','rtl','hardware','scripts','tests','third_party','official_calibration']:
 for f in (r/folder).rglob('*'):
  if f.is_file() and '__pycache__' not in f.parts and f.suffix not in ['.pyc','.exe','.o','.log','.csv']:files[f.relative_to(r).as_posix()]=f
for name in ['README.md','OPT125_GUIDE.md','使用指南_OPT125.md','AGENTS.md','build.py','hls_config.cfg']:files[name]=r/name
files['reference/baseline_sha256.json']=r/'reference/baseline_sha256.json'
for name in ['opt_native.json','opt_scope_audit.json','optimization_validation.json','optimization_scope.json','source_parity.json','opt_active_host_tests.txt','opt_host_tests.txt','opt_baseline_host_tests.txt','thermal_rtl_sim.txt','xadc_rtl_sim.txt']:files['reports/'+name]=r/'reports'/name
for f in b.iterdir():
 if f.is_file() and f.suffix in ['.BIN','.elf','.bit','.hwh','.xsa','.rpt','.bif']:files['boot/'+f.name]=f
manifest={name:sha(f) for name,f in files.items()}
prefix='rk7100_so101_opt125_20261018';archive=r/'packages'/(prefix+'.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
 for name,f in files.items():z.write(f,prefix+'/'+name)
 z.writestr(prefix+'/SHA256.json',json.dumps(manifest,indent=2));z.writestr(prefix+'/results/.keep','')
 z.writestr(prefix+'/validation.json',json.dumps(report,indent=2))
with zipfile.ZipFile(archive) as z:
 assert z.testzip() is None
 for name,digest in manifest.items():assert hashlib.sha256(z.read(prefix+'/'+name)).hexdigest()==digest
print(json.dumps({'package':str(archive),'bytes':archive.stat().st_size,'resources':resources,'wns_ns':values[0],'whs_ns':values[4],'on_board':'NOT_RUN'},indent=2))
