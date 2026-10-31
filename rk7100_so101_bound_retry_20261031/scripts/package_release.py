from pathlib import Path
import json,hashlib,re,zipfile,xml.etree.ElementTree as E
r=Path('D:/VivadoProjects/RK7020_So_101')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert (r/'boot/BOOT.BIN').stat().st_size>1000000
assert b'CHIP_THERMAL1_V1' in (r/'boot/ik_uart1_servo.elf').read_bytes()
assert b'BOARD_RK7100_F_V1' in (r/'boot/ik_uart1_servo.elf').read_bytes()
assert (r/'boot/rk7100_so101.bit').read_bytes()==(r/'build/hardware_export/rk7100_so101.bit').read_bytes()
assert 'xc7z100ffg900-2' in (r/'build/rk7100_so101.xpr').read_text()
for n in ['hls_csim.log','hls_cosim.log']:
 s=(r/'logs'/n).read_text(errors='replace');assert 'FAIL' not in s;assert 'PASS' in s
host=(r/'reports/thermal_host_regression.txt').read_text();assert 'Ran 42 tests' in host and '\nOK' in host
for f in ['thermal_rtl_sim.txt','xadc_rtl_sim.txt']:
 content=(r/'reports'/f).read_text();assert 'PASS' in content and 'FAIL' not in content
root=E.parse(r/'boot/rk7100_so101.hwh').getroot();ps=next(m for m in root.iter('MODULE') if m.get('INSTANCE')=='processing_system7_0');params={v.get('NAME'):v.get('VALUE') for v in ps.iter('PARAMETER')}
assert params['PCW_ACT_FPGA0_PERIPHERAL_FREQMHZ']=='100.000000'
assert params['PCW_ACT_UART_PERIPHERAL_FREQMHZ']=='100.000000'
assert params['PCW_UART0_UART0_IO']=='MIO 14 .. 15' and params['PCW_UART1_UART1_IO']=='EMIO'
xp=(r/'build/vitis_workspace/platform/export/platform/sw/standalone_ps7_cortexa9_0/include/xparameters.h').read_text()
assert re.search(r'XPAR_XF_SOLVER_LM_SO101_0_BASEADDR\s+0x40000000',xp,re.I)
assert re.search(r'XPAR_XF_SOLVER_LM_SO101_1_BASEADDR\s+0x40010000',xp,re.I)
timing=(r/'boot/timing.rpt').read_text();assert 'All user specified timing constraints are met.' in timing
timing_values=next(line.split() for line in timing.split('WNS(ns)',1)[1].splitlines() if re.match(r'^\s*-?\d+\.\d+',line))
util=(r/'boot/utilization.rpt').read_text();resources={}
for label,total in [('Slice LUTs',277400),('Slice Registers',554800),('Block RAM Tile',755),('DSPs',2020)]:
 m=re.search(r'\| '+re.escape(label)+r'\s+\|\s*(\d+)\s*\|[^\n]*?\|\s*([0-9.]+)\s*\|\s*$',util,re.M);assert m,label
 resources[label]={'used':int(m[1]),'total':total,'percent':float(m[2])}
report={'build_status':'PASS','on_board_status':'NOT_RUN','hardware_programmed':False,'servo_motion_performed':False,'part':'xc7z100ffg900-2','ip_cores':2,'clock_mhz':100,'core_addresses':['0x40000000','0x40010000'],'debug_uart':{'unit':0,'rx_mio':14,'tx_mio':15,'baud':115200},'servo_uart':{'unit':1,'mode':'EMIO','tx_header':'J8.3','tx_package':'AK16','rx_header':'J8.4','rx_package':'AJ16','baud':1000000},'resources':resources,'timing':{'setup_wns_ns':float(timing_values[0]),'hold_whs_ns':float(timing_values[4]),'tns_ns':float(timing_values[1]),'timing_met':True},'source_parity':json.loads((r/'reports/source_parity.json').read_text()),'hls_csim':'PASS: 180 geometry checks + two full iterations','hls_rtl_cosim':'PASS: two full iterations, matching C results','host_tests':42,'native_encoder_roundtrips':12363,'native_control_fault_temperature_speed_tests':'PASS','ip_register_parity':'PASS: 36 registers','ddr_initialization_parity':json.loads((r/'reports/ddr_initialization_parity.json').read_text()),'bsp':'new Vitis 2024.2 platform from RK7100 XSA','fsbl':'new RK7100 FSBL','bootgen':'PASS','rebuild_script_tested_stages':['Platform','Boot'],'drc':'no Error or Critical Warning entries; pipeline/packing warnings and DSP advisories retained in drc.rpt','boot_sha256':sha(r/'boot/BOOT.BIN')}
report['chip_thermal']={'status':'PASS_SIMULATION_AND_BUILD','normal_pwm_pct':50,'pwm_hz':25000,'full_speed_at_c':65,'return_to_50_below_c':60,'freeze_ik_at_c':75,'manual_resume_below_c':65,'sensor_stale_timeout_s':1,'sensor_fault':'full speed + freeze IK','fan_pwm_pin':'W21','fan_tach_pin':'Y21','gpio_base':'0x41200000','tach_pulses_per_rev':2,'autonomous_pl':True,'rtl_tests':'PASS: PWM + thresholds + stale + OT + tach','xadc_primitive_simulation':'PASS: 45/70/80/55C actual primitive + DRP + averaging','native_torque_preservation':'PASS: freeze retains last goal/torque; explicit resume only','physical_fan_rpm_and_temperature':'NOT_RUN'}
report['timing']['asynchronous_fan_io']='explicit false paths: tach input synchronized; PWM output has no sampling clock; no_input/no_output_delay entries for these ports remain documented'
(r/'reports/validation.json').write_text(json.dumps(report,indent=2),encoding='utf8')
# All source/deployment material remains under the requested project directory.
(r/'packages').mkdir(exist_ok=True)
files=[]
for sub in ['firmware','host','tests','sources','hardware','scripts','third_party','mapping_review','official_calibration','reports','reference','rtl']:
 for f in (r/sub).rglob('*'):
  if f.is_file() and 'pre_fan_20261016' not in f.parts and '__pycache__' not in f.parts and f.suffix not in ['.exe','.pyc','.o']:files.append(f)
for name in ['README.md','AGENTS.md','build.py','hls_config.cfg']:files.append(r/name)
for name in ['BOOT.BIN','boot.bif','fsbl_uart1.elf','ik_uart1_servo.elf','rk7100_so101.bit','rk7100_so101.hwh','rk7100_so101.xsa','timing.rpt','utilization.rpt','drc.rpt']:files.append(r/'boot'/name)
(r/'SHA256.json').write_text(json.dumps({f.relative_to(r).as_posix():sha(f) for f in files},indent=2));files.append(r/'SHA256.json')
zip_path=r/'packages/rk7100_so101_thermal_20261016.zip'
with zipfile.ZipFile(zip_path,'w',zipfile.ZIP_DEFLATED) as z:
 for f in files:z.write(f,'rk7100_so101_thermal_20261016/'+f.relative_to(r).as_posix())
 z.writestr('rk7100_so101_thermal_20261016/results/.keep','')
with zipfile.ZipFile(zip_path) as z:
 assert z.testzip() is None
 manifest=json.loads(z.read('rk7100_so101_thermal_20261016/SHA256.json'))
 for name,digest in manifest.items():assert hashlib.sha256(z.read('rk7100_so101_thermal_20261016/'+name)).hexdigest()==digest,name
print('PACKAGE',zip_path,zip_path.stat().st_size)
print('RESOURCES',resources)
print('BUILD PASS; ON-BOARD NOT RUN; NO MOTION')
