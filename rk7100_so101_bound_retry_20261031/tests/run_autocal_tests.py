"""Run only new calibration/trace checks with simulated buses. No real hardware."""
from pathlib import Path
import json,subprocess,sys,tempfile

package=Path(__file__).resolve().parents[1]
gcc=Path(r'D:\Software\minGW\mingw64\bin\gcc.exe')
report={'hardware_used':False,'serial_port_opened':False,'status':'FAIL','checks':[]}
try:
    with tempfile.TemporaryDirectory(prefix='rk7020_autocal_tests_') as temp:
        for name,sources in (
            ('test_st3215',['st3215.c']),
            ('test_cal_bridge',['cal_bridge.c','st3215.c']),
            ('test_st_trace',['st3215.c'])):
            executable=Path(temp)/(name+'.exe')
            command=[str(gcc),'-std=c99','-O2','-Wall','-Wextra','-Werror',
                     str(package/'tests'/(name+'.c')),
                     *[str(package/'firmware'/s) for s in sources],'-o',str(executable)]
            built=subprocess.run(command,capture_output=True,text=True,timeout=60)
            if built.returncode:
                print(built.stdout+built.stderr);built.check_returncode()
            tested=subprocess.run([str(executable)],capture_output=True,text=True,timeout=60)
            report['checks'].append({'name':name,'exit_code':tested.returncode,'output':tested.stdout+tested.stderr})
            print(tested.stdout+tested.stderr,end='',flush=True);tested.check_returncode()
        for name in ('test_auto_calibrate_zynq.py','test_auto_calibrate_usb.py','test_temperature_debounce.py','test_reference_calibration_usb.py','test_hold_current_usb.py','test_encoder_reference_return.py','test_retry_usb_calibration.py','test_usb_feedback_diagnostics.py','test_vendor_calibration_math.py',
                     'test_trace_host.py','test_sync_autocal_baseline.py'):
            tested=subprocess.run([sys.executable,str(package/'tests'/name)],capture_output=True,text=True,timeout=60)
            report['checks'].append({'name':name,'exit_code':tested.returncode,'output':tested.stdout+tested.stderr})
            print(tested.stdout+tested.stderr,end='',flush=True);tested.check_returncode()
    report['status']='PASS'
finally:
    (package/'tests/autocal_unit_results.json').write_text(json.dumps(report,indent=2),encoding='utf8')
