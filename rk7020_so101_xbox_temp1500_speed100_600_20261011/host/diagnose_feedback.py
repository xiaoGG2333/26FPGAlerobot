"""Stationary feedback diagnosis through CAL1. Never enables torque or motion.

BEGIN takes an EEPROM snapshot. STOP confirms zero speed/torque and restores
that snapshot; this is a no-motion check, not a completely read-only session.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
from auto_calibrate_zynq import Bridge, Calibrator, CalibrationError, signed

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',default='COM16')
    parser.add_argument('--samples',type=int,default=20)
    parser.add_argument('--out',type=Path)
    parser.add_argument('--baseline',type=Path,help='Optional previous before.json for exact EEPROM comparison')
    args=parser.parse_args()
    if not 1<=args.samples<=100:parser.error('samples must be 1..100')
    folder=args.out or Path(__file__).resolve().parents[1]/'results'/('static_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    if folder.exists():parser.error('Output folder exists; preserve previous records')
    folder.mkdir(parents=True)
    bridge=None
    report={'status':'NOT_COMPLETED','motion_commands_sent':False,'torque_enable_commands_sent':False,
            'samples':[],'note':'Static success does not prove feedback reliability during motion'}
    try:
        bridge=Bridge(args.port,folder);bridge.start()
        before=Calibrator(bridge,folder).backup()
        if args.baseline:
            original={r['id']:r for r in json.loads(args.baseline.read_text(encoding='utf8'))['motors']}
            for motor in before['motors']:
                if motor['eeprom_hex']!=original[motor['id']]['eeprom_hex']:
                    raise CalibrationError(f"ID{motor['id']}: EEPROM differs from supplied baseline")
            report['baseline_eeprom_match']=True
        for sample in range(args.samples):
            for motor in range(1,7):
                temperature=bridge.read(motor,63,1)[0]
                voltage=bridge.read(motor,62,1)[0]
                status=bridge.read(motor,65,1)[0]
                pair=bridge.read(motor,62,2)
                block=bridge.read(motor,56,15)
                torque=bridge.read(motor,40,1)[0]
                row={'sample':sample,'id':motor,'temperature_single':temperature,
                     'voltage_single_raw':voltage,'status_single':status,
                     'temperature_pair':pair[1],'temperature_block':block[7],
                     'voltage_pair_raw':pair[0],'voltage_block_raw':block[6],
                     'status_block':block[9],'position':signed(block[:2],15),'torque':torque}
                report['samples'].append(row)
                if (torque!=0 or max(temperature,pair[1],block[7])>55
                    or any(not 90<=v<=126 for v in (voltage,pair[0],block[6]))
                    or status or block[9]):
                    raise CalibrationError('Unhealthy static feedback: '+str(row))
        report['cleanup']=bridge.finish(False)
        report['status']='STATIC_PASS'
        report['temperature_ranges']={str(motor):[
            min(v for row in report['samples'] if row['id']==motor
                for v in (row['temperature_single'],row['temperature_pair'],row['temperature_block'])),
            max(v for row in report['samples'] if row['id']==motor
                for v in (row['temperature_single'],row['temperature_pair'],row['temperature_block']))]
            for motor in range(1,7)}
        print('STATIC PASS; all six torque off; '+str(report['temperature_ranges']),flush=True)
    except BaseException as error:
        report['error']=repr(error)
        if bridge:
            try:report['cleanup']=bridge.finish(False)
            except Exception as cleanup_error:report['cleanup_error']=repr(cleanup_error)
        raise
    finally:
        if bridge:
            try:report['bus_trace']=bridge.capture_trace(folder)
            except Exception as error:report['bus_trace_error']=repr(error)
            try:bridge.close()
            except Exception as error:report['close_error']=repr(error)
        (folder/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf8')
        print('Recorded '+str(folder),flush=True)

if __name__=='__main__':
    try:main()
    except (CalibrationError,OSError,KeyboardInterrupt,ValueError) as error:raise SystemExit(str(error))
