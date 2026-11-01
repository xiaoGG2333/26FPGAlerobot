"""Optional sideband telemetry; RESULT1/STATE1 formats remain unchanged."""
import math
THERMAL_FIELDS=['chip_temp_c','fan_pwm_pct','fan_rpm','chip_thermal_state','chip_sensor_ok']
def parse_thermal(line):
    parts=line.split()
    if len(parts)!=7 or parts[0]!='THERMAL1':raise ValueError('Invalid THERMAL1 frame')
    seq=int(parts[1]);d=dict(p.split('=',1) for p in parts[2:])
    temp=float(d['chip_c']);fan=int(d['fan_pct']);rpm=int(d['rpm']);ok=int(d['sensor_ok'])
    if seq<0 or fan not in (50,100) or not 0<=rpm<=65535 or ok not in (0,1):raise ValueError('Invalid thermal fields')
    if d['state'] not in ('NORMAL','FULL','HOT_HOLD','SENSOR_FAULT'):raise ValueError('Invalid thermal state')
    if ok and (not math.isfinite(temp) or not -20<=temp<=120):raise ValueError('Invalid chip temperature')
    if not ok and (fan!=100 or d['state']!='SENSOR_FAULT'):raise ValueError('Invalid sensor fault policy')
    if d['state']=='NORMAL' and (not ok or fan!=50):raise ValueError('Invalid normal thermal state')
    if d['state'] in ('FULL','HOT_HOLD') and (not ok or fan!=100):raise ValueError('Invalid full-speed thermal state')
    return dict(seq=seq,chip_temp_c=temp,fan_pwm_pct=fan,fan_rpm=rpm,chip_thermal_state=d['state'],chip_sensor_ok=ok)
def consume_thermal(uart,line):
    if not line.startswith('THERMAL1 '):return False
    try:d=parse_thermal(line)
    except (ValueError,KeyError):
        print('CHIP TELEMETRY INVALID:',line,flush=True);return True
    uart.thermal=d
    print('CHIP: '+('%.2f C'%d['chip_temp_c'] if d['chip_sensor_ok'] else 'SENSOR FAULT')+
          ' | fan_pwm=%d%% rpm=%d | %s'%(d['fan_pwm_pct'],d['fan_rpm'],d['chip_thermal_state']),flush=True)
    return True
def thermal_values(uart):
    d=getattr(uart,'thermal',{}) if uart is not None else {}
    return [d.get(k,'') for k in THERMAL_FIELDS]
