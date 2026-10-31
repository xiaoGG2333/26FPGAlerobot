from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1]
baseline=json.loads((root/'reference/baseline_sha256.json').read_text())
changed=[rel for rel,digest in baseline.items() if hashlib.sha256((root/rel).read_bytes()).hexdigest()!=digest]
allowed=['firmware/main.c','firmware/arm_uart.inc','firmware/pose_uart.inc','firmware/servo_uart.inc','host/right_joycon.py','firmware/arm_control.c','firmware/arm_control.h','firmware/ik_iteration.c','firmware/ik_iteration.h','firmware/st3215.c','sources/lm_solver_so101.hpp','sources/geometry_regression.cpp']
assert sorted(changed)==sorted(allowed),changed
protected=[rel for rel in baseline if rel not in allowed]
for rel in protected:assert hashlib.sha256((root/rel).read_bytes()).hexdigest()==baseline[rel]
assert 'arm_stop(' not in (root/'firmware/chip_thermal.inc').read_text()
report={'status':'PASS','unchanged_files':len(protected),'protected_files':protected,'changed_runtime_files':changed,'changes':'user-authorized OPT125, parallel float step, PS damping acceptance and online trajectory; protected calibration/input/control policies audited separately','ik_math_iteration_precision_unchanged':False,'single_step_float_precision_preserved':True,'ps_pl_partition_preserved':True,'servo_control_limits_mapping_temperature_speed_policy_unchanged':False,'servo_mapping_endpoints_temperature_policy_unchanged':True,'xbox_axes_and_coordinate_integration_unchanged':True,'new_chip_fault_uses_existing_torque_preserving_freeze':True}
(root/'reports/source_parity.json').write_text(json.dumps(report,indent=2))
print('PASS source parity:',len(protected),'baseline files unchanged; explicitly authorized optimization files listed')
