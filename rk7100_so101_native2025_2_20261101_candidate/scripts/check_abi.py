from pathlib import Path
import re,json
r=Path(__file__).resolve().parents[1]
base=json.loads((r/'manifests/registers_baseline.json').read_text())
p=r/'build/hls/hls/impl/misc/drivers/xf_solver_lm_so101_v1_0/src/xxf_solver_lm_so101_hw.h'
actual=dict(re.findall(r'#define\s+(\w+)\s+(0x[0-9a-fA-F]+|[0-9]+)',p.read_text()))
report={'status':'PASS' if base==actual else 'FAIL','baseline':base,'migrated':actual}
(r/'reports/register_abi.json').write_text(json.dumps(report,indent=2))
print(report['status'],len(actual),'register definitions')
if base!=actual:raise SystemExit(1)
