from pathlib import Path
import json
r=Path(__file__).resolve().parents[1]
a=json.loads((r/'reference/baseline_bd.json').read_text())['design']
b=json.loads((r/'build/v/ik.srcs/sources_1/bd/design_1/design_1.bd').read_text())['design']
def nets(d):return {k:sorted(v.get('ports',v.get('interface_ports',[]))) for k,v in d.items()}
def interfaces(d):return {k:{p:v.get(p) for p in ['mode','vlnv','parameters']} for k,v in d.items()}
checks={'ps_configuration':a['components']['processing_system7_0']['parameters']==b['components']['processing_system7_0']['parameters'],'external_ports':a['ports']==b['ports'],'interface_parameters':interfaces(a['interface_ports'])==interfaces(b['interface_ports']),'signal_connections':nets(a['nets'])==nets(b['nets']),'bus_connections':nets(a['interface_nets'])==nets(b['interface_nets']),'address_map':a['addressing']==b['addressing']}
report={'status':'PASS' if all(checks.values()) else 'FAIL','checks':checks}
(r/'reports/hardware_parity.json').write_text(json.dumps(report,indent=2));print(report)
if not all(checks.values()):raise SystemExit(1)
