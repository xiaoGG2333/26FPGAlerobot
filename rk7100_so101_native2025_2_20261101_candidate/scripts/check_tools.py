from pathlib import Path
import json,subprocess
r=Path(__file__).resolve().parents[1];c=json.loads((r/'config/toolchain.json').read_text());amd=Path(c['amd_root'])
result={}
for name,exe,args in [('vivado',amd/'Vivado/bin/vivado.bat',['-version']),('hls',amd/'Vitis/bin/v++.bat',['--version']),('vitis',amd/'Vitis/bin/vitis.bat',['-v'])]:
 p=subprocess.run([str(exe),*args],cwd=r/'build',capture_output=True,text=True);text=p.stdout+p.stderr
 result[name]={'path':str(exe),'version_output':text,'exit_code':p.returncode}
 if '2025.2' not in text:raise RuntimeError(name+' version check failed: '+text)
(r/'reports/tool_versions.json').write_text(json.dumps(result,indent=2));print('PASS all tool entries report 2025.2')
