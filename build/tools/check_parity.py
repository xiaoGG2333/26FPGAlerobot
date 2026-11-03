from pathlib import Path
import hashlib,json
r=Path(__file__).resolve().parents[1]
manifest=json.loads((r/'manifests/baseline_runtime_sha256.json').read_text())
compat=json.loads((r/'manifests/migration_compatibility_sha256.json').read_text())
manifest.update({p:v['sha256'] for p,v in compat.items()})
bad=[p for p,h in manifest.items() if not (r/p).is_file() or hashlib.sha256((r/p).read_bytes()).hexdigest()!=h]
report={'status':'FAIL' if bad else 'PASS','protected_files':len(manifest),'changed':bad,'documented_compatibility_adaptations':list(compat)}
(r/'reports/runtime_parity.json').write_text(json.dumps(report,indent=2))
print(report)
if bad:raise SystemExit(1)
