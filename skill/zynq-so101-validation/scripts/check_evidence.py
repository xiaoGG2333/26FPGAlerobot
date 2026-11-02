"""Read-only file provenance check. MATCH never means functional acceptance."""
import argparse
import hashlib
import json
from pathlib import Path


def check(root, index):
    root = Path(root).resolve()
    findings = []
    for entry in index['files']:
        rel = entry['path']
        path = (root / rel).resolve()
        item = {'path': rel}
        if Path(rel).is_absolute() or not path.is_relative_to(root):
            item['status'] = 'INVALID_PATH'
        elif not path.is_file():
            item['status'] = 'MISSING'
        else:
            data = path.read_bytes()
            item['actual_sha256'] = hashlib.sha256(data).hexdigest()
            item['status'] = 'MATCH' if item['actual_sha256'] == entry['sha256'] else 'CHANGED'
            if path.suffix.lower() == '.json':
                try:
                    report = json.loads(data)
                    if isinstance(report, dict):
                        item['reported_validation'] = {
                            key: report[key] for key in
                            ('status', 'board_execution', 'motion_performed', 'formal_baseline_switched')
                            if key in report
                        }
                except (ValueError, UnicodeError):
                    item['status'] = 'INVALID_JSON'
        findings.append(item)
    return {
        'file_integrity': 'MATCH' if findings and all(x['status'] == 'MATCH' for x in findings) else 'REVIEW_REQUIRED',
        'functional_acceptance': 'NOT_ASSESSED',
        'hardware_accessed': False,
        'findings': findings,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', required=True)
    parser.add_argument('--index', type=Path, default=Path(__file__).resolve().parents[1] / 'references/evidence_index.json')
    args = parser.parse_args()
    try:
        result = check(args.project_root, json.loads(args.index.read_text(encoding='utf-8-sig')))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(json.dumps({'file_integrity': 'ERROR', 'message': str(exc)}, ensure_ascii=False))
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['file_integrity'] == 'MATCH' else 1


if __name__ == '__main__':
    raise SystemExit(main())
