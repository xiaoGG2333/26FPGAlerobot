import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from check_evidence import check


class EvidenceChecks(unittest.TestCase):
    def test_provenance_does_not_promote_board_acceptance(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'report.json'
            p.write_text(json.dumps({'status': 'PASS_BUILD', 'board_execution': 'NOT_RUN'}), encoding='utf-8')
            index = {'files': [{'path': p.name, 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}]}
            result = check(d, index)
            self.assertEqual(result['file_integrity'], 'MATCH')
            self.assertEqual(result['functional_acceptance'], 'NOT_ASSESSED')
            self.assertEqual(result['findings'][0]['reported_validation']['board_execution'], 'NOT_RUN')
            p.write_text('{}', encoding='utf-8')
            self.assertEqual(check(d, index)['findings'][0]['status'], 'CHANGED')
            p.write_text('{', encoding='utf-8')
            self.assertEqual(check(d, index)['findings'][0]['status'], 'INVALID_JSON')
            p.unlink()
            self.assertEqual(check(d, index)['findings'][0]['status'], 'MISSING')

    def test_path_escape_and_empty_index_fail(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(check(d, {'files': [{'path': '../outside', 'sha256': ''}]})['findings'][0]['status'], 'INVALID_PATH')
            self.assertEqual(check(d, {'files': []})['file_integrity'], 'REVIEW_REQUIRED')


if __name__ == '__main__':
    unittest.main()
