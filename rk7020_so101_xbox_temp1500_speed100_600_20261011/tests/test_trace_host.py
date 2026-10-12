"""Diagnostic collection tests. No serial port or hardware is opened."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from auto_calibrate_zynq import Bridge,CalibrationError

class TraceTests(unittest.TestCase):
    def make_bridge(self,replies,*,active=False,supported=True):
        b=object.__new__(Bridge)
        b.active=active;b.trace_supported=supported;commands=[]
        def exchange(command):
            commands.append(command)
            return replies[command]
        b.exchange=exchange
        return b,commands

    def test_collects_raw_packets_only_after_stop(self):
        b,commands=self.make_bridge({
            'CAL1 TRACE COUNT':['CAL1 TRACE COUNT 1','CAL1 END'],
            'CAL1 TRACE 0':['CAL1 TRACE 0 id=4 op=2 addr=62 count=2 rc=0 drain=0 truncated=0 tx=FFFF0404023E02B5 rx=FFFF040400791E60','CAL1 END']})
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);result=b.capture_trace(folder)
            saved=json.loads((folder/'bus_trace.json').read_text())
            self.assertEqual(result['transactions'],1)
            self.assertEqual(saved['records'][0]['rx'],'FFFF040400791E60')
        self.assertEqual(commands,['CAL1 TRACE COUNT','CAL1 TRACE 0'])

    def test_active_or_old_firmware_issues_no_diagnostic_commands(self):
        for active,supported in ((True,True),(False,False)):
            b,commands=self.make_bridge({},active=active,supported=supported)
            with tempfile.TemporaryDirectory() as temp:
                b.capture_trace(Path(temp))
                self.assertFalse((Path(temp)/'bus_trace.json').exists())
            self.assertEqual(commands,[])

    def test_bad_count_is_rejected_without_further_requests(self):
        b,commands=self.make_bridge({'CAL1 TRACE COUNT':['CAL1 TRACE COUNT 100']})
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(CalibrationError):b.capture_trace(Path(temp))
        self.assertEqual(commands,['CAL1 TRACE COUNT'])

    def test_bad_packet_does_not_create_successful_trace_file(self):
        b,_=self.make_bridge({'CAL1 TRACE COUNT':['CAL1 TRACE COUNT 1'],
                             'CAL1 TRACE 0':['CAL1 TRACE 0 tx=XX rx=FF']})
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            with self.assertRaises(ValueError):b.capture_trace(folder)
            self.assertFalse((folder/'bus_trace.json').exists())

if __name__=='__main__':unittest.main()
