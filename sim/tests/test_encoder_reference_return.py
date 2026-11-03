"""Fault tests for saved-encoder return; no hardware or serial port opened."""
from pathlib import Path
import json
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parent))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from test_hold_current_usb import Bus, BASELINE
from auto_calibrate_zynq import CalibrationError
import encoder_reference_return_usb as er


class MovingBus(Bus):
    def __init__(self):
        super().__init__()
        self.stall=False

    def write(self,motor,address,data):
        try:
            super().write(motor,address,data)
        finally:
            if address==42 and not self.stall:
                self.memory[motor][56:58]=data


class DeadbandBus(MovingBus):
    def write(self,motor,address,data):
        previous=int.from_bytes(self.read(motor,56,2),'little')
        super().write(motor,address,data)
        if motor==2 and address==42:
            goal=int.from_bytes(data,'little')
            position=(goal-9 if previous<goal-9 else goal+9 if previous>goal+9 else previous)
            self.memory[motor][56:58]=position.to_bytes(2,'little')


def reference(bus):
    return dict(schema='so101_encoder_snapshot_v1',status='SNAPSHOT_PASS',
        eeprom_unchanged=True,six_axes_torque_off=True,position_mode=True,
        register_writes_sent=False,
        encoder_counts_median=[int.from_bytes(bus.read(m,56,2),'little') for m in range(1,7)],
        encoder_counts_spread=[0]*6,
        position_space='Present_Position_after_existing_homing_offset',
        eeprom_before_hex={str(m):bus.read(m,0,40).hex() for m in range(1,7)},
        eeprom_after_hex={str(m):bus.read(m,0,40).hex() for m in range(1,7)})


class Tests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.folder=Path(self.temp.name)
        self.bus=MovingBus()
        self.ref=reference(self.bus)
        self.target={m:self.ref['encoder_counts_median'][m-1]+(9 if m==2 else 0) for m in range(1,6)}
        self.controller=er.EncoderReturn(self.bus,self.folder,BASELINE,self.target,self.ref)

    def tearDown(self):
        self.temp.cleanup()

    def off(self):
        cleanup=self.controller.stop()
        self.assertTrue(cleanup['confirmed'],cleanup)
        self.assertTrue(all(self.bus.read(m,40,1)==b'\0' for m in range(1,7)))
        self.assertFalse(cleanup['old_goal_restored'])

    def test_ramp_returns_to_saved_counts_after_fresh_goals(self):
        self.controller.start()
        for _ in range(8):
            if self.controller.advance():
                break
        else:
            self.fail('Ramp did not settle')
        self.assertEqual(int.from_bytes(self.bus.read(2,56,2),'little'),self.target[2])
        self.assertTrue(all(self.bus.enable_checks[:5]))
        goals=[int.from_bytes(data,'little') for m,address,data in self.bus.writes if m==2 and address==42]
        self.assertTrue(all(0<=b-a<=3 for a,b in zip(goals,goals[1:])))
        self.assertTrue(all(m!=6 and address>=40 for m,address,_ in self.bus.writes))
        self.off()

    def test_excess_distance_rejected_before_any_write(self):
        self.controller.destination[2]+=193
        with self.assertRaises(CalibrationError):
            self.controller.start()
        self.assertEqual(self.bus.writes,[])
        self.off()

    def test_changed_saved_eeprom_rejected_before_any_write(self):
        self.controller.reference['eeprom_before_hex']['4']='00'*40
        with self.assertRaises(CalibrationError):
            self.controller.start()
        self.assertEqual(self.bus.writes,[])
        self.off()

    def test_lost_goal_ack_after_motion_still_closes_torque(self):
        self.controller.start()
        self.bus.failure=(2,42,(self.controller.targets[2]+3).to_bytes(2,'little'))
        with self.assertRaises(CalibrationError):
            self.controller.advance()
        self.off()

    def test_stalled_feedback_does_not_run_command_ramp_away(self):
        self.controller.start()
        self.bus.stall=True
        self.controller.destination[2]+=60
        for _ in range(10):
            self.assertFalse(self.controller.advance())
        original=self.ref['encoder_counts_median'][1]
        self.assertLessEqual(self.controller.targets[2]-original,15)
        self.off()

    def test_nine_count_deadband_finishes_without_claiming_exact_position(self):
        self.bus=DeadbandBus()
        self.ref=reference(self.bus)
        self.target={m:self.ref['encoder_counts_median'][m-1]+(30 if m==2 else 0) for m in range(1,6)}
        self.controller=er.EncoderReturn(self.bus,self.folder,BASELINE,self.target,self.ref)
        self.controller.start()
        for _ in range(20):
            if self.controller.advance():
                break
        else:
            self.fail('Deadband permanently blocked the ramp')
        actual=int.from_bytes(self.bus.read(2,56,2),'little')
        self.assertEqual(self.controller.targets[2],self.target[2])
        self.assertEqual(self.target[2]-actual,9)
        self.assertLessEqual(abs(actual-self.target[2]),er.POSITION_TOLERANCE)
        self.assertLessEqual(er.ADVANCE_TRACKING_TOLERANCE+er.STEP_COUNTS,16)
        self.off()

    def test_stop_file_aborts_ramp_and_cleanup_keeps_latest_goal(self):
        self.controller.start()
        (self.folder/'STOP').touch()
        with self.assertRaises(CalibrationError):
            self.controller.advance()
        self.bus.writes.clear()
        self.off()
        self.assertTrue(all(address!=42 for _,address,_ in self.bus.writes))

    def test_candidate_zero_table_cannot_be_used_as_pose(self):
        path=self.folder/'invalid.json'
        path.write_text(json.dumps(dict(schema='so101_operator_zero_candidate_v3',joints=[])))
        with self.assertRaises(CalibrationError):
            er.reference_targets(path,BASELINE)
        self.assertEqual(self.bus.writes,[])

    def test_offline_pose_loader_does_not_add_homing_offset(self):
        path=self.folder/'capture.json'
        path.write_text(json.dumps(self.ref))
        targets,_=er.reference_targets(path,BASELINE)
        self.assertEqual(targets,{m:self.ref['encoder_counts_median'][m-1] for m in range(1,6)})
        self.assertEqual(self.bus.writes,[])


if __name__=='__main__':
    unittest.main()
