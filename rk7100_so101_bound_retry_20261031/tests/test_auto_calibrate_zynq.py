"""No-hardware integration tests for the bounded CAL1 host calibration flow.

All devices, motion feedback, CAL1 cleanup and time are simulated. These tests
exercise the host's decisions; they do not verify firmware rollback, actual
mechanical endpoints or physical/HLS alignment.
"""

from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


HOST_DIR = Path(__file__).resolve().parents[1] / "host"
sys.path.insert(0, str(HOST_DIR))
SPEC = importlib.util.spec_from_file_location("tested_auto_calibrate_zynq", HOST_DIR / "auto_calibrate_zynq.py")
cal = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cal)


class SimClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def time(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class FakeBridge:
    """Position/velocity register model with hard physical stops and offsets."""

    def __init__(self, clock, *, block_positive_motor=None, timeout_motor=None,
                 communication_fail_after=None, reject_final_write=False,
                 corrupt_final_readback=False, close_error=False, recoil_motor=None, midpoint_jitter=False,
                 hot_direct_temp=False, corrupt_feedback_temp=False,
                 reject_acc_readback_mode=None):
        self.clock = clock
        self.block_positive_motor = block_positive_motor
        self.timeout_motor = timeout_motor
        self.communication_fail_after = communication_fail_after
        self.reject_final_write = reject_final_write
        self.corrupt_final_readback = corrupt_final_readback
        self.close_error = close_error
        self.recoil_motor = recoil_motor
        self.midpoint_jitter = midpoint_jitter;self.elbow_runs = 0;self.jitter_pending = False
        self.hot_direct_temp = hot_direct_temp
        self.corrupt_feedback_temp = corrupt_feedback_temp
        self.reject_acc_readback_mode = reject_acc_readback_mode
        self.feedback_count = 0
        self.history = []
        self.started = False
        self.closed = False
        self.active = False
        self.cleanup = []
        self.registers = {}
        self.positions = {}
        self.physical_limits = {}
        self.offsets = {1: 100, 2: -90, 3: 40, 4: -55, 5: 75, 6: -35}
        for motor in range(1, 7):
            raw = bytearray(96)
            raw[3:5] = (777).to_bytes(2, "little")
            raw[5] = motor
            low = 900 - self.offsets[motor]
            high = 3100 - self.offsets[motor]
            raw[9:11] = low.to_bytes(2, "little")
            raw[11:13] = high.to_bytes(2, "little")
            raw[16:18] = (700).to_bytes(2, "little")
            raw[31:33] = cal.encode_signed(self.offsets[motor], 11)
            raw[33] = 0
            raw[40] = 0
            raw[41] = 5
            raw[42:44] = (2000 - self.offsets[motor]).to_bytes(2, "little")
            raw[48:50] = (700).to_bytes(2, "little")
            raw[62] = 122
            raw[63] = 30
            self.registers[motor] = raw
            self.positions[motor] = 2000.0
            self.physical_limits[motor] = (900.0, 3100.0)
        self.baseline = {motor: bytes(raw) for motor, raw in self.registers.items()}

    def start(self):
        self.started = self.active = True
        self.history.append(("BEGIN",))

    def _feedback(self, motor):
        raw = self.registers[motor]
        offset = cal.signed(raw[31:33], 11)
        physical = int(round(self.positions[motor])) % 4096
        # Confirmed on the real STS3215: mode1 feedback is raw, even if its
        # EEPROM offset has not been cleared yet. Mode0 applies that offset.
        logical = physical if raw[33] == 1 else (physical - offset) % 4096
        velocity = cal.signed(raw[46:48], 15)
        if not raw[40] or raw[33] != 1:
            velocity = 0
        low, high = self.physical_limits[motor]
        if (velocity > 0 and self.positions[motor] >= high
                or velocity < 0 and self.positions[motor] <= low
                or velocity > 0 and self.block_positive_motor == motor):
            velocity = 0
        raw[56:58] = logical.to_bytes(2, "little")
        raw[58:60] = cal.encode_signed(velocity, 15)
        raw[60:62] = cal.encode_signed(10 if velocity else 0, 10)
        raw[65] = 0
        raw[66] = int(velocity != 0)
        raw[69:71] = (20 if velocity else 0).to_bytes(2, "little")

    def read(self, motor, address, size):
        if address == 56:
            self.feedback_count += 1
            if (self.communication_fail_after is not None
                    and self.feedback_count >= self.communication_fail_after):
                self.communication_fail_after = None
                raise cal.CalibrationError("simulated unconfirmed serial read")
        self._feedback(motor)
        result = bytes(self.registers[motor][address:address + size])
        raw = self.registers[motor]
        if self.hot_direct_temp and motor == 3 and address == 62 and size == 2 and raw[33] == 1 and cal.signed(raw[46:48],15):
            result = bytes([result[0],80])
        if self.corrupt_feedback_temp and address == 62 and size == 2:
            damaged = bytearray(result);damaged[1] = 150;result = bytes(damaged)
            self.history.append(("CORRUPT_TEMPERATURE", motor, address, size))
        if (address == 41 and size == 1 and self.reject_acc_readback_mode is not None
                and raw[33] == self.reject_acc_readback_mode
                and any(row[:2] == ("MODE_ACC_RESET", motor) for row in self.history)):
            result = b"\0"
        if self.corrupt_final_readback and address == 9 and size == 32:
            self.corrupt_final_readback = False
            damaged = bytearray(result)
            damaged[22] ^= 1
            result = bytes(damaged)
        self.history.append(("READ", motor, address, result))
        return result

    def write(self, motor, address, data):
        data = bytes(data)
        if motor == 3 and address == 46 and cal.signed(data, 15):
            self.elbow_runs += 1
            if self.elbow_runs == 5 and self.midpoint_jitter:self.jitter_pending = True
        if self.reject_final_write and address == 31 and data != b"\0\0":
            self.reject_final_write = False
            raise cal.CalibrationError("simulated final offset write failure")
        self.registers[motor][address:address + len(data)] = data
        self.history.append(("WRITE", motor, address, data))
        if address == 33:
            # Confirmed in the bounded USB probe: a MODE write resets ACC41.
            self.registers[motor][41] = 0
            self.history.append(("MODE_ACC_RESET", motor, data[0]))
        if (motor == self.recoil_motor and address == 40 and data == b"\0"
                and self.positions[motor] <= self.physical_limits[motor][0] + 5):
            self.positions[motor] += 250
        raw = self.registers[motor]
        if (address == 42 and raw[33] == 0 or address == 46 and raw[33] == 1):
            raw[40] = 1
            self.history.append(("AUTO_TORQUE", motor, address, 1))

    def byte(self, motor, address, value):
        self.write(motor, address, bytes([value]))

    def uint(self, motor, address, value):
        self.write(motor, address, int(value).to_bytes(2, "little"))

    def hold(self, motor):
        raw = self.registers[motor]
        assert raw[33] == 0 and raw[40] == 0 and raw[46:48] == b"\0\0"
        assert raw[41] == 5
        self.history.append(("HOLD", motor))
        self._feedback(motor)
        self.uint(motor, 42, int.from_bytes(raw[56:58], "little"))
        if raw[40] == 0:self.byte(motor, 40, 1)

    def keep(self):
        self.history.append(("KEEP",))

    def wait(self, duration):
        timeout_running = False
        for motor, raw in self.registers.items():
            velocity = cal.signed(raw[46:48], 15)
            if not raw[40] or raw[33] != 1 or not velocity:
                continue
            if velocity > 0 and self.block_positive_motor == motor:
                continue
            if motor == 3 and self.jitter_pending:
                self.positions[motor] -= 7;self.jitter_pending = False
                continue
            low, high = self.physical_limits[motor]
            if self.timeout_motor == motor:
                timeout_running = True
                # Encoder keeps moving, slowly enough not to trip travel guard.
                velocity = 25 if velocity > 0 else -25
            self.positions[motor] = min(high, max(low, self.positions[motor] + velocity * duration))
        self.clock.now += duration * (25 if timeout_running else 1)
        self.keep()

    def finish(self, commit):
        self.cleanup.append(commit)
        self.history.append(("COMMIT" if commit else "STOP",))
        for motor, raw in self.registers.items():
            if not commit:
                raw[:40] = self.baseline[motor][:40]
            raw[33] = 0
            raw[40] = 0
            raw[46:48] = b"\0\0"
        self.active = False
        action = "COMMIT" if commit else "STOP"
        return [f"CAL1 {action} off_confirmed=1 rollback_success=1", "CAL1 END"]

    def close(self):
        self.closed = True
        if self.close_error:
            raise OSError("simulated Bad file descriptor while closing serial")


class HostCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def run_main(self, bridge, expected_error=None, inspect=False):
        folder = self.root / "run"
        args = ["--port", "FAKE", "--out", str(folder)]
        if inspect:
            args.append("--inspect")
        with patch.object(cal, "time", bridge.clock), patch.object(cal, "Bridge", return_value=bridge), redirect_stdout(io.StringIO()):
            if expected_error is None:
                cal.main(args)
            else:
                with self.assertRaisesRegex(Exception, expected_error):
                    cal.main(args)
        return json.loads((folder / "summary.json").read_text()), folder

    def test_complete_six_axis_flow_and_offset_switching(self):
        bridge = FakeBridge(SimClock())
        summary, folder = self.run_main(bridge)
        self.assertEqual(summary["status"], "CALIBRATION_PASS")
        self.assertEqual(bridge.cleanup, [True])
        self.assertTrue(bridge.closed)
        results = json.loads((folder / "rk7020_right_so101.json").read_text())
        after = json.loads((folder / "after_write_readback.json").read_text())
        self.assertEqual(len(after), 6)
        self.assertTrue(all(row["mode"] == 0 and row["torque"] == 0 for row in after))
        self.assertEqual(set(results), set(cal.MOTOR_IDS))
        records = summary["endpoint_records"]
        first_names = [row["name"] for row in records if row["label"] == "first boundary"]
        self.assertEqual(first_names, list(cal.ORDER))
        self.assertEqual([(row["name"], row["direction"]) for row in records
                          if row["label"] == "vendor recovery"],
                         [("shoulder_lift", -1), ("elbow_flex", 1), ("wrist_flex", 1)])
        for motor in range(1, 7):
            first_goal = next(i for i, row in enumerate(bridge.history)
                              if row[:3] == ("WRITE", motor, 42))
            first_enable = next(i for i, row in enumerate(bridge.history)
                                if row[:2] == ("AUTO_TORQUE", motor)
                                or row[:3] == ("WRITE", motor, 40) and row[3] == b"\1")
            self.assertLess(first_goal, first_enable)
            for addr in (16, 48, 41):
                self.assertLess(next(i for i, row in enumerate(bridge.history)
                                     if row[:3] == ("WRITE", motor, addr)), first_goal)
        backup = json.loads((folder / "before.json").read_text())
        self.assertEqual([row["homing_offset"] for row in backup["motors"]], list(bridge.offsets.values()))
        for name, motor in cal.MOTOR_IDS.items():
            raw = bridge.registers[motor]
            result = results[name]
            self.assertEqual(cal.signed(raw[31:33], 11), result["homing_offset"])
            self.assertEqual(cal.word(raw[9:11]), result["range_min"])
            self.assertEqual(cal.word(raw[11:13]), result["range_max"])
            self.assertEqual(raw[33], 0)
            self.assertEqual(raw[40], 0)
            zero_writes = [row for row in bridge.history
                           if row[:3] == ("WRITE", motor, 31) and row[3] == b"\0\0"]
            self.assertEqual(len(zero_writes), 1)
        self.assertFalse(summary["physical_alignment_verified"])
        self.assertFalse(summary["geometry_verified"])

    def test_mode_reset_requires_acc_write_and_read_before_speed_or_hold(self):
        bridge = FakeBridge(SimClock())
        summary, _ = self.run_main(bridge)
        self.assertEqual(summary["status"], "CALIBRATION_PASS")
        checked = {0: 0, 1: 0}
        for index, row in enumerate(bridge.history):
            if row[0] != "MODE_ACC_RESET":
                continue
            _, motor, mode = row
            later = [event for event in bridge.history[index + 1:]
                     if len(event) > 1 and event[1] == motor]
            next_motion = next((j for j, event in enumerate(later)
                                if event[0] == "HOLD" or event[:3] == ("WRITE", motor, 46)), None)
            # Final disabled mode0 switch performs no further motion or HOLD.
            if next_motion is None:
                continue
            preceding = later[:next_motion]
            write_at = next(j for j, event in enumerate(preceding)
                            if event == ("WRITE", motor, 41, b"\x05"))
            read_at = next(j for j, event in enumerate(preceding)
                           if event == ("READ", motor, 41, b"\x05"))
            self.assertLess(write_at, read_at)
            checked[mode] += 1
        self.assertGreater(checked[0], 0)
        self.assertGreater(checked[1], 0)

    def test_acc_readback_mismatch_after_either_mode_stops_without_commit(self):
        for mode in (0, 1):
            with self.subTest(mode=mode):
                bridge = FakeBridge(SimClock(), reject_acc_readback_mode=mode)
                # run_main creates one preserved folder per invocation.
                self.root = self.root / str(mode)
                self.root.mkdir()
                summary, folder = self.run_main(bridge, "acceleration readback after mode")
                self.assertEqual(bridge.cleanup, [False])
                self.assertFalse(summary["commit_confirmed"])
                self.assertFalse((folder / "rk7020_right_so101.json").exists())
                self.assertTrue(all(raw[40] == 0 for raw in bridge.registers.values()))

    def test_short_one_way_stall_is_rejected_and_stopped(self):
        bridge = FakeBridge(SimClock(), block_positive_motor=3)
        summary, folder = self.run_main(bridge, "possible under-torque stall")
        self.assertEqual(summary["status"], "NOT_COMPLETED")
        self.assertEqual(bridge.cleanup, [False])
        self.assertFalse((folder / "rk7020_right_so101.json").exists())
        self.assertEqual(bridge.registers[3][:40], bridge.baseline[3][:40])
        self.assertTrue(all(raw[40] == 0 for raw in bridge.registers.values()))

    def test_recoil_after_unloading_uses_actual_next_start(self):
        bridge = FakeBridge(SimClock(), recoil_motor=3)
        summary, folder = self.run_main(bridge)
        self.assertEqual(summary['status'], 'CALIBRATION_PASS')
        repeats = [r for r in summary['endpoint_records']
                   if r['id'] == 3 and r['label'] == 'repeat first boundary']
        self.assertEqual(len(repeats), 1)
        self.assertEqual(repeats[0]['start_position'], 1150)
        self.assertEqual(repeats[0]['travel'], 1950)
        self.assertEqual(repeats[0]['expected_directional_travel'], 1950)
        self.assertTrue((folder/'rk7020_right_so101.json').exists())

    def test_midpoint_net_travel_accounts_for_small_initial_recoil(self):
        bridge = FakeBridge(SimClock(), recoil_motor=3, midpoint_jitter=True)
        summary, folder = self.run_main(bridge)
        self.assertEqual(summary['status'], 'CALIBRATION_PASS')
        self.assertTrue((folder/'rk7020_right_so101.json').exists())

    def test_direct_temperature_alarm_stops_even_if_position_block_is_cold(self):
        bridge = FakeBridge(SimClock(), hot_direct_temp=True)
        summary, folder = self.run_main(bridge,'unhealthy feedback')
        self.assertEqual(bridge.cleanup,[False])
        self.assertFalse(summary['commit_confirmed'])
        self.assertFalse((folder/'rk7020_right_so101.json').exists())

    def test_temperature_pair_fault_is_injected_and_stops_before_any_hold(self):
        bridge = FakeBridge(SimClock(), corrupt_feedback_temp=True)
        summary, folder = self.run_main(bridge, 'unhealthy feedback')
        self.assertTrue(any(row[0] == "CORRUPT_TEMPERATURE" for row in bridge.history))
        self.assertFalse(any(row[0] == "HOLD" for row in bridge.history))
        self.assertEqual(bridge.cleanup, [False])
        self.assertFalse(summary['commit_confirmed'])
        self.assertFalse((folder/'rk7020_right_so101.json').exists())

    def test_exploration_timeout_sends_stop(self):
        bridge = FakeBridge(SimClock(), timeout_motor=3)
        summary, _ = self.run_main(bridge, "travel/direction/time limit reached")
        self.assertIn("time limit reached", summary["error"])
        self.assertEqual(bridge.cleanup, [False])
        self.assertTrue(bridge.closed)
        self.assertTrue(all(raw[40] == 0 for raw in bridge.registers.values()))

    def test_communication_exception_sends_stop(self):
        bridge = FakeBridge(SimClock(), communication_fail_after=18)
        summary, _ = self.run_main(bridge, "unconfirmed serial read")
        self.assertEqual(summary["status"], "NOT_COMPLETED")
        self.assertEqual(bridge.cleanup, [False])
        self.assertTrue(bridge.closed)

    def test_final_offset_write_failure_sends_stop_not_commit(self):
        bridge = FakeBridge(SimClock(), reject_final_write=True)
        summary, folder = self.run_main(bridge, "final offset write failure")
        self.assertEqual(summary["status"], "NOT_COMPLETED")
        self.assertEqual(bridge.cleanup, [False])
        self.assertFalse((folder / "rk7020_right_so101.json").exists())
        for motor in range(1, 7):
            self.assertEqual(bridge.registers[motor][:40], bridge.baseline[motor][:40])

    def test_inspect_never_enables_torque_or_changes_offsets(self):
        bridge = FakeBridge(SimClock())
        summary, _ = self.run_main(bridge, inspect=True)
        self.assertEqual(summary["status"], "INSPECT_PASS")
        self.assertFalse(any(row[0] == "WRITE" for row in bridge.history))
        self.assertEqual(bridge.cleanup, [True])
        self.assertFalse(summary["real_calibration_complete"])

    def test_final_offset_readback_mismatch_stops_before_commit(self):
        bridge = FakeBridge(SimClock(), corrupt_final_readback=True)
        summary, folder = self.run_main(bridge, "final configuration/position readback mismatch")
        self.assertEqual(summary["status"], "NOT_COMPLETED")
        self.assertEqual(bridge.cleanup, [False])
        self.assertFalse((folder / "rk7020_right_so101.json").exists())

    def test_close_exception_does_not_prevent_summary(self):
        bridge = FakeBridge(SimClock(), close_error=True)
        folder = self.root / "close_error_run"
        args = ["--port", "FAKE", "--out", str(folder), "--inspect"]
        with patch.object(cal, "time", bridge.clock), patch.object(cal, "Bridge", return_value=bridge), redirect_stdout(io.StringIO()):
            try:
                cal.main(args)
            except OSError:
                pass
        self.assertTrue((folder / "summary.json").exists(),
                        "serial.close failure must not erase the cleanup report")
        summary = json.loads((folder / "summary.json").read_text())
        self.assertIn("close_error", summary)
        self.assertEqual(bridge.cleanup, [True])

    def test_final_output_error_preserves_candidate_and_committed_state(self):
        bridge = FakeBridge(SimClock())
        original_write_text = Path.write_text

        def fail_only_final_calibration(path, *args, **kwargs):
            if path.name == "rk7020_right_so101.json":
                raise OSError("simulated final calibration JSON save failure")
            return original_write_text(path, *args, **kwargs)

        with patch.object(Path, "write_text", fail_only_final_calibration):
            summary, folder = self.run_main(bridge, "final calibration JSON save failure")

        self.assertEqual(summary["status"], "COMMITTED_OUTPUT_ERROR")
        self.assertTrue(summary["commit_confirmed"])
        self.assertFalse(summary["real_calibration_complete"])
        self.assertIn("rollback_not_applicable", summary)
        self.assertIn("already committed", summary["rollback_not_applicable"])
        self.assertEqual(bridge.cleanup, [True],
                         "a confirmed COMMIT must not be followed by a pretend rollback")
        self.assertTrue(bridge.closed)
        self.assertFalse((folder / "rk7020_right_so101.json").exists())
        candidate = json.loads((folder / "candidate_calibration.json").read_text())
        self.assertEqual(set(candidate), set(cal.MOTOR_IDS))
        for name, motor in cal.MOTOR_IDS.items():
            raw = bridge.registers[motor]
            self.assertEqual(cal.signed(raw[31:33], 11), candidate[name]["homing_offset"])
            self.assertEqual(cal.word(raw[9:11]), candidate[name]["range_min"])
            self.assertEqual(cal.word(raw[11:13]), candidate[name]["range_max"])
            self.assertEqual(raw[40], 0)


if __name__ == "__main__":
    unittest.main()
