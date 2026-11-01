"""Hardware-free tests for the adapted manufacturer endpoint mathematics."""

import importlib.util
import json
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "host" / "vendor_calibration_math.py"
SPEC = importlib.util.spec_from_file_location("vendor_calibration_math", MODULE_PATH)
math = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(math)


class VendorCalibrationMathTests(unittest.TestCase):
    def test_order_directions_and_recovery_actions(self):
        self.assertEqual(math.ORDER, ("elbow_flex", "wrist_flex", "wrist_roll",
                                     "gripper", "shoulder_pan", "shoulder_lift"))
        for name in math.MOTOR_IDS:
            self.assertEqual(math.DIRECTIONS[name], (-1, 1) if name == "shoulder_lift" else (1, -1))
        self.assertEqual(math.PRE_ACTIONS["shoulder_lift"], (("wrist_roll", -1), ("elbow_flex", -1)))
        actual = [action for name in math.RECOVERY_ORDER
                  for action in math.POST_ACTIONS.get(name, ())]
        self.assertEqual(actual, [("shoulder_lift", -1), ("elbow_flex", 1), ("wrist_flex", 1)])

    def test_all_six_axes_native_json(self):
        for name, motor_id in math.MOTOR_IDS.items():
            first, second = (900, 3100) if name == "shoulder_lift" else (3100, 900)
            calibration, aux = math.compute_calibration(name, motor_id, first, second)
            self.assertEqual(set(calibration), {"id", "drive_mode", "homing_offset", "range_min", "range_max"})
            self.assertEqual(calibration["id"], motor_id)
            self.assertEqual(calibration["drive_mode"], 0)
            self.assertEqual(aux["span"], 2200)
            self.assertEqual(aux["physical_range_min"], 900)
            self.assertEqual(aux["physical_range_max"], 3100)
            self.assertFalse(aux["physical_alignment_verified"])
            self.assertFalse(aux["geometry_verified"])
            json.dumps(calibration)

    def test_unwrapped_crossing_zero(self):
        for name in ("wrist_flex", "shoulder_lift"):
            endpoints = (800, 3500) if name == "wrist_flex" else (3500, 800)
            calibration, aux = math.compute_calibration(name, math.MOTOR_IDS[name], *endpoints)
            self.assertEqual((aux["physical_range_min"], aux["physical_range_max"]), (3500, 4896))
            self.assertEqual(aux["span"], 1396)
            self.assertEqual(aux["physical_zero"], 102)
            self.assertEqual(calibration["homing_offset"], -1945)
            self.assertEqual((calibration["range_min"], calibration["range_max"]), (1349, 2745))

    def test_pan_minus_150_and_vendor_immediate_stop(self):
        calibration, aux = math.compute_calibration("shoulder_pan", 1, 3100, 900)
        self.assertEqual(aux["half_offset"], 950)
        self.assertEqual(aux["physical_zero"], 1850)
        self.assertEqual(calibration["homing_offset"], -197)
        self.assertEqual((calibration["range_min"], calibration["range_max"]), (1097, 3297))
        self.assertTrue(aux["vendor_midpoint_motion"])
        for name in math.VENDOR_MIDPOINT_IMMEDIATE_STOP:
            endpoints = (900, 3100) if name == "shoulder_lift" else (3100, 900)
            _, aux = math.compute_calibration(name, math.MOTOR_IDS[name], *endpoints)
            self.assertFalse(aux["vendor_midpoint_motion"])

    def test_gripper_maps_min_to_zero_max_to_span(self):
        calibration, aux = math.compute_calibration("gripper", 6, 3300, 1800)
        self.assertEqual((calibration["range_min"], calibration["range_max"]), (0, 1500))
        self.assertEqual(calibration["homing_offset"], 1800)
        self.assertEqual(aux["physical_zero"], 1800)
        self.assertFalse(aux["vendor_midpoint_motion"])
        calibration, aux = math.compute_calibration("gripper", 6, 600, 3500)
        self.assertEqual(calibration["homing_offset"], -596)
        self.assertEqual(calibration["range_max"], 1196)

    def test_odd_span_preserves_upstream_2047_choice(self):
        calibration, aux = math.compute_calibration("elbow_flex", 3, 3101, 900)
        self.assertEqual(aux["half_offset"], 1100)
        self.assertEqual((calibration["range_min"], calibration["range_max"]), (947, 3148))
        self.assertEqual(calibration["homing_offset"], -47)

    def test_normalization_and_reject_ambiguous_half_turn(self):
        self.assertEqual(math.normalize_homing_offset(3500), -596)
        self.assertEqual(math.normalize_homing_offset(-3000), 1096)
        self.assertEqual(math.normalize_homing_offset(4096 + 123), 123)
        for offset in (2048, -2048, 4096 + 2048):
            with self.assertRaises(ValueError):
                math.normalize_homing_offset(offset)
        with self.assertRaises(ValueError):
            math.compute_calibration("gripper", 6, 3300, 2048)

    def test_reject_invalid_endpoints_mapping_and_ranges(self):
        for first, second in ((1000, 1000), (-1, 1000), (4096, 1000), (1000.1, 2000), (True, 2000)):
            with self.assertRaises(ValueError):
                math.compute_calibration("elbow_flex", 3, first, second)
        with self.assertRaises(ValueError):
            math.compute_calibration("unknown", 1, 3100, 900)
        with self.assertRaises(ValueError):
            math.compute_calibration("elbow_flex", 2, 3100, 900)
        with self.assertRaises(ValueError):
            math.compute_calibration("shoulder_pan", 1, 1100, 1000)
        with self.assertRaises(ValueError):
            math.compute_calibration("shoulder_pan", 1, 4095, 0)
        with self.assertRaises(ValueError):
            math.unwrap_physical_limit_range(3100, 900, 0)
        with self.assertRaises(ValueError):
            math.fold_unwrapped_range_to_single_turn(0, 4096, 0)


if __name__ == "__main__":
    unittest.main()
