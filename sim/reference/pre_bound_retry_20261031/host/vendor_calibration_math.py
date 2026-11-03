# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pure endpoint mathematics adapted from the manufacturer's LeRobot fork.

Source: https://github.com/JoyandAI/lerobot/blob/
072e8355fbb9a7b520915d44877aa876794efb31/src/lerobot/motors/auto_calibrate.py
Original source SHA256: 09258889dd89ddabec5b6411cfb4eeb669d50bfacdd69123fb30711c201cbbe3

Changes: no device classes, imports, serial access, EEPROM writes or movement;
accept zero-offset physical endpoint counts and return a native calibration
dictionary plus explanatory values; explicitly reject malformed endpoints,
impossible ranges and the upstream signed-offset half-turn ambiguity. This
module does not establish HLS geometry, model zero angles or motor directions.

Direction +1 means the upstream CLOCKWISE / positive velocity command, and
-1 means ANTI_CLOCKWISE / negative velocity. These names describe commands,
not an independently observed physical rotation direction.
"""

from numbers import Integral


SOURCE_COMMIT = "072e8355fbb9a7b520915d44877aa876794efb31"
SOURCE_URL = (
    "https://github.com/JoyandAI/lerobot/blob/" + SOURCE_COMMIT
    + "/src/lerobot/motors/auto_calibrate.py"
)
SERVO_RESOLUTION = 4096
TARGET_CENTER = (SERVO_RESOLUTION - 1) // 2  # Exact upstream choice: 2047.
SHOULDER_PAN_HALF_OFFSET_ADJUSTMENT = -150

ORDER = (
    "elbow_flex", "wrist_flex", "wrist_roll", "gripper",
    "shoulder_pan", "shoulder_lift",
)

MOTOR_IDS = {
    "shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
    "wrist_flex": 4, "wrist_roll": 5, "gripper": 6,
}

DIRECTIONS = {
    name: ((-1, +1) if name == "shoulder_lift" else (+1, -1))
    for name in MOTOR_IDS
}

PRE_ACTIONS = {
    "shoulder_lift": (("wrist_roll", -1), ("elbow_flex", -1)),
}

POST_ACTIONS = {
    "shoulder_lift": (("shoulder_lift", -1),),
    "elbow_flex": (("elbow_flex", +1),),
    "wrist_flex": (("wrist_flex", +1),),
}

# Upstream runs post-actions in bus.motors insertion order, NOT in ORDER.
RECOVERY_ORDER = tuple(MOTOR_IDS)
VENDOR_MIDPOINT_IMMEDIATE_STOP = ("shoulder_lift", "wrist_roll")


def _integer(value, label):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"{label} must be an integer")
    return int(value)


def normalize_homing_offset(offset, bits=11):
    """Upstream period folding, rejecting its non-encodable half-turn case.

    The vendor's sequential loops return +2048 for +/-2048 with 11 magnitude
    bits. The STS sign-magnitude encoding accepts only -2047..+2047; a caller
    must resolve that physical boundary rather than silently change its sign.
    """
    offset = _integer(offset, "offset")
    bits = _integer(bits, "bits")
    if bits < 1 or bits > 31:
        raise ValueError("bits must be in 1..31")
    max_offset = (1 << bits) - 1
    period = 1 << (bits + 1)
    while offset > max_offset:
        offset -= period
    while offset < -max_offset:
        offset += period
    if not -max_offset <= offset <= max_offset:
        raise ValueError("half-turn homing offset is not sign-magnitude encodable")
    return offset


def unwrap_physical_limit_range(first_limit, second_limit, second_direction,
                                resolution=SERVO_RESOLUTION):
    """Represent the directional endpoints as one increasing physical range."""
    first_limit = _integer(first_limit, "first_limit")
    second_limit = _integer(second_limit, "second_limit")
    resolution = _integer(resolution, "resolution")
    second_direction = _integer(second_direction, "second_direction")
    if resolution < 2 or not (0 <= first_limit < resolution
                              and 0 <= second_limit < resolution):
        raise ValueError("physical endpoints must lie within one servo turn")
    if second_direction not in (-1, +1):
        raise ValueError("direction must be +1 or -1")
    if first_limit == second_limit:
        raise ValueError("identical endpoints do not establish a mechanical range")
    if second_direction == +1:
        return first_limit, first_limit + ((second_limit - first_limit + resolution) % resolution)
    return second_limit, second_limit + ((first_limit - second_limit + resolution) % resolution)


def fold_unwrapped_range_to_single_turn(range_min, range_max, offset,
                                        max_position=SERVO_RESOLUTION - 1):
    """Apply the upstream whole-turn shifts; caller must check final bounds."""
    range_min = _integer(range_min, "range_min")
    range_max = _integer(range_max, "range_max")
    offset = _integer(offset, "offset")
    max_position = _integer(max_position, "max_position")
    resolution = max_position + 1
    if max_position < 1 or range_max <= range_min or range_max - range_min >= resolution:
        raise ValueError("range must have positive span smaller than one turn")
    while range_min < 0:
        range_min += resolution
        range_max += resolution
        offset -= resolution
    while range_max > max_position:
        range_min -= resolution
        range_max -= resolution
        offset += resolution
    return range_min, range_max, offset


def compute_calibration(name, motor_id, first_limit, second_limit):
    """Return ``(native_motor_calibration, auxiliary_endpoint_values)``.

    Endpoints must be captured with Homing_Offset=0 and ordered according to
    DIRECTIONS[name]. The zero-offset servo position is still periodic. Do
    not feed already offset-adjusted logical positions without converting
    them to that physical convention first.

    The gripper maps physical_range_min to count 0 and physical_range_max to
    count span. This preserves the manufacturer's algorithm; it deliberately
    does not label these endpoints as physically open/closed. Installation
    direction must be checked separately.
    """
    if name not in MOTOR_IDS:
        raise ValueError(f"unknown SO101 motor: {name}")
    motor_id = _integer(motor_id, "motor_id")
    if motor_id != MOTOR_IDS[name]:
        raise ValueError(f"{name} requires motor ID {MOTOR_IDS[name]}")
    first_limit = _integer(first_limit, "first_limit")
    second_limit = _integer(second_limit, "second_limit")
    first_direction, second_direction = DIRECTIONS[name]
    physical_min, physical_max = unwrap_physical_limit_range(
        first_limit, second_limit, second_direction,
    )
    span = physical_max - physical_min
    half_offset = span // 2
    if name == "shoulder_pan":
        half_offset += SHOULDER_PAN_HALF_OFFSET_ADJUSTMENT
    if name != "gripper" and not 0 <= half_offset <= span:
        raise ValueError("vendor midpoint adjustment lies outside measured range")

    if name == "gripper":
        raw_offset = physical_min
        range_min, range_max = 0, span
        physical_zero = physical_min
    else:
        physical_zero = physical_min + half_offset
        raw_offset = physical_zero - TARGET_CENTER
        range_min = TARGET_CENTER - half_offset
        range_max = TARGET_CENTER + (span - half_offset)

    offset = normalize_homing_offset(raw_offset)
    range_min, range_max, offset = fold_unwrapped_range_to_single_turn(
        range_min, range_max, offset,
    )
    offset = normalize_homing_offset(offset)
    if not 0 <= range_min < range_max < SERVO_RESOLUTION:
        raise ValueError("calculated logical range cannot fit one servo turn")

    calibration = {
        "id": motor_id,
        "drive_mode": 0,
        "homing_offset": offset,
        "range_min": range_min,
        "range_max": range_max,
    }
    auxiliary = {
        "span": span,
        "physical_range_min": physical_min,
        "physical_range_max": physical_max,
        "first_limit": first_limit,
        "second_limit": second_limit,
        "first_direction": first_direction,
        "second_direction": second_direction,
        "half_offset": half_offset,
        "physical_zero_unwrapped": physical_zero,
        "physical_zero": physical_zero % SERVO_RESOLUTION,
        "target_center": 0 if name == "gripper" else TARGET_CENTER,
        "vendor_midpoint_motion": name != "gripper"
            and name not in VENDOR_MIDPOINT_IMMEDIATE_STOP,
        "source_commit": SOURCE_COMMIT,
        "physical_alignment_verified": False,
        "geometry_verified": False,
    }
    return calibration, auxiliary
