Native protocol tests (requires C compiler):

`gcc -std=c99 -Wall -Wextra -Werror tests/test_st3215.c firmware/st3215.c -o tests/test_st3215`

Run the resulting executable. Does not connect to hardware. Full mocked IK regression results are in validation.json.

Bounded motion and bus fault tests:

`gcc -std=c99 -Wall -Wextra -Werror tests/test_motor_control.c firmware/motor_control.c firmware/st3215.c -lm -o tests/test_motor_control`

Run the executable. It tests the real driver and controller with simulated servo memory; no serial port is opened.

Host framing and command sequencing: `python tests/test_motor_host.py` (mock serial, no hardware access).

Calibration-specific checks: `python tests/run_autocal_tests.py`. This compiles
the ST3215 protocol, CAL1 controller and trace harnesses with warnings as errors,
then runs host sequencing, vendor mathematics, trace retrieval and baseline
sync tests. The report is `autocal_unit_results.json`; all buses are simulated.

The host model includes this batch's observed MODE-change ACC reset and
automatic torque enable on goal/speed writes. It checks ACC restoration/readback
before motion, real READ62/2 temperature failure injection, partial-calibration
rejection and STOP on unconfirmed transactions. Native tests cover OFF-first
cleanup and blocking Mode/Speed/EEPROM writes when OFF is unconfirmed.

These checks do not certify physical endpoints, kinematic alignment or the
latest BOOT on the board. Inspect `../validation.json` and the specific real
`../results/<run>/summary.json` for those outcomes.

`test_auto_calibrate_usb.py` adds 18 tests for direct USB protocol parsing,
complete backup, all-axis caps, mode/offset conventions, temperature faults,
monitoring gaps, uncertain OFF, repeat cleanup and complete simulated COMMIT.
The full runner includes this file. It opens no serial port.
