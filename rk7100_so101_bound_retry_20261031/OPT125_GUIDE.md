# RK7100 SO101 OPT125 / online trajectory update (2026-10-18)

## Scope
PS retains complete IK iteration, endpoint acceptance and motion planning. PL
retains a single float LM step with unchanged AXI register addresses and two cores.
No timing telemetry was added. No board programming or motor movement is part of
build/test validation. Physical calibration/geometry flags remain unverified.

## Changes
- FCLK0 and HLS target 125MHz (8ns); UART reference remains 100MHz. FSBL must be
  updated together with bitstream/app. Fan period/tach/stale use 125MHz; XADC
  divider 10 retains 12.5MHz ADC clock. Do NOT combine old FSBL/bit with new app.
- Fixed-size spatially parallel symmetric JtJ and Jte, float Cholesky columns
  with parallel off-diagonals; no fixed-point conversion. RTL regression required.
- Bounded tight ap_done polling, free-running PS timer deadline; no per-poll 10us sleep.
- Production IK: bounded trial-step retries, accept only lower energy, damping
  x0.7 on acceptance / x4 on rejection, bounded 1e-5..1000. This is monotonic
  damping adaptation, NOT gain-ratio LM. Original ik_solve retained for selftests.
- New valid endpoint goes to a transactional PS five-axis quintic Bezier plan.
  Common duration, continuous position/velocity/acceleration on replan; position,
  velocity, acceleration and jerk certified using subdivided Bernstein bounds.
  Ruckig-inspired API/constraints, NOT Ruckig code or a time-optimal solver.
  Infeasible replan rejects only the new endpoint. Jerk is bounded, not continuous.
- 20ms nominal waypoint service. Serial/IK work may delay service; never jump to
  catch up. Feedback/health at original 80ms. These are NOT hard realtime claims.
- ID1/5 velocity limits 600; ID2/3/4 900 counts/s. Register speeds ID1/5 100..600,
  ID2/3/4 400..900; mathematical trajectory velocity can be zero or below 400.
  Default acceleration [1200,1200,1500,1800,1800] counts/s^2 and jerk
  [6000,6000,7500,9000,9000] counts/s^3. These are provisional software limits.
  Internal servo ACC remains 5; actual acceleration/following needs bench checks.
- No normal periodic temperature frames. Fault entry and then at most every 2s;
  one recovery notice. Explicit THERMAL1 query still returns a diagnostic snapshot.
  Chip >60C continuously >1500ms freezes new targets (torque retained); recovery
  <55C plus explicit ARM resume. Sensor/OT faults retain immediate fault handling.
  Fan full >=55C, back to 50% <=50C. Servo >55C/>1500ms filter unchanged.
- Mapping, measured encoder endpoints, Xbox axes, gripper preview only, STOP off,
  failed startup off and servo temperature off remain unchanged. On recoverable
  faults progression freezes at the last transmitted waypoint; final accepted
  endpoint is retained. Resume replans from current feedback to that endpoint.
- Normal host console output 1Hz, --verbose enables all frames. CSV remains full.

## Ubuntu / SD
Use the ZIP matching this update, not a renamed old folder. Copy boot/BOOT.BIN
from the new package to the SD root (back up old BOOT first), safely eject and
cold boot in SD mode. Old known-good image is retained in the Windows project at
reference/pre_opt_20261018/boot/BOOT.BIN. The old complete release ZIP is retained.

```bash
source ~/venvs/joycon-rk7020/bin/activate
python -m zipfile -e "$HOME/下载/rk7100_so101_opt125_20261018.zip" "$HOME/"
cd ~/rk7100_so101_opt125_20261018
python -m serial.tools.list_ports -v
python host/xbox_input.py --list
python scripts/verify_board.py --port /dev/ttyUSB1 --servos --out results/read_only.json
```
HELLO must include OPT125_SCURVE_20261018_V1. THERMAL1 diagnostic below60C,
valid fan/tach and six valid servo replies are required before a physical trial.
First use preview (no drive flags), then start a supported short-path trial:

```bash
python -u host/xbox_control.py --port /dev/ttyUSB1 --xbox-device /dev/input/event6 --xbox-right-axes rxry --hz 5 --csv results/preview.csv
# Next command enables physical motion only after Enter. Support the arm for STOP.
python -u host/xbox_control.py --port /dev/ttyUSB1 --xbox-device /dev/input/event6 --xbox-right-axes rxry --test-drive --manual-control --hz 5 --csv results/drive.csv
```
Use actual device names. Neutral finishes the last target with torque on; Ctrl+C
sends STOP/torque off. Check startup, gentle direction change, neutral completion,
actual-vs-target following, and warm operation before increasing input rate.
No ID5 automatic test was performed. Confirm movement in chat before such a test.

Optional RAM-only per-axis profiles (applied before enable; reset on power cycle):
```
--joint-max-speeds 600 900 900 900 600
--joint-max-accelerations 1200 1200 1500 1800 1800
--joint-max-jerks 6000 6000 7500 9000 9000
```
Speed ceilings are 600/900/900/900/600; acceleration/jerk command ceilings 10000/
100000 are validation caps, not certified physical limits. Lower values allowed.

## Rebuild / validation
Windows build scripts default to build_opt125 project and boot_opt125 output;
old build/ project and old boot/ remain for reference/rollback.
Run scripts/build_all.ps1 -Stage All; RTL sim via run_thermal_sim.ps1/run_xadc_sim.ps1;
new native tests via scripts/run_opt_native.py. reports/opt_scope_audit.json records
protected input/mapping/pins and unchanged servo-temperature filter.
Full legacy Python discovery includes historical calibration fixtures absent from
this migrated project and stale legacy expectations. Compare its failed IDs with
the pre-update baseline; do not report that full suite as passing.
