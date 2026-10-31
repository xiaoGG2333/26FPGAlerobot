# Project scope
This directory is the authoritative RK-ZYNQ7100-F SO101 project despite its RK7020_So_101 directory name. Keep subsequent project edits, build artifacts, logs and packages here.
The baseline is rk7020_so101_xbox_temp1500_speed100_600_20261011. Preserve inverse kinematics math/precision, iteration behavior, encoder mapping/endpoints, temperature filter, speed 100..600 and servo torque/fault policies unless the user specifically requests changes. Board adaptation must remain separate from algorithm/control changes.
Run scripts/check_source_parity.py to audit the initial migration. Hardware movement is not part of build validation. Do not silently enable motion in build or diagnostic scripts.

The user authorized chip thermal protection and telemetry: PL 25kHz fan50%, >=65C full with60C recovery, >=75C or sensor fault freezes new IK and retains servo goals/torque; explicit resume only below65C. Original servo55C/1.5s shutdown policy stays unchanged. Thermal frames are sideband; do not change RESULT1/STATE1 layouts.

User-authorized optimization 2026-10-18: retain partition; 125MHz FCLK0 with
counter/ADC calibration, bounded tight polling, float fixed-size spatial matrix
structure; production monotonic damping trial acceptance. PS online C2 quintic
S-shaped joint plans, symmetric velocity/acceleration/jerk limits configurable
in RAM. Chip >60C/>1500ms freeze, <55C explicit resume; fan55/50 hysteresis;
no periodic normal telemetry. Protect mapping/endpoints, servo55C/>1500ms filter
and torque/fault policies. Original sources/BOOT in reference/pre_opt_20261018;
new build_opt125 and boot_opt125, no physical movement during validation.
