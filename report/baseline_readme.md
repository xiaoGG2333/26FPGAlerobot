# RK7100 SO101 native 2025.2 migration

Independent source snapshot. Original project outside this directory is read-only.
All firmware/HLS/host/RTL behavior is protected by manifests/baseline_runtime_sha256.json.
Toolchain: config/toolchain.json. Python 3 required for orchestration.

Build: `python scripts/build_all.py`
Resume: `python scripts/build_all.py --from-stage hardware`
Stages: check, csim, synth, cosim, package, hardware, platform, boot.
Logs: logs/. Status: reports/build_status.json. Output: board/boot/BOOT.BIN.
Clean reproduction: use an empty build/ and board/boot/ in a copied source tree.
No old XPR, BSP, prebuilt IP, or old toolchain output is needed.
The included 2024.2 solver library SOURCE is pinned intentionally; no prebuilt library is reused.
Hardware: xc7z100ffg900-2, 125MHz, IK 0x40000000/0x40010000, thermal GPIO 0x41200000.
Do not use a partial or failed build on hardware. No build command connects to a board.
Physical validation has NOT been performed for this migration. The old version remains authoritative until acceptance.

Additional build gates: tools, abi, bdcheck. The hardware project is build/v/ik.xpr;
the Vitis workspace is build/ws. Compact names avoid Windows IP path-length failures.
The XADC simulation stimulus is tests/fixtures/xadc_design.txt.
Optional native regressions use config/toolchain.json native_gcc.
Ubuntu Xbox runtime dependencies may be pinned with config/host_requirements_lock.txt.
Create an isolated virtualenv before installation; do not install evdev on Windows.
Historical Joy-Con input additionally needs the original joycon-robotics dependency.
See migration_guide.md for the Chinese build and board acceptance instructions.

Windows checkout root must be <=44 characters and contain no spaces (e.g. D:/fpga2025). This build entry checks the limit before launching tools.
