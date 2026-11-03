"""Prepare a disposable tool layout from the competition sources and run an offline stage."""
import argparse
import filecmp
import json
import shutil
import subprocess
import sys
from pathlib import Path

build = Path(__file__).resolve().parent
root = build.parent
work = build / "work"
offline = {
    "verify_software": "verify_software.py",
    "run_opt_native": "run_opt_native.py",
    "run_thermal_sim": "run_thermal_sim.ps1",
    "run_xadc_sim": "run_xadc_sim.ps1",
}


def prepare():
    layout = json.loads((build / "layout.json").read_text(encoding="utf-8"))
    for original, current in layout.items():
        # Board binaries and archived reports are evidence, never fresh build outputs.
        if original.startswith(("board/", "reports/")):
            continue
        src, dst = root / current, work / original
        dst.parent.mkdir(parents=True, exist_ok=True)
        if not dst.exists() or not filecmp.cmp(src, dst, shallow=False):
            shutil.copyfile(src, dst)
    (work / "reports").mkdir(exist_ok=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", choices=offline)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--from-stage")
    parser.add_argument("--through")
    args = parser.parse_args()
    if sum((bool(args.run), args.prepare_only, bool(args.from_stage or args.through))) > 1:
        parser.error("Select one action: prepare, offline test, or build stages.")
    prepare()
    if args.prepare_only:
        print(work)
        sys.exit(0)
    script = work / "scripts" / (offline[args.run] if args.run else "build_all.py")
    command = (["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File"]
               if script.suffix == ".ps1" else [sys.executable])
    command.append(str(script))
    for flag, value in (("--from-stage", args.from_stage), ("--through", args.through)):
        if value:
            command.extend([flag, value])
    sys.exit(subprocess.call(command, cwd=work))
