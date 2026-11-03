@echo off
cd /d "%~dp0.." || exit /b 1
if not exist results\w3_pose mkdir results\w3_pose
python -u sw\pose_map.py --source 0 --record results\w3_pose\calib_rec.mp4 > results\w3_pose\calib_run.log 2>&1
