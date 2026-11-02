@echo off
cd /d D:\C_O_D_E\2026\FPGA_AMD
D:\Anaconda3\envs\hand-pi\python.exe -u sw\pose_map.py --source 0 --record results\w3_pose\calib_rec.mp4 1> results\w3_pose\calib_run.log 2>&1
