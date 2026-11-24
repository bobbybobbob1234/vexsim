@echo off
cd /d "%~dp0"
start "" vex_sim.exe --wheel-radius 1.625 --track-width 13 --max-rpm 600 --gear-ratio 0.75 --left-ports 11,-2,-1 --right-ports -20,9,10 --robot-width 18 --robot-height 18 --start-x -61.2 --start-y 37.55 --start-heading 90
