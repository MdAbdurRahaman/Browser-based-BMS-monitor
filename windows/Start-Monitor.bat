@echo off
title BMS Monitor
rem Prints live BMS readings every 2 seconds.
rem Port and protocol come from the .env file in the project folder; if it is missing,
rem or the port there is wrong, the BMS is found automatically.
rem You can still override: Start-Monitor.bat COM26 daly
cd /d "%~dp0..\ros2\bms_ros"
where python >nul 2>nul || (echo Python was not found. Install it from https://www.python.org/ and tick "Add to PATH". & pause & exit /b 1)
python -c "import serial" 2>nul || pip install pyserial
set ARGS=
if not "%~1"=="" set ARGS=--port %~1
if not "%~2"=="" set ARGS=%ARGS% --protocol %~2
echo Press Ctrl+C to stop.
python -m bms_ros.bms_node --loop 2 %ARGS%
pause
