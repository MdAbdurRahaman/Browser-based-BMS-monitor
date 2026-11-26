@echo off
title BMS Monitor
rem Prints live BMS readings every 2 seconds. Usage: Start-Monitor.bat [PORT] [PROTOCOL]
rem Defaults: COM26 daly
set PORT=%~1
if "%PORT%"=="" set PORT=COM26
set PROTO=%~2
if "%PROTO%"=="" set PROTO=daly
cd /d "%~dp0..\ros2\bms_ros"
where python >nul 2>nul || (echo Python was not found. Install it from https://www.python.org/ and tick "Add to PATH". & pause & exit /b 1)
python -c "import serial" 2>nul || pip install pyserial
echo Reading %PROTO% BMS on %PORT%. Press Ctrl+C to stop.
python -m bms_ros.client --port %PORT% --protocol %PROTO% --loop 2
pause
