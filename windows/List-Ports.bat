@echo off
title Serial ports
cd /d "%~dp0..\ros2\bms_ros"
python -c "import serial" 2>nul || pip install pyserial
echo USB serial ports (the BMS cable shows as USB-SERIAL CH340):
python -m bms_ros.bms_node --list
echo.
echo To fix the port, put it in the .env file in the project folder, e.g. BMS_PORT=COM26
pause
