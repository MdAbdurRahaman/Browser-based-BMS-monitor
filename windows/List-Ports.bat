@echo off
title Serial ports
python -c "import serial" 2>nul || pip install pyserial
python -m serial.tools.list_ports -v
echo.
echo The BMS adapter shows as "USB-SERIAL CH340". Use its COM number with Start-Monitor.bat, e.g.:
echo   Start-Monitor.bat COM26 daly
pause
