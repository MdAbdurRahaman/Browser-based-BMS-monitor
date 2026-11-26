@echo off
title BMS Web App
rem Serves the BMS web app on http://localhost:8765 and opens it in Microsoft Edge.
rem Web Serial needs Chrome or Edge, and a localhost or https page.
cd /d "%~dp0.."
where python >nul 2>nul || (echo Python was not found. Install it from https://www.python.org/ and tick "Add to PATH". & pause & exit /b 1)
start "BMS web server (close to stop)" /min python -m http.server 8765 --bind 127.0.0.1
timeout /t 2 /nobreak >nul
start "" msedge "http://localhost:8765/index.html#daly"
echo.
echo  BMS web app: http://localhost:8765/index.html
echo  In the page click "Connect COM port" and pick "USB-SERIAL CH340 (COM26)".
echo  Close pcmaster first - only one program can use the port.
echo  To stop the web server, close the minimised "BMS web server" window.
echo.
pause
