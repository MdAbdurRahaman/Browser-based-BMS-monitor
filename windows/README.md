# Running on Windows

Double-click launchers. They need [Python](https://www.python.org/downloads/) on PATH; the libraries in `requirements.txt` (only pyserial) are installed automatically on first run.

| File | What it does |
|---|---|
| `Start-WebApp.bat` | Serves the web app at `http://localhost:8765` and opens it in Microsoft Edge with the Daly protocol selected. Click **Connect COM port** and pick `USB-SERIAL CH340`. |
| `Start-Monitor.bat` | Prints live readings (JSON) every 2 seconds in the console. Port and protocol come from the `.env` file in the project folder; if it is missing or the port is wrong, the BMS is found automatically. Override with `Start-Monitor.bat COM5 jbd`. |
| `List-Ports.bat` | Lists the USB serial ports. |

## .env

Create `.env` in the project folder (copy `ros2/bms_ros/.env.example`):

```
BMS_PORT=COM26
BMS_PROTOCOL=daly
```

Leave `BMS_PORT` empty to always auto-detect.

Only one program can use the COM port at a time: close the vendor tool (PCMaster), the web app or the monitor before starting another.

The web app needs Chrome or Edge (Firefox has no Web Serial). Add `#daly` or `#jbd` to the page address to pre-select the protocol.
