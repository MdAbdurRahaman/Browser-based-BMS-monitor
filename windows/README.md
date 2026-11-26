# Running on Windows

Double-click launchers. They need [Python](https://www.python.org/downloads/) on PATH; pyserial is installed automatically.

| File | What it does |
|---|---|
| `Start-WebApp.bat` | Serves the web app at `http://localhost:8765` and opens it in Microsoft Edge with the Daly protocol selected. Click **Connect COM port** and pick `USB-SERIAL CH340`. |
| `Start-Monitor.bat` | Prints live readings (JSON) every 2 seconds in the console. Defaults to `COM26` and `daly`; for another port run `Start-Monitor.bat COM5 jbd`. |
| `List-Ports.bat` | Lists the serial ports so you can find the BMS adapter's COM number. |

Only one program can use the COM port at a time: close the vendor tool (PCMaster), the web app or the monitor before starting another.

The web app needs Chrome or Edge (Firefox has no Web Serial). Add `#daly` or `#jbd` to the page address to pre-select the protocol.
