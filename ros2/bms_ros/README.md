# bms_ros

A ROS 2 driver for **JBD / Xiaoxiang** and **Daly** smart BMS boards over USB serial. It uses the same protocols as the browser monitor in this repo and publishes the standard `sensor_msgs/BatteryState`, so any node, Nav2, RViz plugin or web dashboard can use it without custom message types.

Tested target: **ROS 2 Jazzy** on Ubuntu 24.04 (it also works on Humble).

## Topics and services

All names are relative to the node's namespace, which is `/bms` by default.

| Name | Type | Contents |
|---|---|---|
| `/bms/battery_state` | `sensor_msgs/BatteryState` | voltage, current, charge/capacity (Ah), percentage (0–1), per-cell voltages (V), average temperature, status (charging / discharging / full) and health (good / overheat / cold / overvoltage / failure) |
| `/bms/status` | `std_msgs/String` (JSON) | every decoded field: cells in mV, every temperature, FET states, cycles, balancing mask, fault names, power. Easy to read from rosbridge or a web UI |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | OK / WARN (FET off, cell delta > 50 mV) / ERROR (protection fault, no reply), with key values |
| `/bms/reconnect` | `std_srvs/Trigger` | close and reopen the serial port |

`battery_state` uses the sensor-data QoS (best effort). Subscribe with `qos_profile_sensor_data`, or `--qos-reliability best_effort` on the command line.

## Install

```bash
sudo apt install python3-serial
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/MdAbdurRahaman/Browser-based-BMS-monitor.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select bms_ros
source install/setup.bash
```

Give your user access to serial ports once, then log out and back in:

```bash
sudo usermod -aG dialout $USER
```

## Configuration: `.env` and auto-detection

Port and protocol normally come from a `.env` file. Copy [`.env.example`](.env.example) to `.env`:

```
BMS_PORT=/dev/ttyUSB0      # COM26 on Windows
BMS_PROTOCOL=daly          # daly | jbd
BMS_BAUD=9600
BMS_RATE=1.0
BMS_TIMEOUT=0.6
```

The node looks for `.env` in this order: the `env_file` parameter, `$BMS_ENV_FILE`, the current folder, this package folder, the repository root. Environment variables named `BMS_*` override the file, and ROS parameters override both.

**Auto-detection.** If `BMS_PORT` is empty or `auto`, or the port doesn't exist (unplugged, renumbered), or it doesn't answer, the node tries every USB serial adapter. Known BMS cable chips go first (CH340/CH341/CH9102, CP210x, FTDI, PL2303). On each port it tries Daly and then JBD (or only `BMS_PROTOCOL` if set) and uses the first one that answers. Bluetooth and built-in serial ports are skipped. When the connection drops, it searches again every 5 seconds.

## Run

```bash
# check the BMS answers (no ROS graph needed)
ros2 run bms_ros bms_probe --verbose          # uses .env / auto-detect
ros2 run bms_ros bms_probe --list             # list USB serial ports

# start the node
ros2 launch bms_ros bms.launch.py             # .env / auto-detect
ros2 launch bms_ros bms.launch.py port:=/dev/ttyUSB0 protocol:=daly

# look at the data
ros2 topic echo /bms/battery_state --qos-reliability best_effort
ros2 topic echo /bms/status
```

For more than one pack, launch the node once per pack with its own namespace and `.env` (give each a fixed port, since auto-detect picks the first BMS it finds):

```bash
ros2 launch bms_ros bms.launch.py namespace:=pack_a env_file:=$HOME/pack_a.env
ros2 launch bms_ros bms.launch.py namespace:=pack_b env_file:=$HOME/pack_b.env
```

Use `/dev/serial/by-id/usb-...` instead of `/dev/ttyUSB0` so the port name stays the same after re-plugging.

### Windows COM port (e.g. COM19) with ROS in WSL

WSL can't see Windows COM ports directly. Pass the USB adapter through with [usbipd-win](https://github.com/dorssel/usbipd-win):

```powershell
# PowerShell as administrator
usbipd list                       # find the BUSID of the CH340 / CP210x adapter
usbipd bind --busid 2-3
usbipd attach --wsl --busid 2-3
```

Inside WSL it then appears as `/dev/ttyUSB0`. While it is attached, Windows (and the browser monitor) can't use COM19. Run `usbipd detach --busid 2-3` to give it back.

## Parameters

See [`config/bms.yaml`](config/bms.yaml).

| Parameter | Default | Meaning |
|---|---|---|
| `port` | `auto` | serial device; `auto` = `.env`, then detect |
| `protocol` | `auto` | `daly`, `jbd`, or `auto` |
| `baud` | `0` | 0 = `.env` or 9600 |
| `rate` | `0.0` | readings per second; 0 = `.env` or 1.0 |
| `timeout` | `0.0` | seconds per reply; 0 = `.env` or 0.6 |
| `env_file` | `""` | explicit `.env` path |
| `frame_id` | `battery` | header frame of `battery_state` |
| `debug_frames` | `false` | log every byte sent and received |

If no BMS is found, the node keeps running and retries. Diagnostics report ERROR until a BMS answers. `/bms/status` includes the `port` that is in use.

## Using it from your own nodes

**Python subscriber:**

```python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

self.create_subscription(BatteryState, '/bms/battery_state', self.on_batt, qos_profile_sensor_data)

def on_batt(self, m):
    self.get_logger().info(f'{m.voltage:.2f} V  {m.current:.1f} A  {m.percentage*100:.0f}%  cells={list(m.cell_voltage)}')
```

**Full JSON (faults, FETs, cycles):**

```python
import json
from std_msgs.msg import String
self.create_subscription(String, '/bms/status', lambda m: print(json.loads(m.data)['faults']), 10)
```

**Without ROS:** `bms_node.py` falls back to a console reader when `rclpy` isn't installed, for example on Windows. It only needs pyserial:

```bash
python -m bms_ros.bms_node --loop 2            # .env / auto-detect, print JSON every 2 s
python -m bms_ros.bms_node --port COM26 --protocol daly --verbose
```

```python
from bms_ros.bms_node import BmsConnection, Settings
conn = BmsConnection(Settings.load())          # reads .env, auto-detects
print(conn.read().to_dict(), conn.port)
```

## Tests

The tests run against a simulated BMS, so no hardware or ROS is needed:

```bash
pip install -r ../../requirements-dev.txt
cd ros2/bms_ros && python3 -m pytest test/
# or inside a workspace
colcon test --packages-select bms_ros && colcon test-result --verbose
```

## Package layout

```
bms_ros/
├── bms_ros/
│   ├── protocols.py   Daly + JBD: request bytes, reply decoding (no serial, no ROS)
│   └── bms_node.py    settings (.env), serial reading, auto-detect, console mode, ROS 2 node
├── .env.example
├── config/bms.yaml
├── launch/bms.launch.py
├── test/test_protocols.py
├── package.xml
└── setup.py
```
