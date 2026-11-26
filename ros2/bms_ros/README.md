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

## Run

```bash
# check the BMS answers (no ROS graph needed)
ros2 run bms_ros bms_probe --port /dev/ttyUSB0 --protocol jbd --verbose

# start the node
ros2 launch bms_ros bms.launch.py port:=/dev/ttyUSB0 protocol:=jbd

# look at the data
ros2 topic echo /bms/battery_state --qos-reliability best_effort
ros2 topic echo /bms/status
```

For more than one pack, launch the node once per pack with a different namespace:

```bash
ros2 launch bms_ros bms.launch.py namespace:=pack_a port:=/dev/ttyUSB0 protocol:=jbd
ros2 launch bms_ros bms.launch.py namespace:=pack_b port:=/dev/ttyUSB1 protocol:=daly
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
| `port` | `/dev/ttyUSB0` | serial device |
| `baud` | `9600` | baud rate |
| `protocol` | `jbd` | `jbd` or `daly` |
| `rate` | `1.0` | polls per second |
| `timeout` | `0.6` | seconds to wait for each reply |
| `frame_id` | `battery` | header frame |
| `location`, `serial_number` | `""` | copied into `BatteryState` |
| `design_capacity` | `0.0` | Ah, 0 = unknown |
| `technology` | `auto` | `auto` (guessed from cell voltage), `lifepo4`, `lion`, `lipo`, `unknown` |
| `full_soc_threshold` | `99.0` | SOC % at which an idle pack reports FULL |
| `idle_current` | `0.05` | A; smaller currents count as idle |
| `debug_frames` | `false` | log every TX/RX frame in hex |

If the port can't be opened or is unplugged, the node keeps running and retries on every poll. Diagnostics report ERROR until the port works again.

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

**Without ROS:** the protocol code in `bms_ros/protocols.py` and `bms_ros/client.py` only needs pyserial:

```python
import serial
from bms_ros.client import BmsClient
with serial.Serial('/dev/ttyUSB0', 9600, timeout=0.05) as port:   # or 'COM19' on Windows
    print(BmsClient(port, 'jbd').read().to_dict())
```

## Tests

The tests run against a simulated BMS, so no hardware or ROS is needed:

```bash
cd ros2/bms_ros && python3 -m pytest test/
# or inside a workspace
colcon test --packages-select bms_ros && colcon test-result --verbose
```

## Package layout

```
bms_ros/
├── bms_ros/
│   ├── protocols.py   JBD + Daly frame building and decoding (pure Python)
│   ├── client.py      serial polling client + bms_probe CLI
│   └── bms_node.py    ROS 2 node
├── config/bms.yaml
├── launch/bms.launch.py
├── test/test_protocols.py
├── package.xml
└── setup.py
```
