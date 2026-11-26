"""
BMS reader: finds the BMS on a serial port, reads it, and shares the readings.

Two ways to run it:
  ROS 2 node    ros2 run bms_ros bms_node             (publishes ROS topics)
  Console       python -m bms_ros.bms_node --loop 2   (prints JSON; no ROS needed, works on Windows)

Where settings come from (first one wins):
  1. command-line options or ROS parameters
  2. environment variables BMS_PORT, BMS_PROTOCOL, BMS_BAUD, BMS_RATE, BMS_TIMEOUT
  3. a .env file (see .env.example)
  4. auto-detect: every USB serial port is tried with each protocol until a BMS answers

ROS topics (inside the node's namespace, /bms by default):
  battery_state  sensor_msgs/BatteryState  standard battery message
  status         std_msgs/String           JSON with every value + the port in use
  /diagnostics   diagnostic_msgs/DiagnosticArray
Service:
  reconnect      std_srvs/Trigger          close the port and search again
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional, Tuple

from .protocols import READERS, BmsSample

# USB-serial chips used in BMS cables (vendor id, product id). These ports are tried first.
KNOWN_CHIPS = {(0x1A86, 0x7523), (0x1A86, 0x5523), (0x1A86, 0x55D4),   # CH340 / CH341 / CH9102
               (0x10C4, 0xEA60),                                        # CP210x
               (0x0403, 0x6001), (0x0403, 0x6015),                      # FTDI
               (0x067B, 0x2303)}                                        # PL2303


class BmsError(RuntimeError):
    """The BMS did not answer, or the port failed."""


# ============================================================================= Settings
@dataclass
class Settings:
    port: str = ''          # '' or 'auto' = detect
    protocol: str = ''      # 'daly', 'jbd', or '' = try both
    baud: int = 9600
    rate: float = 1.0       # readings per second (ROS node)
    timeout: float = 0.6    # seconds to wait for each reply
    env_file: str = ''      # the .env file that was used, for messages

    @classmethod
    def load(cls, env_path: str = '') -> 'Settings':
        """Read the .env file, then let BMS_* environment variables override it."""
        values, path = {}, find_env_file(env_path)
        if path:
            for line in path.read_text(encoding='utf-8-sig').splitlines():
                key, sep, val = line.partition('=')
                if sep and not key.strip().startswith('#'):
                    values[key.strip()] = val.split(' #')[0].strip().strip('"\'')
        values.update({k: v for k, v in os.environ.items() if k.startswith('BMS_')})

        def get(key, kind, default):
            try:
                return kind(values[key]) if values.get(key) else default
            except ValueError:
                return default

        return cls(port=get('BMS_PORT', str, ''), protocol=get('BMS_PROTOCOL', str, '').lower(),
                   baud=get('BMS_BAUD', int, 9600), rate=get('BMS_RATE', float, 1.0),
                   timeout=get('BMS_TIMEOUT', float, 0.6), env_file=str(path or ''))


def find_env_file(explicit: str = '') -> Optional[Path]:
    """First .env found: given path, $BMS_ENV_FILE, current folder, package folder,
    repo root, then the installed ROS share folder (install/bms_ros/share/bms_ros/.env)."""
    here = Path(__file__).resolve()
    places = [explicit, os.environ.get('BMS_ENV_FILE'), Path.cwd() / '.env', here.parents[1] / '.env']
    if len(here.parents) > 3:
        places.append(here.parents[3] / '.env')
    try:
        from ament_index_python.packages import get_package_share_directory
        places.append(Path(get_package_share_directory('bms_ros')) / '.env')
    except Exception:  # not running under ROS
        pass
    return next((Path(p) for p in places if p and Path(p).is_file()), None)


def is_auto(value) -> bool:
    return str(value).strip().lower() in ('', 'auto', '0', '0.0')


# ====================================================================== Serial reading
def open_serial(port: str, baud: int):
    import serial  # pyserial, imported here so the rest works without it (tests)
    return serial.Serial(port, baud, timeout=0.05)


def read_bms(ser, protocol: str, timeout: float, log: Optional[Callable] = None) -> BmsSample:
    """Read one full sample from an open port. Raises BmsError if there is no answer."""
    def ask(request: bytes, done: Callable[[bytes], bool]) -> bytes:
        # send a request, then collect bytes until the reply is complete or time runs out
        ser.reset_input_buffer()
        ser.write(request)
        buf, deadline = bytearray(), time.monotonic() + timeout
        while time.monotonic() < deadline and not done(bytes(buf)):
            buf += ser.read(ser.in_waiting or 1)
        if log:
            log(f'TX {request.hex(" ").upper()}\nRX {bytes(buf).hex(" ").upper() or "(nothing)"}')
        return bytes(buf)

    sample = READERS[protocol](ask)
    if sample is None:
        raise BmsError(f'No reply from a {protocol.upper()} BMS')
    return sample


# ===================================================================== Port detection
def usb_ports() -> List[Tuple[str, str]]:
    """USB serial ports as (name, description), BMS cable chips first.
    Bluetooth and built-in ports are skipped because opening them can hang."""
    from serial.tools import list_ports
    ports = [p for p in list_ports.comports() if p.vid is not None]
    ports.sort(key=lambda p: (p.vid, p.pid) not in KNOWN_CHIPS)
    return [(p.device, p.description) for p in ports]


def try_port(port: str, s: Settings, opener=None) -> Optional[str]:
    """Return the protocol that answers on this port, or None."""
    try:
        ser = (opener or open_serial)(port, s.baud)
    except Exception:                      # missing, or used by another program
        return None
    try:
        for proto in [s.protocol] if s.protocol in READERS else list(READERS):
            try:
                read_bms(ser, proto, min(s.timeout, 0.4))
                return proto
            except BmsError:
                continue
    finally:
        ser.close()
    return None


def find_bms(s: Settings, log: Callable = print, opener=None,
             ports: Optional[List[Tuple[str, str]]] = None) -> Optional[Tuple[str, str]]:
    """Find (port, protocol): the configured port first, then every USB serial port."""
    ports = usb_ports() if ports is None else ports
    if not is_auto(s.port):
        proto = try_port(s.port, s, opener)
        if proto:
            return s.port, proto
        log(f'{s.port}: no BMS answered (missing, busy or asleep), searching other ports')
    for name, desc in ports:
        if name.upper() == s.port.upper():
            continue
        log(f'Trying {name} ({desc})')
        proto = try_port(name, s, opener)
        if proto:
            log(f'Found {proto.upper()} BMS on {name}')
            return name, proto
    return None


class BmsConnection:
    """Keeps one BMS connected: finds it, reads it, and searches again if it drops."""

    def __init__(self, settings: Settings, log: Callable = print, frame_log: Optional[Callable] = None):
        self.s, self.log, self.frame_log = settings, log, frame_log
        self.ser = self.port = self.protocol = None

    def read(self) -> BmsSample:
        if self.ser is None:
            found = find_bms(self.s, self.log)
            if not found:
                raise BmsError('No BMS found on any USB serial port')
            self.port, self.protocol = found
            self.log(f'Connected: {self.protocol.upper()} BMS on {self.port}')
        try:
            self.ser = self.ser or open_serial(self.port, self.s.baud)
            return read_bms(self.ser, self.protocol, self.s.timeout, self.frame_log)
        except Exception as e:             # no answer or cable unplugged: search again next time
            self.close()
            raise BmsError(str(e)) from e

    def close(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None


# ======================================================================== Console mode
def console_main(argv=None):
    """Print readings as JSON. Works without ROS."""
    ap = argparse.ArgumentParser(description='Read a Daly or JBD BMS and print JSON')
    ap.add_argument('--port', help='e.g. COM26 or /dev/ttyUSB0 (default: .env, then auto-detect)')
    ap.add_argument('--protocol', choices=['daly', 'jbd', 'auto'])
    ap.add_argument('--baud', type=int)
    ap.add_argument('--env', default='', help='path to a .env file')
    ap.add_argument('--loop', type=float, default=0, help='repeat every N seconds')
    ap.add_argument('--list', action='store_true', help='list USB serial ports and exit')
    ap.add_argument('--verbose', action='store_true', help='show the raw bytes sent and received')
    a = ap.parse_args(argv)

    if a.list:
        for name, desc in usb_ports():
            print(f'{name}\t{desc}')
        return
    s = Settings.load(a.env)
    s.port = a.port or s.port
    s.protocol = '' if a.protocol == 'auto' else (a.protocol or s.protocol)
    s.baud = a.baud or s.baud
    print(f'port={s.port or "auto"} protocol={s.protocol or "auto"} baud={s.baud} .env={s.env_file or "none"}')

    conn = BmsConnection(s, frame_log=print if a.verbose else None)
    try:
        while True:
            try:
                print(json.dumps({**conn.read().to_dict(), 'port': conn.port}, indent=2))
            except BmsError as e:
                print('error:', e)
            if not a.loop:
                break
            time.sleep(a.loop)
    except KeyboardInterrupt:
        pass
    finally:
        conn.close()


# ============================================================================= ROS 2
def ros_main(args=None):
    """Run as a ROS 2 node."""
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    B = BatteryState

    class BmsNode(Node):
        def __init__(self):
            super().__init__('bms')
            # ROS parameters override the .env file unless left at 'auto' / 0
            param = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
            s = Settings.load(param('env_file', ''))
            for name, kind in (('port', str), ('protocol', str), ('baud', int), ('rate', float), ('timeout', float)):
                value = param(name, kind())
                if not is_auto(value):
                    setattr(s, name, value.lower() if name == 'protocol' else value)
            self.frame_id = param('frame_id', 'battery')
            log = self.get_logger()
            self.conn = BmsConnection(s, log.info, log.info if param('debug_frames', False) else None)
            self.next_search = 0.0

            self.pub_battery = self.create_publisher(B, 'battery_state', qos_profile_sensor_data)
            self.pub_status = self.create_publisher(String, 'status', 10)
            self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
            self.create_service(Trigger, 'reconnect', self.on_reconnect)
            self.create_timer(1.0 / max(s.rate, 0.05), self.tick)
            log.info(f'port={s.port or "auto"} protocol={s.protocol or "auto"} baud={s.baud} '
                     f'rate={s.rate} Hz .env={s.env_file or "none"}')

        def on_reconnect(self, _request, response):
            self.conn.close()
            self.next_search = 0.0
            response.success, response.message = True, 'Searching for the BMS on the next reading'
            return response

        def tick(self):
            """Called every 1/rate seconds: read the BMS and publish."""
            if self.conn.ser is None and time.monotonic() < self.next_search:
                return                                 # wait 5 s between searches
            try:
                s = self.conn.read()
            except BmsError as e:
                self.next_search = time.monotonic() + 5.0
                self.get_logger().warn(str(e), throttle_duration_sec=10.0)
                self.publish_diag(None, str(e))
                return
            self.pub_battery.publish(self.battery_msg(s))
            self.pub_status.publish(String(data=json.dumps({**s.to_dict(), 'port': self.conn.port})))
            self.publish_diag(s)

        def battery_msg(self, s: BmsSample) -> BatteryState:
            """Fill the standard ROS battery message."""
            m = B()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.frame_id
            m.voltage, m.current = float(s.voltage), float(s.current)
            m.charge, m.capacity, m.design_capacity = float(s.remaining_ah), float(s.full_ah), math.nan
            m.percentage = float(s.soc) / 100                        # ROS uses 0..1
            m.temperature = sum(s.temps_c) / len(s.temps_c) if s.temps_c else math.nan
            m.cell_voltage = [mv / 1000 for mv in s.cells_mv]        # ROS uses volts
            m.cell_temperature = [math.nan] * len(s.cells_mv)
            m.present = True
            m.power_supply_status = (B.POWER_SUPPLY_STATUS_CHARGING if s.current > 0.05 else
                                     B.POWER_SUPPLY_STATUS_DISCHARGING if s.current < -0.05 else
                                     B.POWER_SUPPLY_STATUS_FULL if s.soc >= 99 else
                                     B.POWER_SUPPLY_STATUS_NOT_CHARGING)
            faults = ' '.join(s.faults).lower()
            m.power_supply_health = (B.POWER_SUPPLY_HEALTH_GOOD if not faults else
                                     B.POWER_SUPPLY_HEALTH_OVERHEAT if 'over-temp' in faults else
                                     B.POWER_SUPPLY_HEALTH_COLD if 'under-temp' in faults else
                                     B.POWER_SUPPLY_HEALTH_OVERVOLTAGE if 'over-voltage' in faults else
                                     B.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
            if s.cells_mv:                                           # guess chemistry from cell voltage
                m.power_supply_technology = (B.POWER_SUPPLY_TECHNOLOGY_LIFE if max(s.cells_mv) < 3700
                                             else B.POWER_SUPPLY_TECHNOLOGY_LION)
            return m

        def publish_diag(self, s: Optional[BmsSample], error: str = ''):
            """OK / WARN / ERROR summary for rqt_robot_monitor and similar tools."""
            st = DiagnosticStatus(name='bms', hardware_id=str(self.conn.port or 'not found'))
            if s is None:
                st.level, st.message = DiagnosticStatus.ERROR, error
            else:
                delta = max(s.cells_mv) - min(s.cells_mv) if s.cells_mv else 0
                if s.faults:
                    st.level, st.message = DiagnosticStatus.ERROR, ', '.join(s.faults)
                elif delta > 50 or s.charge_fet is False or s.discharge_fet is False:
                    st.level, st.message = DiagnosticStatus.WARN, f'Cell delta {delta} mV, FETs {s.charge_fet}/{s.discharge_fet}'
                else:
                    st.level, st.message = DiagnosticStatus.OK, 'OK'
                st.values = [KeyValue(key=k, value=str(v)) for k, v in (
                    ('voltage_V', s.voltage), ('current_A', s.current), ('soc_pct', s.soc),
                    ('cell_delta_mV', delta), ('temps_C', s.temps_c), ('cycles', s.cycles))]
            msg = DiagnosticArray(status=[st])
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_diag.publish(msg)

    rclpy.init(args=args)
    node = BmsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.conn.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ========================================================================= Entry point
def main(args=None):
    """Console mode if ROS is not installed or console options are given; otherwise ROS node."""
    argv = sys.argv[1:] if args is None else list(args)
    own_args = argv[:argv.index('--ros-args')] if '--ros-args' in argv else argv
    try:
        import rclpy  # noqa: F401
        has_ros = True
    except ImportError:
        has_ros = False
    if own_args or not has_ros:
        console_main(own_args)
    else:
        ros_main(args)


if __name__ == '__main__':
    main()
