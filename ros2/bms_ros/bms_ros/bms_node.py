"""BMS reader and ROS 2 node for JBD / Xiaoxiang and Daly smart BMS boards.

One file, two ways to run it:

  ROS 2 node      ros2 run bms_ros bms_node            (or the launch file)
  Plain console   python -m bms_ros.bms_node --loop 2  (Windows, or anywhere without ROS)

Settings come from, highest priority first:
  1. command-line arguments / ROS parameters
  2. environment variables (BMS_PORT, BMS_PROTOCOL, ...)
  3. a .env file (see .env.example)
  4. automatic detection: every USB serial adapter is tried with each protocol
     until one answers like a BMS.

If the configured port is missing (unplugged, or Windows gave it a new COM
number) or does not answer, the reader falls back to automatic detection, and it
re-detects whenever the connection drops.

Published topics (ROS mode, relative to the node namespace):
  battery_state  sensor_msgs/BatteryState
  status         std_msgs/String (JSON with every decoded field + port/protocol)
  /diagnostics   diagnostic_msgs/DiagnosticArray
Service:
  reconnect      std_srvs/Trigger  (close the port and detect again)
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
from typing import Callable, Dict, List, Optional, Tuple

from . import protocols as p

PROTOCOLS = ('daly', 'jbd')
DEFAULT_BAUD = 9600

# USB-serial chips commonly used with BMS cables, tried first during detection
KNOWN_ADAPTERS = {
    (0x1A86, 0x7523): 'CH340', (0x1A86, 0x5523): 'CH341', (0x1A86, 0x55D4): 'CH9102',
    (0x10C4, 0xEA60): 'CP210x', (0x0403, 0x6001): 'FT232R', (0x0403, 0x6015): 'FT231X',
    (0x0403, 0x6010): 'FT2232', (0x067B, 0x2303): 'PL2303',
}


# ============================================================================ .env
def _parse_env_file(path: Path) -> Dict[str, str]:
    out = {}
    for raw in path.read_text(encoding='utf-8-sig').splitlines():
        line = raw.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        if line.startswith('export '):
            line = line[7:]
        key, val = line.split('=', 1)
        val = val.strip()
        if val[:1] in ('"', "'") and val[-1:] == val[:1]:
            val = val[1:-1]
        elif ' #' in val:
            val = val.split(' #', 1)[0].rstrip()
        out[key.strip()] = val
    return out


def find_env_file(explicit: Optional[str] = None) -> Optional[Path]:
    """First existing .env among: explicit path, $BMS_ENV_FILE, ./.env,
    the package folder, and the repository root."""
    here = Path(__file__).resolve()
    candidates = [explicit, os.environ.get('BMS_ENV_FILE'), Path.cwd() / '.env',
                  here.parent.parent / '.env']
    if len(here.parents) > 3:
        candidates.append(here.parents[3] / '.env')
    for c in candidates:
        if c and Path(c).is_file():
            return Path(c)
    return None


@dataclass
class Settings:
    port: str = ''              # '' or 'auto' = detect
    protocol: str = ''          # '' or 'auto' = detect
    baud: int = DEFAULT_BAUD
    rate: float = 1.0
    timeout: float = 0.6
    env_file: Optional[str] = None

    @classmethod
    def load(cls, env_path: Optional[str] = None) -> 'Settings':
        path = find_env_file(env_path)
        values = _parse_env_file(path) if path else {}
        # real environment variables win over the .env file
        values.update({k: v for k, v in os.environ.items() if k.startswith('BMS_')})

        def get(key, conv, default):
            v = values.get(key, '').strip()
            if not v:
                return default
            try:
                return conv(v)
            except ValueError:
                return default

        return cls(port=get('BMS_PORT', str, ''),
                   protocol=get('BMS_PROTOCOL', str.lower, ''),
                   baud=get('BMS_BAUD', int, DEFAULT_BAUD),
                   rate=get('BMS_RATE', float, 1.0),
                   timeout=get('BMS_TIMEOUT', float, 0.6),
                   env_file=str(path) if path else None)


def _is_auto(v) -> bool:
    return not v or str(v).strip().lower() in ('auto', 'none', '')


# ========================================================================= client
class BmsError(RuntimeError):
    pass


class BmsClient:
    """Polls a BMS through any pyserial-like object (write, read, in_waiting)."""

    def __init__(self, port, protocol: str = 'daly', timeout: float = 0.6,
                 log: Optional[Callable[[str], None]] = None):
        if protocol not in PROTOCOLS:
            raise ValueError("protocol must be 'jbd' or 'daly'")
        self.port = port
        self.protocol = protocol
        self.timeout = timeout
        self.log = log or (lambda _m: None)

    def _transact(self, request: bytes, done, timeout: Optional[float] = None):
        try:
            self.port.reset_input_buffer()
        except Exception:
            pass
        self.log('TX ' + request.hex(' ').upper())
        self.port.write(request)
        buf = bytearray()
        deadline = time.monotonic() + (timeout or self.timeout)
        result = None
        while time.monotonic() < deadline:
            chunk = self.port.read(max(1, getattr(self.port, 'in_waiting', 0) or 1))
            if chunk:
                buf += chunk
                result = done(bytes(buf), False)
                if result:
                    break
        if not result:
            result = done(bytes(buf), True)
        if buf:
            self.log('RX ' + bytes(buf).hex(' ').upper())
        return result

    def read(self) -> p.BmsSample:
        return self._read_jbd() if self.protocol == 'jbd' else self._read_daly()

    def _read_jbd(self) -> p.BmsSample:
        def want(cmd):
            def f(buf, _final):
                fr = p.jbd_find_frame(buf)
                return fr if fr and fr[0] == cmd else None
            return f

        fr = self._transact(p.jbd_request(p.JBD_BASIC), want(p.JBD_BASIC))
        if not fr or fr[1] != 0:
            raise BmsError('No valid reply to JBD basic info request (0x03)')
        s = p.jbd_decode_basic(fr[2])
        fr = self._transact(p.jbd_request(p.JBD_CELLS), want(p.JBD_CELLS))
        if fr and fr[1] == 0:
            s.cells_mv = p.jbd_decode_cells(fr[2])
        return s

    def _read_daly(self) -> p.BmsSample:
        def one(cmd):
            def f(buf, _final):
                fr = p.daly_find_frames(buf, cmd)
                return fr[0] if fr else None
            return f

        def many(cmd, need):
            def f(buf, final):
                fr = p.daly_find_frames(buf, cmd)
                return fr if fr and (len(fr) >= need or final) else None
            return f

        a = self._transact(p.daly_request(p.DALY_SOC), one(p.DALY_SOC))
        if not a:
            raise BmsError('No valid reply to Daly SOC request (0x90)')
        s = p.BmsSample(protocol='daly',
                        voltage=p._u16(a, 0) / 10.0,
                        current=(p._u16(a, 4) - 30000) / 10.0,
                        soc=p._u16(a, 6) / 10.0)

        m = self._transact(p.daly_request(p.DALY_MOS), one(p.DALY_MOS))
        if m:
            s.charge_fet, s.discharge_fet = bool(m[1]), bool(m[2])
            s.remaining_ah = ((m[4] << 24) | (m[5] << 16) | (m[6] << 8) | m[7]) / 1000.0
            if s.soc > 0:
                s.full_ah = round(s.remaining_ah / (s.soc / 100.0), 2)

        st = self._transact(p.daly_request(p.DALY_STATUS), one(p.DALY_STATUS))
        n_temp = 0
        if st:
            s.cell_count, n_temp, s.cycles = st[0], st[1], p._u16(st, 5)

        if s.cell_count:
            need = -(-s.cell_count // 3)
            fr = self._transact(p.daly_request(p.DALY_CELLS), many(p.DALY_CELLS, need), timeout=1.2)
            if fr:
                s.cells_mv = p.daly_cells_from_frames(fr, s.cell_count)

        if n_temp:
            fr = self._transact(p.daly_request(p.DALY_TEMPS), many(p.DALY_TEMPS, -(-n_temp // 7)))
            if fr:
                s.temps_c = p.daly_temps_from_frames(fr, n_temp)
        if not s.temps_c:
            t = self._transact(p.daly_request(p.DALY_MINMAX_T), one(p.DALY_MINMAX_T))
            if t:
                s.temps_c = [float(t[0] - 40), float(t[2] - 40)]

        e = self._transact(p.daly_request(p.DALY_FAULT), one(p.DALY_FAULT))
        if e:
            s.faults = p.daly_faults(e)
        return s


# ====================================================================== detection
def list_candidate_ports() -> List[Tuple[str, str]]:
    """USB serial ports as (device, description), known BMS adapters first.
    Bluetooth and built-in ports are skipped: opening them can hang for seconds."""
    from serial.tools import list_ports
    ports = []
    for info in list_ports.comports():
        if info.vid is None:
            continue
        chip = KNOWN_ADAPTERS.get((info.vid, info.pid))
        ports.append((0 if chip else 1, info.device, info.description or chip or ''))
    ports.sort()
    return [(dev, desc) for _rank, dev, desc in ports]


def port_exists(name: str) -> bool:
    if os.path.exists(name):          # /dev/ttyUSB0, /dev/serial/by-id/...
        return True
    from serial.tools import list_ports
    return any(info.device.upper() == name.upper() for info in list_ports.comports())


def open_serial(port: str, baud: int):
    import serial
    return serial.Serial(port, baud, timeout=0.05)


def probe_port(port: str, protocols, baud: int, timeout: float,
               opener=open_serial) -> Optional[Tuple[str, p.BmsSample]]:
    """Return (protocol, first sample) if a BMS answers on this port."""
    try:
        ser = opener(port, baud)
    except Exception:
        return None
    try:
        for proto in protocols:
            try:
                return proto, BmsClient(ser, proto, timeout=min(timeout, 0.4)).read()
            except BmsError:
                continue
            except Exception:
                return None
        return None
    finally:
        try:
            ser.close()
        except Exception:
            pass


def find_bms(settings: Settings, log: Callable[[str], None] = print,
             opener=open_serial, candidates=None) -> Optional[Tuple[str, str]]:
    """Return (port, protocol) of the first port with a BMS that answers."""
    protos = [settings.protocol] if settings.protocol in PROTOCOLS else list(PROTOCOLS)
    tried = set()
    if not _is_auto(settings.port):
        if candidates is not None or port_exists(settings.port):
            r = probe_port(settings.port, protos, settings.baud, settings.timeout, opener)
            if r:
                return settings.port, r[0]
            log(f'{settings.port}: no BMS reply (busy, asleep, or wrong protocol); scanning other ports')
        else:
            log(f'{settings.port} not found; scanning USB serial ports')
        tried.add(settings.port.upper())
    ports = candidates if candidates is not None else list_candidate_ports()
    for dev, desc in ports:
        if dev.upper() in tried:
            continue
        log(f'Trying {dev} {("(" + desc + ")") if desc else ""}')
        r = probe_port(dev, protos, settings.baud, settings.timeout, opener)
        if r:
            log(f'Found {r[0].upper()} BMS on {dev}')
            return dev, r[0]
    return None


class BmsConnection:
    """Keeps a BMS connection alive: detects the port, reads, re-detects on failure."""

    def __init__(self, settings: Settings, log: Callable[[str], None] = print,
                 frame_log: Optional[Callable[[str], None]] = None):
        self.settings = settings
        self.log = log
        self.frame_log = frame_log
        self.serial = None
        self.client: Optional[BmsClient] = None
        self.port: Optional[str] = None
        self.protocol: Optional[str] = None

    @property
    def connected(self) -> bool:
        return self.client is not None

    def connect(self) -> bool:
        self.close()
        found = find_bms(self.settings, self.log)
        if not found:
            return False
        self.port, self.protocol = found
        try:
            self.serial = open_serial(self.port, self.settings.baud)
        except Exception as e:
            self.log(f'Cannot open {self.port}: {e}')
            return False
        self.client = BmsClient(self.serial, self.protocol, self.settings.timeout, self.frame_log)
        self.log(f'Connected: {self.protocol.upper()} BMS on {self.port} @ {self.settings.baud} baud')
        return True

    def read(self) -> p.BmsSample:
        if not self.client and not self.connect():
            raise BmsError('No BMS found on any USB serial port')
        try:
            return self.client.read()
        except BmsError:
            raise
        except Exception as e:        # unplugged, driver error
            self.close()
            raise BmsError(f'Serial error on {self.port}: {e}')

    def close(self):
        try:
            if self.serial:
                self.serial.close()
        except Exception:
            pass
        self.serial = None
        self.client = None


# ======================================================================= console
def cli_main(argv=None):
    ap = argparse.ArgumentParser(description='Read a Daly or JBD BMS over serial and print JSON')
    ap.add_argument('--port', help='e.g. COM26 or /dev/ttyUSB0 (default: .env, then auto-detect)')
    ap.add_argument('--protocol', choices=['daly', 'jbd', 'auto'])
    ap.add_argument('--baud', type=int)
    ap.add_argument('--loop', type=float, default=0, help='repeat every N seconds')
    ap.add_argument('--env', help='path to a .env file')
    ap.add_argument('--list', action='store_true', help='list USB serial ports and exit')
    ap.add_argument('--verbose', action='store_true', help='print raw TX/RX frames')
    ap.add_argument('--probe', action='store_true', help=argparse.SUPPRESS)
    args = ap.parse_args(argv)

    if args.list:
        for dev, desc in list_candidate_ports():
            print(f'{dev}\t{desc}')
        return

    s = Settings.load(args.env)
    if args.port:
        s.port = args.port
    if args.protocol:
        s.protocol = args.protocol
    if args.baud:
        s.baud = args.baud
    print(f'Settings: port={s.port or "auto"} protocol={s.protocol or "auto"} baud={s.baud} '
          f'(.env: {s.env_file or "none found"})')

    conn = BmsConnection(s, log=print, frame_log=print if args.verbose else None)
    try:
        while True:
            try:
                d = conn.read().to_dict()
                d.update(port=conn.port)
                print(json.dumps(d, indent=2))
            except BmsError as e:
                print('error:', e)
                if args.loop:
                    conn.close()
            if not args.loop:
                break
            time.sleep(args.loop)
    except KeyboardInterrupt:
        pass
    finally:
        conn.close()


# =========================================================================== ROS 2
def ros_main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    tech_map = {
        'unknown': BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN,
        'lion': BatteryState.POWER_SUPPLY_TECHNOLOGY_LION,
        'lipo': BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
        'lifepo4': BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE,
    }

    class BmsNode(Node):
        def __init__(self):
            super().__init__('bms')
            P = self.declare_parameter
            env_file = P('env_file', '').value
            s = Settings.load(env_file or None)
            # a ROS parameter overrides .env only when it is set (not '' / 'auto' / 0)
            port, proto, baud = P('port', '').value, P('protocol', '').value, P('baud', 0).value
            rate, timeout = P('rate', 0.0).value, P('timeout', 0.0).value
            if not _is_auto(port):
                s.port = port
            if not _is_auto(proto):
                s.protocol = proto.lower()
            if baud:
                s.baud = int(baud)
            if rate:
                s.rate = float(rate)
            if timeout:
                s.timeout = float(timeout)
            self.settings = s

            self.frame_id = P('frame_id', 'battery').value
            self.location = P('location', '').value
            self.serial_number = P('serial_number', '').value
            self.design_capacity = P('design_capacity', 0.0).value
            self.technology = P('technology', 'auto').value
            self.full_threshold = P('full_soc_threshold', 99.0).value
            self.idle_current = P('idle_current', 0.05).value
            self.retry_interval = P('retry_interval', 5.0).value
            debug = P('debug_frames', False).value

            log = self.get_logger()
            self.conn = BmsConnection(s, log=log.info, frame_log=log.info if debug else None)
            self.next_retry = 0.0

            self.pub_batt = self.create_publisher(BatteryState, 'battery_state', qos_profile_sensor_data)
            self.pub_status = self.create_publisher(String, 'status', 10)
            self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
            self.create_service(Trigger, 'reconnect', self._on_reconnect)
            self.create_timer(1.0 / max(s.rate, 0.05), self._tick)
            log.info(f'BMS node: port={s.port or "auto"} protocol={s.protocol or "auto"} '
                     f'baud={s.baud} rate={s.rate} Hz (.env: {s.env_file or "none found"})')

        def _on_reconnect(self, _req, res):
            res.success = self.conn.connect()
            res.message = (f'Connected to {self.conn.protocol} BMS on {self.conn.port}'
                           if res.success else 'No BMS found')
            return res

        def _tick(self):
            if not self.conn.connected:
                now = time.monotonic()
                if now < self.next_retry:
                    return
                if not self.conn.connect():
                    self.next_retry = now + self.retry_interval
                    self._publish_diag(None, 'No BMS found on any USB serial port')
                    return
            try:
                s = self.conn.read()
            except BmsError as e:
                self.get_logger().warn(str(e), throttle_duration_sec=5.0)
                self._publish_diag(None, str(e))
                return
            self._publish_battery(s)
            d = s.to_dict()
            d.update(port=self.conn.port)
            self.pub_status.publish(String(data=json.dumps(d)))
            self._publish_diag(s, None)

        def _publish_battery(self, s):
            m = BatteryState()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.frame_id
            m.voltage = float(s.voltage)
            m.current = float(s.current)  # ROS convention: negative when discharging
            m.temperature = float(sum(s.temps_c) / len(s.temps_c)) if s.temps_c else math.nan
            m.charge = float(s.remaining_ah)
            m.capacity = float(s.full_ah)
            m.design_capacity = float(self.design_capacity) if self.design_capacity > 0 else math.nan
            m.percentage = float(s.soc) / 100.0 if s.soc == s.soc else math.nan
            m.present = True
            m.cell_voltage = [v / 1000.0 for v in s.cells_mv]
            m.cell_temperature = [math.nan] * len(s.cells_mv)
            m.location = self.location
            m.serial_number = self.serial_number

            if s.soc >= self.full_threshold and abs(s.current) <= self.idle_current:
                m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            elif s.current > self.idle_current:
                m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            elif s.current < -self.idle_current:
                m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            else:
                m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

            text = ' '.join(s.faults).lower()
            if not s.faults:
                m.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            elif 'over-temp' in text:
                m.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT
            elif 'under-temp' in text:
                m.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_COLD
            elif 'over-voltage' in text:
                m.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
            else:
                m.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE

            tech = self.technology
            if tech == 'auto' and s.cells_mv:
                tech = 'lifepo4' if max(s.cells_mv) < 3700 else 'lion'
            m.power_supply_technology = tech_map.get(tech, BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
            self.pub_batt.publish(m)

        def _publish_diag(self, s, error):
            st = DiagnosticStatus(name=f'{self.get_name()}: BMS',
                                  hardware_id=self.serial_number or (self.conn.port or 'auto'))
            kv = [KeyValue(key='port', value=str(self.conn.port or self.settings.port or 'auto')),
                  KeyValue(key='protocol', value=str(self.conn.protocol or self.settings.protocol or 'auto'))]
            if s is None:
                st.level, st.message = DiagnosticStatus.ERROR, error or 'No data'
            else:
                cells = s.cells_mv
                delta = (max(cells) - min(cells)) if cells else 0
                fet_off = s.charge_fet is False or s.discharge_fet is False
                if s.faults:
                    st.level, st.message = DiagnosticStatus.ERROR, ', '.join(s.faults)
                elif fet_off or delta > 50:
                    st.level = DiagnosticStatus.WARN
                    st.message = 'FET off' if fet_off else f'Cell delta {delta} mV'
                else:
                    st.level, st.message = DiagnosticStatus.OK, 'OK'
                kv += [
                    KeyValue(key='voltage_V', value=f'{s.voltage:.2f}'),
                    KeyValue(key='current_A', value=f'{s.current:.2f}'),
                    KeyValue(key='soc_pct', value=f'{s.soc:.1f}'),
                    KeyValue(key='cycles', value=str(s.cycles)),
                    KeyValue(key='charge_fet', value=str(s.charge_fet)),
                    KeyValue(key='discharge_fet', value=str(s.discharge_fet)),
                    KeyValue(key='cell_min_mV', value=str(min(cells)) if cells else '-'),
                    KeyValue(key='cell_max_mV', value=str(max(cells)) if cells else '-'),
                    KeyValue(key='cell_delta_mV', value=str(delta)),
                    KeyValue(key='temps_C', value=', '.join(f'{t:.1f}' for t in s.temps_c)),
                ]
            st.values = kv
            arr = DiagnosticArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.status = [st]
            self.pub_diag.publish(arr)

        def destroy_node(self):
            self.conn.close()
            super().destroy_node()

    rclpy.init(args=args)
    node = BmsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    """ROS 2 node when rclpy is available; console reader otherwise or with --probe/--list/--loop."""
    argv = sys.argv[1:] if args is None else args
    console_flags = {'--probe', '--list', '--loop', '--verbose', '--port', '--protocol', '--env', '--baud'}
    wants_console = any(a.split('=')[0] in console_flags for a in argv)
    if not wants_console:
        try:
            import rclpy  # noqa: F401
        except ImportError:
            wants_console = True
    if wants_console:
        cli_main([a for a in argv if a != '--probe'])
    else:
        ros_main(args)


def probe_main():
    cli_main()


if __name__ == '__main__':
    main()
