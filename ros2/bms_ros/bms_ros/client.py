"""Serial client that polls a BMS and returns BmsSample objects.

Works with any object that has pyserial's interface (write, read, in_waiting,
reset_input_buffer), so it can be used outside ROS too:

    import serial
    from bms_ros.client import BmsClient
    port = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.05)
    print(BmsClient(port, 'jbd').read().to_dict())

Run `ros2 run bms_ros bms_probe --port /dev/ttyUSB0` for a quick check.
"""
from __future__ import annotations

import time
from typing import Callable, Optional

from . import protocols as p


class BmsError(RuntimeError):
    pass


class BmsClient:
    def __init__(self, port, protocol: str = 'jbd', timeout: float = 0.6,
                 log: Optional[Callable[[str], None]] = None):
        if protocol not in ('jbd', 'daly'):
            raise ValueError("protocol must be 'jbd' or 'daly'")
        self.port = port
        self.protocol = protocol
        self.timeout = timeout
        self.log = log or (lambda _m: None)

    # ------------------------------------------------------------------ io
    def _transact(self, request: bytes, done: Callable[[bytes], object], timeout: Optional[float] = None):
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

    # ----------------------------------------------------------------- read
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


def main(argv=None):
    """bms_probe: read the BMS once (or continuously) and print JSON. No ROS needed."""
    import argparse
    import json
    import serial  # pyserial

    ap = argparse.ArgumentParser(description='Read a JBD or Daly BMS over serial and print JSON')
    ap.add_argument('--port', default='/dev/ttyUSB0', help='e.g. /dev/ttyUSB0 or COM19')
    ap.add_argument('--baud', type=int, default=9600)
    ap.add_argument('--protocol', choices=['jbd', 'daly'], default='jbd')
    ap.add_argument('--loop', type=float, default=0, help='repeat every N seconds')
    ap.add_argument('--verbose', action='store_true', help='print raw TX/RX frames')
    args = ap.parse_args(argv)

    with serial.Serial(args.port, args.baud, timeout=0.05) as port:
        client = BmsClient(port, args.protocol, log=print if args.verbose else None)
        while True:
            try:
                print(json.dumps(client.read().to_dict(), indent=2))
            except BmsError as e:
                print('error:', e)
            if not args.loop:
                break
            time.sleep(args.loop)


if __name__ == '__main__':
    main()
