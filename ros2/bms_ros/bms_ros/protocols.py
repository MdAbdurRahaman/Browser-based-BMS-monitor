"""
How to talk to a smart BMS (Battery Management System) over a serial cable.

Two protocols are supported:
  Daly  fixed 13-byte frames:  A5 <address> <command> 08 <8 data bytes> <checksum>
  JBD   variable frames:       DD <command> <status> <length> <data...> <checksum 2 bytes> 77
        (JBD = Jiabaida, also sold as Xiaoxiang / Overkill Solar)

This file only builds the request bytes and turns the reply bytes into numbers.
It never opens a port and does not need ROS, so it can be reused anywhere.
Each reader gets an `ask(request, done)` function from the caller: it sends the
request and returns the bytes received until `done(bytes)` is true or time runs out.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Callable, List, Optional

Ask = Callable[[bytes, Callable[[bytes], bool]], bytes]


@dataclass
class BmsSample:
    """One reading of the battery. NaN or None means the BMS did not report it."""
    protocol: str
    voltage: float = math.nan       # pack voltage, V
    current: float = math.nan       # A; positive = charging, negative = discharging
    soc: float = math.nan           # state of charge, %
    remaining_ah: float = math.nan  # charge left, Ah
    full_ah: float = math.nan       # capacity when full, Ah
    cycles: Optional[int] = None    # charge cycles
    cells_mv: List[int] = field(default_factory=list)    # each cell's voltage, mV
    temps_c: List[float] = field(default_factory=list)   # each temperature sensor, °C
    charge_fet: Optional[bool] = None      # True = BMS allows charging
    discharge_fet: Optional[bool] = None   # True = BMS allows discharging
    balancing_mask: int = 0                # bit n set = cell n+1 is balancing (JBD only)
    faults: List[str] = field(default_factory=list)      # active alarms / protections

    def to_dict(self) -> dict:
        """Dict for JSON: NaN becomes null, and power (W) is added."""
        d = {k: None if isinstance(v, float) and math.isnan(v) else v for k, v in asdict(self).items()}
        d['power'] = round(self.voltage * self.current, 2) if None not in (d['voltage'], d['current']) else None
        return d


def u16(data: bytes, i: int) -> int:
    """Two bytes, big-endian, as an unsigned number."""
    return (data[i] << 8) | data[i + 1]


def s16(data: bytes, i: int) -> int:
    """Two bytes, big-endian, as a signed number."""
    v = u16(data, i)
    return v - 0x10000 if v & 0x8000 else v


# =============================================================================== Daly
# Each command returns one 8-byte frame, except cells (0x95) and temperatures (0x96),
# which return several numbered frames.
DALY_SOC, DALY_MOS, DALY_STATUS, DALY_CELLS, DALY_TEMPS, DALY_FAULTS = 0x90, 0x93, 0x94, 0x95, 0x96, 0x98

# Fault names for the 7 bytes of the 0x98 reply, bit 0 first. None = unused bit.
DALY_FAULT_NAMES = [
    ['Cell over-voltage L1', 'Cell over-voltage L2', 'Cell under-voltage L1', 'Cell under-voltage L2',
     'Pack over-voltage L1', 'Pack over-voltage L2', 'Pack under-voltage L1', 'Pack under-voltage L2'],
    ['Charge over-temp L1', 'Charge over-temp L2', 'Charge under-temp L1', 'Charge under-temp L2',
     'Discharge over-temp L1', 'Discharge over-temp L2', 'Discharge under-temp L1', 'Discharge under-temp L2'],
    ['Charge over-current L1', 'Charge over-current L2', 'Discharge over-current L1', 'Discharge over-current L2',
     'SOC high L1', 'SOC high L2', 'SOC low L1', 'SOC low L2'],
    ['Cell imbalance L1', 'Cell imbalance L2', 'Temp difference L1', 'Temp difference L2', None, None, None, None],
    ['Charge MOS over-temp', 'Discharge MOS over-temp', 'Charge MOS sensor fault', 'Discharge MOS sensor fault',
     'Charge MOS adhesion', 'Discharge MOS adhesion', 'Charge MOS open circuit', 'Discharge MOS open circuit'],
    ['AFE chip fault', 'Voltage sensing fault', 'Temp sensor fault', 'EEPROM fault',
     'RTC fault', 'Precharge failed', 'Vehicle comms fault', 'Network comms fault'],
    ['Current sensor fault', 'Voltage sensor fault', 'Short-circuit protection', 'Low-voltage charge blocked',
     None, None, None, None],
]


def daly_request(cmd: int) -> bytes:
    """Read request, e.g. 0x90 -> A5 40 90 08 00 00 00 00 00 00 00 00 7D."""
    frame = [0xA5, 0x40, cmd, 0x08] + [0] * 8
    return bytes(frame + [sum(frame) & 0xFF])


def daly_parse(buf: bytes, cmd: int) -> List[bytes]:
    """All valid replies to `cmd` found in buf, each as its 8 data bytes."""
    frames, i = [], 0
    while i + 13 <= len(buf):
        f = buf[i:i + 13]
        if f[0] == 0xA5 and f[2] == cmd and f[3] == 8 and sum(f[:12]) & 0xFF == f[12]:
            frames.append(bytes(f[4:12]))
            i += 13
        else:
            i += 1
    return frames


def _daly_ask(ask: Ask, cmd: int, frames_needed: int = 1) -> List[bytes]:
    """Send one Daly command and return its reply frames."""
    buf = ask(daly_request(cmd), lambda b: len(daly_parse(b, cmd)) >= frames_needed)
    return daly_parse(buf, cmd)


def _numbered_values(frames: List[bytes], per_frame: int, count: int, value) -> list:
    """Join numbered frames (byte 0 = frame number from 1) into one list of `count` values."""
    found = {}
    for f in frames:
        for k in range(per_frame):
            idx = (f[0] - 1) * per_frame + k
            if 0 <= idx < count:
                found[idx] = value(f, k)
    return [found[i] for i in sorted(found)]


def read_daly(ask: Ask) -> Optional[BmsSample]:
    """Read everything from a Daly BMS. Returns None if it does not answer."""
    soc = _daly_ask(ask, DALY_SOC)
    if not soc:
        return None
    a = soc[0]
    s = BmsSample('daly', voltage=u16(a, 0) / 10,             # 0.1 V units
                  current=(u16(a, 4) - 30000) / 10,           # 0.1 A units, offset 30000
                  soc=u16(a, 6) / 10)                         # 0.1 % units

    mos = _daly_ask(ask, DALY_MOS)                            # FET states + remaining capacity
    if mos:
        m = mos[0]
        s.charge_fet, s.discharge_fet = bool(m[1]), bool(m[2])
        s.remaining_ah = int.from_bytes(m[4:8], 'big') / 1000  # mAh -> Ah
        if s.soc > 0:
            s.full_ah = round(s.remaining_ah * 100 / s.soc, 2)

    status = _daly_ask(ask, DALY_STATUS)                      # cell count, sensor count, cycles
    n_cells, n_temps = (status[0][0], status[0][1]) if status else (0, 0)
    if status:
        s.cycles = u16(status[0], 5)

    if n_cells:                                               # 3 cells per frame, mV
        frames = _daly_ask(ask, DALY_CELLS, -(-n_cells // 3))
        s.cells_mv = _numbered_values(frames, 3, n_cells, lambda f, k: u16(f, 1 + 2 * k))
    if n_temps:                                               # 7 sensors per frame, °C + 40
        frames = _daly_ask(ask, DALY_TEMPS, -(-n_temps // 7))
        s.temps_c = _numbered_values(frames, 7, n_temps, lambda f, k: float(f[1 + k] - 40))

    faults = _daly_ask(ask, DALY_FAULTS)                      # one bit per fault
    if faults:
        f = faults[0]
        s.faults = [name for byte, names in enumerate(DALY_FAULT_NAMES)
                    for bit, name in enumerate(names) if name and f[byte] & (1 << bit)]
    return s


# ================================================================================ JBD
JBD_BASIC, JBD_CELLS = 0x03, 0x04

# Protection flags in the basic-info reply, bit 0 first
JBD_FAULT_NAMES = [
    'Cell over-voltage', 'Cell under-voltage', 'Pack over-voltage', 'Pack under-voltage',
    'Charge over-temp', 'Charge under-temp', 'Discharge over-temp', 'Discharge under-temp',
    'Charge over-current', 'Discharge over-current', 'Short circuit', 'Front-end IC error',
    'MOS locked by software',
]


def jbd_request(cmd: int) -> bytes:
    """Read request, e.g. 0x03 -> DD A5 03 00 FF FD 77."""
    chk = (0x10000 - cmd) & 0xFFFF
    return bytes([0xDD, 0xA5, cmd, 0x00, chk >> 8, chk & 0xFF, 0x77])


def jbd_parse(buf: bytes, cmd: int) -> Optional[bytes]:
    """Data bytes of the first valid, error-free reply to `cmd` in buf, or None."""
    for i in range(len(buf) - 6):
        if buf[i] != 0xDD or buf[i + 1] != cmd or buf[i + 2] != 0:   # start, command, status OK
            continue
        n = buf[i + 3]
        end = i + 7 + n
        if end > len(buf) or buf[end - 1] != 0x77:
            continue
        if (0x10000 - sum(buf[i + 2:i + 4 + n])) & 0xFFFF == u16(buf, end - 3):
            return bytes(buf[i + 4:i + 4 + n])
    return None


def read_jbd(ask: Ask) -> Optional[BmsSample]:
    """Read everything from a JBD BMS. Returns None if it does not answer."""
    d = jbd_parse(ask(jbd_request(JBD_BASIC), lambda b: jbd_parse(b, JBD_BASIC) is not None), JBD_BASIC)
    if d is None:
        return None
    protection = u16(d, 16)
    s = BmsSample('jbd',
                  voltage=u16(d, 0) / 100,          # 10 mV units
                  current=s16(d, 2) / 100,          # 10 mA units, signed
                  remaining_ah=u16(d, 4) / 100,     # 10 mAh units
                  full_ah=u16(d, 6) / 100,
                  cycles=u16(d, 8),
                  balancing_mask=(u16(d, 14) << 16) | u16(d, 12),
                  faults=[name for bit, name in enumerate(JBD_FAULT_NAMES) if protection & (1 << bit)],
                  soc=float(d[19]),
                  charge_fet=bool(d[20] & 1),
                  discharge_fet=bool(d[20] & 2),
                  temps_c=[round((u16(d, 23 + 2 * k) - 2731) / 10, 1) for k in range(d[22])])  # 0.1 K
    cells = jbd_parse(ask(jbd_request(JBD_CELLS), lambda b: jbd_parse(b, JBD_CELLS) is not None), JBD_CELLS)
    if cells:
        s.cells_mv = [u16(cells, k) for k in range(0, len(cells) - 1, 2)]
    return s


# Protocol name -> reader. Detection tries them in this order.
READERS = {'daly': read_daly, 'jbd': read_jbd}
