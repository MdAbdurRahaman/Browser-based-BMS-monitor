"""BMS serial protocols (JBD / Xiaoxiang and Daly UART).

Pure Python, no ROS and no pyserial imports, so this module can be reused in any
application. Every function works on bytes / lists of ints.
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import List, Optional


@dataclass
class BmsSample:
    protocol: str
    voltage: float = float('nan')          # V
    current: float = float('nan')          # A, + charging, - discharging
    soc: float = float('nan')              # percent 0..100
    remaining_ah: float = float('nan')
    full_ah: float = float('nan')
    cycles: Optional[int] = None
    cell_count: int = 0
    cells_mv: List[int] = field(default_factory=list)
    temps_c: List[float] = field(default_factory=list)
    charge_fet: Optional[bool] = None
    discharge_fet: Optional[bool] = None
    balancing_mask: int = 0
    faults: List[str] = field(default_factory=list)

    @property
    def power(self) -> float:
        return self.voltage * self.current

    def to_dict(self) -> dict:
        d = asdict(self)
        d['power'] = self.power
        # NaN is not valid JSON; turn it into None
        return {k: (None if isinstance(v, float) and v != v else v) for k, v in d.items()}


def _u16(d, o):
    return (d[o] << 8) | d[o + 1]


def _s16(d, o):
    v = _u16(d, o)
    return v - 0x10000 if v > 0x7FFF else v


# --------------------------------------------------------------------------- JBD
JBD_BASIC = 0x03
JBD_CELLS = 0x04
JBD_FAULTS = [
    'Cell over-voltage', 'Cell under-voltage', 'Pack over-voltage', 'Pack under-voltage',
    'Charge over-temp', 'Charge under-temp', 'Discharge over-temp', 'Discharge under-temp',
    'Charge over-current', 'Discharge over-current', 'Short circuit', 'Front-end IC error',
    'MOS locked by software',
]


def jbd_request(cmd: int) -> bytes:
    """Read request: DD A5 <cmd> 00 <chk_hi> <chk_lo> 77."""
    chk = (0x10000 - (cmd + 0x00)) & 0xFFFF
    return bytes([0xDD, 0xA5, cmd, 0x00, chk >> 8, chk & 0xFF, 0x77])


def jbd_find_frame(buf: bytes):
    """Return (cmd, status, data) for the first valid frame in buf, else None."""
    for i in range(len(buf)):
        if buf[i] != 0xDD or i + 4 > len(buf):
            continue
        ln = buf[i + 3]
        end = i + 4 + ln + 3
        if end > len(buf):
            return None  # incomplete, wait for more bytes
        if buf[end - 1] != 0x77:
            continue
        body = buf[i + 2:i + 4 + ln]
        chk = (buf[end - 3] << 8) | buf[end - 2]
        if (0x10000 - sum(body)) & 0xFFFF != chk:
            continue
        return buf[i + 1], buf[i + 2], bytes(buf[i + 4:i + 4 + ln])
    return None


def jbd_decode_basic(d: bytes) -> BmsSample:
    ntc = d[22]
    prot = _u16(d, 16)
    return BmsSample(
        protocol='jbd',
        voltage=_u16(d, 0) / 100.0,
        current=_s16(d, 2) / 100.0,
        remaining_ah=_u16(d, 4) / 100.0,
        full_ah=_u16(d, 6) / 100.0,
        cycles=_u16(d, 8),
        balancing_mask=(_u16(d, 14) << 16) | _u16(d, 12),
        faults=[name for bit, name in enumerate(JBD_FAULTS) if prot & (1 << bit)],
        soc=float(d[19]),
        charge_fet=bool(d[20] & 1),
        discharge_fet=bool(d[20] & 2),
        cell_count=d[21],
        temps_c=[round((_u16(d, 23 + 2 * k) - 2731) / 10.0, 1) for k in range(ntc)],
    )


def jbd_decode_cells(d: bytes) -> List[int]:
    return [_u16(d, k) for k in range(0, len(d) - 1, 2)]


# -------------------------------------------------------------------------- Daly
DALY_SOC, DALY_MINMAX_T, DALY_MOS, DALY_STATUS, DALY_CELLS, DALY_TEMPS, DALY_FAULT = (
    0x90, 0x92, 0x93, 0x94, 0x95, 0x96, 0x98)

_DALY_FAULT_BITS = [
    'Cell over-voltage L1', 'Cell over-voltage L2', 'Cell under-voltage L1', 'Cell under-voltage L2',
    'Pack over-voltage L1', 'Pack over-voltage L2', 'Pack under-voltage L1', 'Pack under-voltage L2',
    'Charge over-temp L1', 'Charge over-temp L2', 'Charge under-temp L1', 'Charge under-temp L2',
    'Discharge over-temp L1', 'Discharge over-temp L2', 'Discharge under-temp L1', 'Discharge under-temp L2',
    'Charge over-current L1', 'Charge over-current L2', 'Discharge over-current L1', 'Discharge over-current L2',
    'SOC high L1', 'SOC high L2', 'SOC low L1', 'SOC low L2',
    'Cell imbalance L1', 'Cell imbalance L2', 'Temp difference L1', 'Temp difference L2', None, None, None, None,
    'Charge MOS over-temp', 'Discharge MOS over-temp', 'Charge MOS sensor fault', 'Discharge MOS sensor fault',
    'Charge MOS adhesion', 'Discharge MOS adhesion', 'Charge MOS open circuit', 'Discharge MOS open circuit',
    'AFE chip fault', 'Voltage sensing fault', 'Temp sensor fault', 'EEPROM fault',
    'RTC fault', 'Precharge failed', 'Vehicle comms fault', 'Network comms fault',
    'Current sensor fault', 'Voltage sensor fault', 'Short-circuit protection', 'Low-voltage charge blocked',
]


def daly_request(cmd: int, address: int = 0x40) -> bytes:
    f = [0xA5, address, cmd, 0x08] + [0] * 8
    f.append(sum(f) & 0xFF)
    return bytes(f)


def daly_find_frames(buf: bytes, cmd: int) -> List[bytes]:
    """Return the 8-byte data field of every valid response frame for cmd."""
    out, i = [], 0
    while i + 13 <= len(buf):
        if buf[i] == 0xA5 and buf[i + 2] == cmd and buf[i + 3] == 0x08 and sum(buf[i:i + 12]) & 0xFF == buf[i + 12]:
            out.append(bytes(buf[i + 4:i + 12]))
            i += 13
        else:
            i += 1
    return out


def daly_faults(d: bytes) -> List[str]:
    names = []
    for byte in range(min(7, len(d))):
        for bit in range(8):
            idx = byte * 8 + bit
            if idx < len(_DALY_FAULT_BITS) and _DALY_FAULT_BITS[idx] and d[byte] & (1 << bit):
                names.append(_DALY_FAULT_BITS[idx])
    return names


def daly_cells_from_frames(frames: List[bytes], n: int) -> List[int]:
    cells = [None] * n
    for f in frames:
        for k in range(3):
            idx = (f[0] - 1) * 3 + k
            if 0 <= idx < n:
                cells[idx] = _u16(f, 1 + 2 * k)
    return [c for c in cells if c is not None]


def daly_temps_from_frames(frames: List[bytes], n: int) -> List[float]:
    temps = [None] * n
    for f in frames:
        for k in range(7):
            idx = (f[0] - 1) * 7 + k
            if 0 <= idx < n:
                temps[idx] = float(f[1 + k] - 40)
    return [t for t in temps if t is not None]
