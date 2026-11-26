"""Tests against a simulated BMS, so no hardware or ROS is needed:  python -m pytest test/"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bms_ros import protocols as p  # noqa: E402
from bms_ros.bms_node import BmsError, Settings, find_bms, read_bms  # noqa: E402


class FakeSerial:
    """Pretends to be a BMS: answers each known request with a canned reply, a few bytes at a time."""

    def __init__(self, replies):
        self.replies, self.rx = replies, bytearray()

    def reset_input_buffer(self):
        self.rx.clear()

    def write(self, data):
        self.rx += self.replies.get(bytes(data), b'')

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n=1):
        out = bytes(self.rx[:min(n, 5)])
        del self.rx[:len(out)]
        return out

    def close(self):
        pass


def be16(v):
    return (v & 0xFFFF).to_bytes(2, 'big')


def jbd_reply(cmd, data):
    body = bytes([0, len(data)]) + data
    chk = (0x10000 - sum(body)) & 0xFFFF
    return bytes([0xDD, cmd]) + body + be16(chk) + b'\x77'


# Real Daly replies captured from a 6S pack (COM26)
REAL_DALY = {
    0x90: 'A5 01 90 08 00 E4 00 00 75 30 01 B6 7E',
    0x93: 'A5 01 93 08 00 01 01 15 00 00 2F E8 6F',
    0x94: 'A5 01 94 08 06 02 00 00 00 00 04 47 95',
    0x95: 'A5 01 95 08 01 0E F6 0F AC 0D 30 00 40 A5 01 95 08 02 0E DA 0F 42 0F 6A 00 F7',
    0x96: 'A5 01 96 08 01 40 42 FF FF FF FF 00 C3',
    0x98: 'A5 01 98 08 00 00 00 01 00 00 00 00 47',
}
DALY_PORT = {p.daly_request(cmd): bytes.fromhex(hx) for cmd, hx in REAL_DALY.items()}


def test_request_frames():
    assert p.daly_request(0x90).hex(' ').upper() == 'A5 40 90 08 00 00 00 00 00 00 00 00 7D'
    assert p.jbd_request(0x03).hex(' ').upper() == 'DD A5 03 00 FF FD 77'


def test_daly_real_capture():
    s = read_bms(FakeSerial(DALY_PORT), 'daly', 0.3)
    assert (s.voltage, s.current, s.soc, s.cycles) == (22.8, 0.0, 43.8, 4)
    assert s.remaining_ah == 12.264 and s.full_ah == 28.0
    assert s.cells_mv == [3830, 4012, 3376, 3802, 3906, 3946]
    assert s.temps_c == [24.0, 26.0]
    assert s.charge_fet and s.discharge_fet
    assert s.faults == ['Cell imbalance L1']
    assert s.to_dict()['power'] == 0.0


def test_jbd_read_and_faults():
    basic = (be16(1326) + be16(-1240) + be16(6820) + be16(10000) + be16(142) + be16(0) + be16(0b10)
             + be16(0) + be16(0b1000001) + bytes([0x10, 68, 0b11, 4, 2]) + be16(2731 + 274) + be16(2731 + 281))
    cells = b''.join(be16(v) for v in (3312, 3318, 3309, 3321))
    port = FakeSerial({p.jbd_request(3): b'\x00\x11' + jbd_reply(3, basic),   # leading noise is skipped
                       p.jbd_request(4): jbd_reply(4, cells)})
    s = read_bms(port, 'jbd', 0.3)
    assert (s.voltage, s.current, s.soc, s.cycles) == (13.26, -12.4, 68, 142)
    assert s.cells_mv == [3312, 3318, 3309, 3321] and s.temps_c == [27.4, 28.1]
    assert s.balancing_mask == 0b10
    assert s.faults == ['Cell over-voltage', 'Discharge over-temp']


def test_bad_checksum_and_silence():
    bad = bytearray(jbd_reply(3, bytes(25)))
    bad[-2] ^= 0xFF
    assert p.jbd_parse(bytes(bad), 3) is None
    with pytest.raises(BmsError):
        read_bms(FakeSerial({}), 'daly', 0.1)


def test_env_file(tmp_path, monkeypatch):
    for k in [k for k in os.environ if k.startswith('BMS_')]:
        monkeypatch.delenv(k)
    env = tmp_path / '.env'
    env.write_text('# comment\nBMS_PORT=COM26\nBMS_PROTOCOL="DALY"\nBMS_BAUD=19200  # fast\nBMS_RATE=\n')
    s = Settings.load(str(env))
    assert (s.port, s.protocol, s.baud, s.rate) == ('COM26', 'daly', 19200, 1.0)
    monkeypatch.setenv('BMS_PORT', 'COM7')            # environment variable beats .env
    assert Settings.load(str(env)).port == 'COM7'


def test_auto_detect_skips_missing_port():
    ports = {'COM26': FakeSerial(DALY_PORT)}

    def opener(name, _baud):
        if name not in ports:
            raise OSError('not found')
        return ports[name]

    usb = [('COM5', 'USB device'), ('COM26', 'USB-SERIAL CH340')]
    assert find_bms(Settings(port='COM19', timeout=0.2), lambda m: None, opener, usb) == ('COM26', 'daly')
    assert find_bms(Settings(protocol='jbd', timeout=0.2), lambda m: None, opener, usb) is None
