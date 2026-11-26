"""Protocol tests against a simulated BMS. Run with: python3 -m pytest test/ (no ROS or hardware needed)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bms_ros import protocols as p  # noqa: E402
from bms_ros.bms_node import BmsClient, BmsError, Settings, find_bms, _parse_env_file  # noqa: E402


class FakeSerial:
    """Answers each request with a canned reply, delivered a few bytes at a time."""

    def __init__(self, replies):
        self.replies = replies
        self.rx = bytearray()

    def reset_input_buffer(self):
        self.rx.clear()

    def write(self, data):
        self.rx += self.replies.get(bytes(data), b'')

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n=1):
        n = min(n, 5, len(self.rx))
        out = bytes(self.rx[:n])
        del self.rx[:n]
        return out


# ---------------------------------------------------------------- JBD helpers
def jbd_reply(cmd, data, status=0):
    body = bytes([status, len(data)]) + data
    chk = (0x10000 - sum(body)) & 0xFFFF
    return bytes([0xDD, cmd]) + body + bytes([chk >> 8, chk & 0xFF, 0x77])


def be16(v):
    v &= 0xFFFF
    return bytes([v >> 8, v & 0xFF])


CELLS = [3312, 3318, 3309, 3321]


def jbd_basic_data(current_10ma=-1240, prot=0):
    return (be16(1326) + be16(current_10ma) + be16(6820) + be16(10000) + be16(142) + be16(0)
            + be16(0b0010) + be16(0) + be16(prot) + bytes([0x10, 68, 0b11, 4, 2])
            + be16(2731 + 274) + be16(2731 + 281))


def test_jbd_request_frames():
    assert p.jbd_request(0x03).hex(' ').upper() == 'DD A5 03 00 FF FD 77'
    assert p.jbd_request(0x04).hex(' ').upper() == 'DD A5 04 00 FF FC 77'


def test_jbd_full_read():
    port = FakeSerial({
        p.jbd_request(3): b'\x00\x11' + jbd_reply(3, jbd_basic_data()),  # leading noise
        p.jbd_request(4): jbd_reply(4, b''.join(be16(c) for c in CELLS)),
    })
    s = BmsClient(port, 'jbd', timeout=0.3).read()
    assert s.voltage == 13.26
    assert s.current == -12.4
    assert s.remaining_ah == 68.2 and s.full_ah == 100.0
    assert s.cycles == 142 and s.soc == 68
    assert s.charge_fet and s.discharge_fet
    assert s.cell_count == 4 and s.cells_mv == CELLS
    assert s.temps_c == [27.4, 28.1]
    assert s.balancing_mask == 0b10
    assert s.faults == []
    d = s.to_dict()
    assert round(d['power'], 2) == round(13.26 * -12.4, 2)


def test_jbd_faults_decoded():
    port = FakeSerial({p.jbd_request(3): jbd_reply(3, jbd_basic_data(prot=(1 << 0) | (1 << 6)))})
    s = BmsClient(port, 'jbd', timeout=0.2).read()
    assert s.faults == ['Cell over-voltage', 'Discharge over-temp']


def test_jbd_bad_checksum_rejected():
    bad = bytearray(jbd_reply(3, jbd_basic_data()))
    bad[-2] ^= 0xFF
    assert p.jbd_find_frame(bytes(bad)) is None
    try:
        BmsClient(FakeSerial({p.jbd_request(3): bytes(bad)}), 'jbd', timeout=0.15).read()
        assert False, 'expected BmsError'
    except BmsError:
        pass


# --------------------------------------------------------------- Daly helpers
def daly_reply(cmd, data8):
    f = bytes([0xA5, 0x01, cmd, 0x08]) + bytes(data8)
    return f + bytes([sum(f) & 0xFF])


def test_daly_request_frame():
    assert p.daly_request(0x90).hex(' ').upper() == 'A5 40 90 08 00 00 00 00 00 00 00 00 7D'


def test_daly_full_read():
    cells = [3301, 3302, 3303, 3304, 3305]
    port = FakeSerial({
        p.daly_request(0x90): daly_reply(0x90, be16(166) + be16(0) + be16(30000 + 52) + be16(755)),
        p.daly_request(0x93): daly_reply(0x93, bytes([1, 1, 0, 0]) + (75500).to_bytes(4, 'big')),
        p.daly_request(0x94): daly_reply(0x94, bytes([5, 2, 0, 0, 0]) + be16(37) + b'\x00'),
        p.daly_request(0x95): (daly_reply(0x95, bytes([1]) + be16(cells[0]) + be16(cells[1]) + be16(cells[2]) + b'\x00')
                               + daly_reply(0x95, bytes([2]) + be16(cells[3]) + be16(cells[4]) + be16(0) + b'\x00')),
        p.daly_request(0x96): daly_reply(0x96, bytes([1, 65, 63, 0, 0, 0, 0, 0])),
        p.daly_request(0x98): daly_reply(0x98, bytes([0b1, 0, 0, 0, 0, 0, 0b100, 0])),
    })
    s = BmsClient(port, 'daly', timeout=0.3).read()
    assert s.voltage == 16.6
    assert s.current == 5.2
    assert s.soc == 75.5
    assert s.charge_fet is True and s.discharge_fet is False
    assert s.remaining_ah == 75.5 and s.full_ah == 100.0
    assert s.cell_count == 5 and s.cycles == 37
    assert s.cells_mv == cells
    assert s.temps_c == [25.0, 23.0]
    assert s.faults == ['Cell over-voltage L1', 'Short-circuit protection']


def test_daly_no_reply():
    try:
        BmsClient(FakeSerial({}), 'daly', timeout=0.1).read()
        assert False
    except BmsError:
        pass


# ------------------------------------------------------------- .env + detection
def test_env_file_parsing(tmp_path, monkeypatch):
    env = tmp_path / '.env'
    env.write_text('# comment\nBMS_PORT=COM26\nBMS_PROTOCOL="DALY"\nBMS_BAUD=19200  # fast\nBMS_RATE=\n')
    assert _parse_env_file(env)['BMS_BAUD'] == '19200'
    for k in list(__import__('os').environ):
        if k.startswith('BMS_'):
            monkeypatch.delenv(k)
    s = Settings.load(str(env))
    assert (s.port, s.protocol, s.baud, s.rate) == ('COM26', 'daly', 19200, 1.0)
    monkeypatch.setenv('BMS_PORT', 'COM7')          # real env var wins over .env
    assert Settings.load(str(env)).port == 'COM7'


def _daly_port():
    return FakeSerial({
        p.daly_request(0x90): daly_reply(0x90, be16(228) + be16(0) + be16(30000) + be16(438)),
    })


def test_detect_falls_back_when_configured_port_missing_or_silent():
    ports = {'COM19': FakeSerial({}), 'COM26': _daly_port()}

    def opener(name, _baud):
        if name not in ports:
            raise OSError('not found')
        return ports[name]
    FakeSerial.close = lambda self: None
    logs = []
    s = Settings(port='COM19', protocol='', timeout=0.15)
    assert find_bms(s, logs.append, opener, candidates=[('COM19', 'CH340'), ('COM26', 'CH340')]) == ('COM26', 'daly')
    assert any('COM19' in m for m in logs)
    s = Settings(port='', protocol='jbd', timeout=0.15)  # wrong protocol forced: nothing answers
    assert find_bms(s, logs.append, opener, candidates=[('COM26', 'CH340')]) is None
