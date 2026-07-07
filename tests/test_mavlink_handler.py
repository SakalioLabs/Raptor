import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.comm.mavlink_handler import MAVLinkHandler, MAVLinkMessage

def test_pack_v2_header():
    h = MAVLinkHandler()
    msg = MAVLinkMessage(msg_id=0, payload=b'\x00'*9)
    data = h.pack(msg)
    assert data[0] == 0xFD
    assert data[1] == 9

def test_heartbeat():
    h = MAVLinkHandler()
    data = h.send_heartbeat()
    assert len(data) > 12
    assert data[0] == 0xFD

def test_seq_increment():
    h = MAVLinkHandler()
    d1 = h.send_heartbeat()
    d2 = h.send_heartbeat()
    assert d2[3] == (d1[3] + 1) & 0xFF

def test_parse_header():
    h = MAVLinkHandler()
    msg = MAVLinkMessage(msg_id=30, payload=b'\x00'*4, seq=5, sys_id=1, comp_id=1)
    data = h.pack(msg)
    hdr = h.parse_header(data)
    assert hdr is not None
    assert hdr['msg_id'] == 30
    assert hdr['seq'] == 5

def test_parse_short_data():
    h = MAVLinkHandler()
    assert h.parse_header(b'\x00') is None
