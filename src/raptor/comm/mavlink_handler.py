"""MAVLink message parser and handler."""
from dataclasses import dataclass, field
from enum import IntEnum
import struct


class MAVType(IntEnum):
    HEARTBEAT = 0
    ATTITUDE = 30
    GPS_RAW_INT = 24
    COMMAND_LONG = 76
    MISSION_ITEM = 39


@dataclass
class MAVLinkMessage:
    msg_id: int
    payload: bytes = field(default_factory=bytes)
    seq: int = 0
    sys_id: int = 1
    comp_id: int = 1


class MAVLinkHandler:
    STX_V2 = 0xFD
    HEADER_LEN_V2 = 12

    def __init__(self, sys_id: int = 1, comp_id: int = 1):
        self.sys_id = sys_id
        self.comp_id = comp_id
        self._tx_seq = 0
        self._rx_buffer = bytearray()

    def pack(self, msg: MAVLinkMessage) -> bytes:
        payload_len = len(msg.payload)
        header = bytearray()
        header.append(self.STX_V2)
        header.append(payload_len)
        header.append(0)
        header.append(msg.seq)
        header.append(msg.sys_id)
        header.append(msg.comp_id)
        header.append(0)
        header.extend(struct.pack('<I', msg.msg_id)[:3])
        return bytes(header) + msg.payload

    def send_heartbeat(self) -> bytes:
        payload = struct.pack('<BBBBBI', 0, 0, 0, 0, 0, 0)
        msg = MAVLinkMessage(msg_id=MAVType.HEARTBEAT, payload=payload,
                             seq=self._tx_seq, sys_id=self.sys_id, comp_id=self.comp_id)
        self._tx_seq = (self._tx_seq + 1) & 0xFF
        return self.pack(msg)

    def parse_header(self, data: bytes) -> dict | None:
        if len(data) < self.HEADER_LEN_V2:
            return None
        if data[0] != self.STX_V2:
            return None
        payload_len = data[1]
        seq = data[3]
        sys_id = data[4]
        comp_id = data[5]
        msg_id = data[7] | (data[8] << 8) | (data[9] << 16)
        return {"payload_len": payload_len, "seq": seq, "sys_id": sys_id,
                "comp_id": comp_id, "msg_id": msg_id}
