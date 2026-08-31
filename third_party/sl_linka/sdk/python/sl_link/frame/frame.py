import struct
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional
from .crc16 import crc16_ccitt

# Constants
SL_STX1 = 0xFD
SL_STX2 = 0x55
SL_TAIL = 0xFE
SL_HEADER_SIZE = 15
SL_PROTOCOL_VERSION = 0x01

@dataclass
class SlFrame:
    version: int
    flags: int
    seq: int
    ack_seq: int
    src_id: int
    dst_id: int
    comp_id: int
    msg_id: int
    payload: bytes
    crc: int = 0

    def pack(self) -> bytes:
        """Pack frame into bytes for transmission"""
        payload_len = len(self.payload)
        
        # Build header (Little Endian)
        # B: uint8, H: uint16
        header = struct.pack('<BBBBHHBBBHH', 
                             SL_STX1, 
                             SL_STX2, 
                             self.version, 
                             self.flags, 
                             self.seq, 
                             self.ack_seq, 
                             self.src_id, 
                             self.dst_id, 
                             self.comp_id, 
                             self.msg_id, 
                             payload_len)
        
        # Calculate CRC over header + payload
        data_to_crc = header + self.payload
        self.crc = crc16_ccitt(data_to_crc)
        
        # Build final frame: header + payload + crc(2) + tail(1)
        return data_to_crc + struct.pack('<H', self.crc) + bytes([SL_TAIL])

class ParseState(Enum):
    WAIT_STX1 = 0
    WAIT_STX2 = 1
    PARSE_HEADER = 2
    PARSE_PAYLOAD = 3
    PARSE_CRC = 4
    PARSE_TAIL = 5

class SlFrameParser:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = ParseState.WAIT_STX1
        self.header_buffer = bytearray()
        self.payload_buffer = bytearray()
        self.payload_len = 0
        self.crc_buffer = bytearray()
        self.crc_received = 0

    def parse_byte(self, b: int) -> Optional[SlFrame]:
        if self.state == ParseState.WAIT_STX1:
            if b == SL_STX1:
                self.header_buffer = bytearray([b])
                self.state = ParseState.WAIT_STX2
            return None

        elif self.state == ParseState.WAIT_STX2:
            if b == SL_STX2:
                self.header_buffer.append(b)
                self.state = ParseState.PARSE_HEADER
            else:
                self.reset()
            return None

        elif self.state == ParseState.PARSE_HEADER:
            self.header_buffer.append(b)
            if len(self.header_buffer) == SL_HEADER_SIZE:
                # Unpack header
                try:
                    # header format: stx1, stx2, ver, flags, seq, ack_seq, src, dst, comp, msg_id, payload_len
                    header_vals = struct.unpack('<BBBBHHBBBHH', self.header_buffer)
                    self.version = header_vals[2]
                    self.flags = header_vals[3]
                    self.seq = header_vals[4]
                    self.ack_seq = header_vals[5]
                    self.src_id = header_vals[6]
                    self.dst_id = header_vals[7]
                    self.comp_id = header_vals[8]
                    self.msg_id = header_vals[9]
                    self.payload_len = header_vals[10]
                    
                    if self.payload_len > 0:
                        self.state = ParseState.PARSE_PAYLOAD
                    else:
                        self.state = ParseState.PARSE_CRC
                except Exception:
                    self.reset()
            return None

        elif self.state == ParseState.PARSE_PAYLOAD:
            self.payload_buffer.append(b)
            if len(self.payload_buffer) == self.payload_len:
                self.state = ParseState.PARSE_CRC
            return None

        elif self.state == ParseState.PARSE_CRC:
            self.crc_buffer.append(b)
            if len(self.crc_buffer) == 2:
                self.crc_received = struct.unpack('<H', self.crc_buffer)[0]
                
                # Verify CRC
                data_to_crc = self.header_buffer + self.payload_buffer
                crc_calc = crc16_ccitt(data_to_crc)
                
                if crc_calc == self.crc_received:
                    self.state = ParseState.PARSE_TAIL
                else:
                    self.reset()
            return None

        elif self.state == ParseState.PARSE_TAIL:
            if b == SL_TAIL:
                frame = SlFrame(
                    version=self.version,
                    flags=self.flags,
                    seq=self.seq,
                    ack_seq=self.ack_seq,
                    src_id=self.src_id,
                    dst_id=self.dst_id,
                    comp_id=self.comp_id,
                    msg_id=self.msg_id,
                    payload=bytes(self.payload_buffer),
                    crc=self.crc_received
                )
                self.reset()
                return frame
            else:
                self.reset()
            return None

        return None

    def parse(self, data: bytes) -> List[SlFrame]:
        frames = []
        for b in data:
            frame = self.parse_byte(b)
            if frame:
                frames.append(frame)
        return frames
