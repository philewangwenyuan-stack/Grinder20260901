"""
SL-LinkA frame handling module.
Hand-written code for protocol frame processing.
"""

from .crc16 import crc16_ccitt, crc16_ccitt_bytes
from .frame import SlFrame, SlFrameParser, SL_PROTOCOL_VERSION
from .messages import MessageHandler
from .unit_converter import (
    get_real_value,
    set_wire_value,
    to_dict_real,
    from_dict_real,
    get_field_info,
    list_scaled_fields,
)
