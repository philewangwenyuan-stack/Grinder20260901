"""
SL-LinkA Python SDK

Protocol implementation for SL-LinkA communication.

Usage:
    from sl_link import SlFrame, SlFrameParser
    from sl_link.message_gen import sl_link_pb2 as pb
"""

__version__ = "1.0.0"

# Re-export frame handling classes
from .frame import (
    crc16_ccitt,
    SlFrame,
    SlFrameParser,
    SL_PROTOCOL_VERSION,
    MessageHandler,
    get_real_value,
    set_wire_value,
    to_dict_real,
    from_dict_real,
    get_field_info,
    list_scaled_fields,
)

# Make message_gen accessible
from . import message_gen
