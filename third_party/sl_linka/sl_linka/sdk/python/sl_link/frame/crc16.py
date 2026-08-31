"""
CRC16-CCITT calculation for SL-LinkA protocol

Polynomial: 0x1021
Initial value: 0xFFFF
"""

try:
    import binascii
except Exception:  # pragma: no cover - platform fallback
    binascii = None


def crc16_ccitt(data: bytes) -> int:
    """
    Calculate CRC16-CCITT checksum.
    
    Args:
        data: Input bytes to calculate CRC over
        
    Returns:
        16-bit CRC value
    """
    if binascii is not None:
        # binascii.crc_hqx uses CRC-CCITT with polynomial 0x1021 and caller-supplied init.
        return binascii.crc_hqx(data, 0xFFFF) & 0xFFFF

    crc = 0xFFFF

    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1

    return crc & 0xFFFF


def crc16_ccitt_bytes(data: bytes) -> bytes:
    """
    Calculate CRC16-CCITT and return as Little Endian bytes.
    
    Args:
        data: Input bytes to calculate CRC over
        
    Returns:
        2 bytes in Little Endian order
    """
    crc = crc16_ccitt(data)
    return crc.to_bytes(2, byteorder='little')
