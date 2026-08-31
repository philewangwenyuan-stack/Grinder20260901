package sl_link

object Crc16 {
    /**
     * Calculate CRC16-CCITT checksum
     * Polynomial: 0x1021
     * Initial value: 0xFFFF
     */
    fun calculate(data: ByteArray, offset: Int = 0, length: Int = data.size): UShort {
        var crc = 0xFFFF
        
        for (i in offset until offset + length) {
            crc = crc xor ((data[i].toInt() and 0xFF) shl 8)
            
            repeat(8) {
                crc = if ((crc and 0x8000) != 0) {
                    (crc shl 1) xor 0x1021
                } else {
                    crc shl 1
                }
            }
        }
        
        return (crc and 0xFFFF).toUShort()
    }
}
