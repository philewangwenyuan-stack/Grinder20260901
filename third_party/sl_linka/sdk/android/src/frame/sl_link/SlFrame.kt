package sl_link

/**
 * SL-LinkA frame data class
 */
data class SlFrame(
    val version: UByte,
    val flags: UByte,
    val seq: UShort,
    val ackSeq: UShort,
    val srcId: UByte,
    val dstId: UByte,
    val compId: UByte,
    val msgId: UShort,
    val payload: ByteArray,
    val crc: UShort
) {
    companion object {
        const val STX1: UByte = 0xFDu
        const val STX2: UByte = 0x55u
        const val TAIL: UByte = 0xFEu
        const val HEADER_SIZE = 15
    }
    
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        
        other as SlFrame
        
        if (version != other.version) return false
        if (flags != other.flags) return false
        if (seq != other.seq) return false
        if (ackSeq != other.ackSeq) return false
        if (srcId != other.srcId) return false
        if (dstId != other.dstId) return false
        if (compId != other.compId) return false
        if (msgId != other.msgId) return false
        if (!payload.contentEquals(other.payload)) return false
        if (crc != other.crc) return false
        
        return true
    }
    
    override fun hashCode(): Int {
        var result = version.hashCode()
        result = 31 * result + flags.hashCode()
        result = 31 * result + seq.hashCode()
        result = 31 * result + ackSeq.hashCode()
        result = 31 * result + srcId.hashCode()
        result = 31 * result + dstId.hashCode()
        result = 31 * result + compId.hashCode()
        result = 31 * result + msgId.hashCode()
        result = 31 * result + payload.contentHashCode()
        result = 31 * result + crc.hashCode()
        return result
    }
}
