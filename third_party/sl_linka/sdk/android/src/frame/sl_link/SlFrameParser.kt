package sl_link

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * SL-LinkA frame parser with support for fragmented and concatenated packets
 */
class SlFrameParser {
    
    private enum class ParseState {
        WAIT_STX1,
        WAIT_STX2,
        PARSE_HEADER,
        PARSE_PAYLOAD,
        PARSE_CRC,
        PARSE_TAIL
    }
    
    private var state = ParseState.WAIT_STX1
    private val headerBuffer = ByteArray(SlFrame.HEADER_SIZE)
    private var headerIndex = 0
    private var payloadBuffer = ByteArray(0)
    private var payloadIndex = 0
    private var payloadLength = 0
    private var crcReceived: UShort = 0u
    private var crcIndex = 0
    
    // Parsed header fields
    private var version: UByte = 0u
    private var flags: UByte = 0u
    private var seq: UShort = 0u
    private var ackSeq: UShort = 0u
    private var srcId: UByte = 0u
    private var dstId: UByte = 0u
    private var compId: UByte = 0u
    private var msgId: UShort = 0u
    
    /**
     * Parse incoming data and return list of complete frames
     * Handles both fragmented frames and multiple frames in one buffer
     */
    fun parse(data: ByteArray): List<SlFrame> {
        val frames = mutableListOf<SlFrame>()
        
        for (byte in data) {
            val frame = parseByte(byte)
            if (frame != null) {
                frames.add(frame)
            }
        }
        
        return frames
    }
    
    /**
     * Reset parser to initial state
     */
    fun reset() {
        state = ParseState.WAIT_STX1
        headerIndex = 0
        payloadIndex = 0
        crcIndex = 0
    }
    
    private fun parseByte(byte: Byte): SlFrame? {
        val ubyte = byte.toUByte()
        
        when (state) {
            ParseState.WAIT_STX1 -> {
                if (ubyte == SlFrame.STX1) {
                    headerBuffer[0] = byte
                    headerIndex = 1
                    state = ParseState.WAIT_STX2
                }
            }
            
            ParseState.WAIT_STX2 -> {
                if (ubyte == SlFrame.STX2) {
                    headerBuffer[1] = byte
                    headerIndex = 2
                    state = ParseState.PARSE_HEADER
                } else {
                    reset()
                }
            }
            
            ParseState.PARSE_HEADER -> {
                headerBuffer[headerIndex++] = byte
                
                if (headerIndex >= SlFrame.HEADER_SIZE) {
                    // Parse header (Little Endian)
                    val buffer = ByteBuffer.wrap(headerBuffer).order(ByteOrder.LITTLE_ENDIAN)
                    buffer.position(0)
                    buffer.get() // STX1
                    buffer.get() // STX2
                    version = buffer.get().toUByte()
                    flags = buffer.get().toUByte()
                    seq = buffer.short.toUShort()
                    ackSeq = buffer.short.toUShort()
                    srcId = buffer.get().toUByte()
                    dstId = buffer.get().toUByte()
                    compId = buffer.get().toUByte()
                    msgId = buffer.short.toUShort()
                    payloadLength = buffer.short.toInt() and 0xFFFF
                    
                    // Validate payload length
                    if (payloadLength > 65535) {
                        reset()
                        return null
                    }
                    
                    // Prepare payload buffer
                    if (payloadLength > 0) {
                        payloadBuffer = ByteArray(payloadLength)
                        payloadIndex = 0
                        state = ParseState.PARSE_PAYLOAD
                    } else {
                        state = ParseState.PARSE_CRC
                        crcIndex = 0
                    }
                }
            }
            
            ParseState.PARSE_PAYLOAD -> {
                payloadBuffer[payloadIndex++] = byte
                
                if (payloadIndex >= payloadLength) {
                    state = ParseState.PARSE_CRC
                    crcIndex = 0
                }
            }
            
            ParseState.PARSE_CRC -> {
                if (crcIndex == 0) {
                    crcReceived = ubyte.toUShort()
                    crcIndex = 1
                } else {
                    crcReceived = (crcReceived.toInt() or (ubyte.toInt() shl 8)).toUShort()
                    
                    // Validate CRC
                    val crcCalc = calculateFrameCrc()
                    if (crcCalc == crcReceived) {
                        state = ParseState.PARSE_TAIL
                    } else {
                        // CRC mismatch
                        reset()
                    }
                }
            }
            
            ParseState.PARSE_TAIL -> {
                if (ubyte == SlFrame.TAIL) {
                    // Frame complete!
                    val frame = SlFrame(
                        version = version,
                        flags = flags,
                        seq = seq,
                        ackSeq = ackSeq,
                        srcId = srcId,
                        dstId = dstId,
                        compId = compId,
                        msgId = msgId,
                        payload = payloadBuffer.copyOf(),
                        crc = crcReceived
                    )
                    reset()
                    return frame
                } else {
                    // Invalid tail
                    reset()
                }
            }
        }
        
        return null
    }
    
    private fun calculateFrameCrc(): UShort {
        // CRC over header + payload
        val data = ByteArray(SlFrame.HEADER_SIZE + payloadLength)
        System.arraycopy(headerBuffer, 0, data, 0, SlFrame.HEADER_SIZE)
        if (payloadLength > 0) {
            System.arraycopy(payloadBuffer, 0, data, SlFrame.HEADER_SIZE, payloadLength)
        }
        return Crc16.calculate(data)
    }
}
