package sl_link

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * SL-LinkA message builder for creating protocol frames.
 */
object SlMessageBuilder {

    private var sequenceNumber: UShort = 0u
    private var sourceId: UByte = 0x01u

    fun setSourceId(srcId: UByte) {
        sourceId = srcId
    }

    private fun getNextSeq(): UShort {
        val current = sequenceNumber
        sequenceNumber = ((sequenceNumber.toInt() + 1) and 0xFFFF).toUShort()
        return current
    }

    fun buildFrame(
        msgId: UShort,
        payloadData: ByteArray,
        dstId: UByte,
        compId: UByte,
        flags: UByte = 0u,
        ackSeq: UShort = 0u
    ): ByteArray {
        val payloadLen = payloadData.size
        val frameSize = SlFrame.HEADER_SIZE + payloadLen + 2 + 1

        val buffer = ByteBuffer.allocate(frameSize).order(ByteOrder.LITTLE_ENDIAN)
        buffer.put(SlFrame.STX1.toByte())
        buffer.put(SlFrame.STX2.toByte())
        buffer.put(0x01.toByte())
        buffer.put(flags.toByte())
        buffer.putShort(getNextSeq().toShort())
        buffer.putShort(ackSeq.toShort())
        buffer.put(sourceId.toByte())
        buffer.put(dstId.toByte())
        buffer.put(compId.toByte())
        buffer.putShort(msgId.toShort())
        buffer.putShort(payloadLen.toShort())
        buffer.put(payloadData)

        val crcData = buffer.array().copyOfRange(0, SlFrame.HEADER_SIZE + payloadLen)
        val crc = Crc16.calculate(crcData)
        buffer.putShort(crc.toShort())
        buffer.put(SlFrame.TAIL.toByte())
        return buffer.array()
    }

    fun buildWifiConfigRaw(serializedData: ByteArray, dstId: UByte): ByteArray {
        return buildFrame(0x0201u, serializedData, dstId, 0x04u)
    }

    fun buildSettingsReadRequestRaw(serializedData: ByteArray, dstId: UByte): ByteArray {
        return buildFrame(0x0203u, serializedData, dstId, 0x05u)
    }

    fun buildSettingsWriteRequestRaw(serializedData: ByteArray, dstId: UByte): ByteArray {
        return buildFrame(0x0205u, serializedData, dstId, 0x05u)
    }

    fun buildCameraFrameRequestRaw(serializedData: ByteArray, dstId: UByte): ByteArray {
        return buildFrame(0x0302u, serializedData, dstId, 0x06u)
    }

    fun buildMapRequestRaw(serializedData: ByteArray, dstId: UByte): ByteArray {
        return buildFrame(0x0304u, serializedData, dstId, 0x06u)
    }

    fun buildControlCommandRaw(serializedData: ByteArray, dstId: UByte): ByteArray {
        return buildFrame(0x0401u, serializedData, dstId, 0x07u)
    }

    fun buildTaskConfigRaw(serializedData: ByteArray, dstId: UByte): ByteArray {
        return buildFrame(0x0501u, serializedData, dstId, 0x08u)
    }

    fun buildTaskCommandRaw(serializedData: ByteArray, dstId: UByte): ByteArray {
        return buildFrame(0x0502u, serializedData, dstId, 0x08u)
    }

    fun buildTaskPathRequestRaw(serializedData: ByteArray, dstId: UByte): ByteArray {
        return buildFrame(0x0505u, serializedData, dstId, 0x08u)
    }

    fun buildMapPreviewRequestRaw(serializedData: ByteArray, dstId: UByte): ByteArray {
        return buildFrame(0x0507u, serializedData, dstId, 0x08u)
    }

    fun buildMapEditCommandRaw(serializedData: ByteArray, dstId: UByte): ByteArray {
        return buildFrame(0x0509u, serializedData, dstId, 0x08u)
    }

    fun buildVideoStreamInfoRequestRaw(serializedData: ByteArray, dstId: UByte): ByteArray {
        return buildFrame(0x050Cu, serializedData, dstId, 0x08u)
    }

    fun buildAck(receivedFrame: SlFrame, payloadData: ByteArray = ByteArray(0)): ByteArray {
        return buildFrame(
            msgId = receivedFrame.msgId,
            payloadData = payloadData,
            dstId = receivedFrame.srcId,
            compId = receivedFrame.compId,
            flags = 0x00u,
            ackSeq = receivedFrame.seq
        )
    }
}
