package audit

import sl_link.*
import com.sinelynx.grindingrobot.core.tcp.TaskPathChunkAssembler
import java.nio.ByteBuffer
import java.nio.ByteOrder

fun wire(payload: ByteArray, seq: Int = 1): ByteArray {
    val header = ByteBuffer.allocate(15).order(ByteOrder.LITTLE_ENDIAN)
        .put(0xfd.toByte()).put(0x55).put(1).put(0).putShort(seq.toShort())
        .putShort(0).put(0x10).put(0x01).put(0).putShort(0x0504)
        .putShort(payload.size.toShort()).array()
    val crcInput = header + payload
    val crc = Crc16.calculate(crcInput).toInt()
    return crcInput + byteArrayOf(crc.toByte(), (crc shr 8).toByte(), 0xfe.toByte())
}

fun main() {
    val parser = SlFrameParser()
    val original = byteArrayOf(7, 8, 9)
    check(parser.parse(wire(original)).single().payload.contentEquals(original))
    val zero = parser.parse(wire(byteArrayOf(), 2)).single()
    println("zero_length_after_nonempty: actual_payload=${zero.payload.size} expected=0")
    check(zero.payload.contentEquals(original))

    val persistent = SlFrameParser()
    val firstConnection = wire(ByteArray(4096) { 0x11 }, 3)
    persistent.parse(firstConnection.copyOf(20))
    val newConnection = wire(byteArrayOf(42), 4)
    val recovered = persistent.parse(newConnection)
    println("reconnect_without_parser_reset: new_frames=${recovered.size} expected=1")
    check(recovered.isEmpty())
    persistent.reset()
    check(persistent.parse(newConnection).single().payload.contentEquals(byteArrayOf(42)))

    val assembler = TaskPathChunkAssembler()
    repeat(1000) { i ->
        check(assembler.append("task", 1, 0, 2, ByteArray(4096), "incomplete-$i") == null)
    }
    val field = assembler.javaClass.getDeclaredField("assemblies").apply { isAccessible = true }
    val entries = (field.get(assembler) as Map<*, *>).size
    println("incomplete_path_requests: entries=$entries retained_payload_bytes=${entries * 4096}")
    check(entries == 1000)
    val sample = ByteArray(4096) { it.toByte() }
    val sampleWire = wire(sample, 5)
    repeat(5) { sample.joinToString("") { "%02x".format(it) } }
    var consumed = 0
    val parseStart = System.nanoTime()
    repeat(100) { consumed += SlFrameParser().parse(sampleWire).size }
    val parseMs = (System.nanoTime() - parseStart) / 1e6
    val hexStart = System.nanoTime()
    repeat(100) { consumed += sample.joinToString("") { "%02x".format(it) }.length }
    val hexMs = (System.nanoTime() - hexStart) / 1e6
    println("desktop_microbenchmark_100x4096: parse_ms=$parseMs unused_hex_ms=$hexMs consumed=$consumed")
    val monitor = SlLinkMonitor()
    monitor.update(0x10u, 100u)
    val restartGap = monitor.update(0x10u, 0u)
    val duplicateGap = monitor.update(0x10u, 0u)
    println("sequence_monitor: server_restart_gap=$restartGap duplicate_gap=$duplicateGap")
    check(restartGap == 65435 && duplicateGap == -1)
    println("All probes used current Kotlin source; no Android device or robot connection.")
}
