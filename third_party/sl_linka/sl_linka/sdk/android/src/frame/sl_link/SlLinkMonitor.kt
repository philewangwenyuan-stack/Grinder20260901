package sl_link

/**
 * Statistics for a specific device on the SL-LinkA network
 */
data class SlDeviceStats(
    val srcId: UByte,
    var lastSeq: UShort = 0u,
    var totalReceived: Long = 0,
    var totalLost: Long = 0,
    var firstPacketReceived: Boolean = false
) {
    val totalExpected: Long get() = totalReceived + totalLost
    val lossRate: Double get() = if (totalExpected > 0) totalLost.toDouble() / totalExpected else 0.0
}

/**
 * Monitor and track link quality per source device
 */
class SlLinkMonitor {
    private val deviceStats = mutableMapOf<UByte, SlDeviceStats>()

    /**
     * Update stats with a newly received frame
     * @return Number of packets detected lost in this jump
     */
    fun update(srcId: UByte, seq: UShort): Int {
        val stats = deviceStats.getOrPut(srcId) { SlDeviceStats(srcId) }
        var lostInJump = 0

        if (stats.firstPacketReceived) {
            val expected = ((stats.lastSeq.toInt() + 1) and 0xFFFF).toUShort()
            if (seq != expected) {
                // Detected gap
                lostInJump = (seq.toInt() - stats.lastSeq.toInt() + 65536) % 65536 - 1
                stats.totalLost += lostInJump
            }
        } else {
            stats.firstPacketReceived = true
        }

        stats.lastSeq = seq
        stats.totalReceived++
        return lostInJump
    }

    /**
     * Get statistics for all active devices
     */
    fun getAllStats(): List<SlDeviceStats> = deviceStats.values.toList()

    /**
     * Get statistics for a specific device
     */
    fun getDeviceStats(srcId: UByte): SlDeviceStats? = deviceStats[srcId]

    /**
     * Reset statistics for all devices
     */
    fun resetAll() {
        deviceStats.clear()
    }

    /**
     * Print current stats to console
     */
    fun logStats() {
        println("\n--- SL-LinkA Quality Stats ---")
        println("%-8s | %-8s | %-8s | %-8s | %-6s".format("SRC_ID", "RECV", "LOST", "TOTAL", "LOSS%"))
        println("---------|----------|----------|----------|--------")
        
        deviceStats.values.sortedBy { it.srcId }.forEach { s ->
            println("0x%02X     | %-8d | %-8d | %-8d | %3.2f%%".format(
                s.srcId.toInt(), s.totalReceived, s.totalLost, s.totalExpected, s.lossRate * 100.0
            ))
        }
    }
}
