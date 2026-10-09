package audit

import com.sinelynx.grindingrobot.core.tcp.TcpService
import java.io.BufferedOutputStream
import java.io.ByteArrayOutputStream
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import kotlinx.coroutines.*
import org.junit.Assert.*
import org.junit.Test

class AuditTransmissionProbe {
    @Test fun queuedOldSessionWriteUsesNewSessionStream() {
        val dispatcher = Executors.newSingleThreadExecutor().asCoroutineDispatcher()
        val scope = CoroutineScope(dispatcher + SupervisorJob())
        val entered = CountDownLatch(1)
        val release = CountDownLatch(1)
        val completed = CountDownLatch(1)
        val oldBytes = ByteArrayOutputStream()
        val newBytes = ByteArrayOutputStream()
        val service = TcpService()
        fun setField(name: String, value: Any) {
            service.javaClass.getDeclaredField(name).apply { isAccessible = true }.set(service, value)
        }
        try {
            setField("scope", scope)
            setField("isConnected", true)
            setField("outputStream", BufferedOutputStream(oldBytes))
            scope.launch { entered.countDown(); check(release.await(2, TimeUnit.SECONDS)) }
            assertTrue(entered.await(2, TimeUnit.SECONDS))
            assertTrue(service.sendData(byteArrayOf(11, 22, 33)))
            assertEquals(0, oldBytes.size())
            // Model the transport stream replacement during reconnect; sendData
            // reads the mutable global outputStream only when its coroutine runs.
            setField("outputStream", BufferedOutputStream(newBytes))
            scope.launch { completed.countDown() }
            release.countDown()
            assertTrue(completed.await(2, TimeUnit.SECONDS))
            assertEquals(0, oldBytes.size())
            assertArrayEquals(byteArrayOf(11, 22, 33), newBytes.toByteArray())
            println("old_session_queued_send: old_stream_bytes=0 new_stream_bytes=${newBytes.size()}")
        } finally {
            release.countDown()
            scope.cancel()
            dispatcher.close()
        }
    }
}
