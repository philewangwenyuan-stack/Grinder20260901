package audit

import com.sinelynx.grindingrobot.core.model.state.*
import com.sinelynx.grindingrobot.core.data.state.AppState
import com.sinelynx.grindingrobot.core.tcp.SlLinkManager
import com.sinelynx.grindingrobot.feature.device.viewmodel.DeviceTaskMapController
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.*
import org.junit.Assert.*
import org.junit.Test
import sun.misc.Unsafe

class AuditControllerProbe {
    @Test fun failedLoadIsNotRetriedByVisibilityOrRepeatedStatus() = runBlocking {
        var calls = 0
        val controller = DeviceTaskMapController(this, TaskMapDisplayCache(),
            MutableSharedFlow(), MutableSharedFlow(), MutableSharedFlow(),
            sendMap = { calls++; false }, sendRegions = { calls++; false },
            sendPath = { _, _, _ -> calls++; false }, decodeSize = { 1 to 1 })
        try {
            controller.onTask("task", 7)
            controller.setVisible(true)
            repeat(100) { yield() }
            assertEquals(3, calls)
            controller.setVisible(false)
            controller.setVisible(true)
            controller.onTask("task", 7)
            repeat(100) { yield() }
            assertEquals(3, calls)
            println("failed_load_after_foreground: network_calls=$calls (no automatic retry)")
        } finally { controller.close() }
    }

    @Test fun cancelledUiWaitLeavesProductionMapRequestGateBusy() {
        // This allocator makes an inert AppState; these gate methods do not use
        // it or Android APIs. No socket, ROS process, serial port or device.
        val field = Unsafe::class.java.getDeclaredField("theUnsafe").apply { isAccessible = true }
        val unsafe = field.get(null) as Unsafe
        val appState = unsafe.allocateInstance(AppState::class.java) as AppState
        val manager = SlLinkManager(appState)
        assertTrue(manager.expectMapRequest(111))
        // The UI's setVisible(false) only cancels its Flow waiting job. It does
        // not call cancelMapRequest(111), so this is the production gate state.
        assertFalse(manager.expectMapRequest(222))
        manager.cancelMapRequest(111)
        assertTrue(manager.expectMapRequest(222))
        println("map_wait_cancel_without_gate_cancel: immediate_new_request=false; explicit_cancel=true")
    }
}
