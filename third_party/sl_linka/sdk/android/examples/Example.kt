package sl_link.examples

import sl_link.SlFrame
import sl_link.SlFrameParser
import sl_link.SlLinkMonitor
import sl_link.SlMessageBuilder

/**
 * Example usage of the expanded single-lower-device SL-LinkA protocol.
 */
class SlLinkExample {

    private val parser = SlFrameParser()
    private val monitor = SlLinkMonitor()

    init {
        SlMessageBuilder.setSourceId(0x01u)
    }

    fun sendWifiConfig(serializedWifiConfig: ByteArray): ByteArray =
        SlMessageBuilder.buildWifiConfigRaw(serializedWifiConfig, 0x10u)

    fun readSettings(serializedRequest: ByteArray): ByteArray =
        SlMessageBuilder.buildSettingsReadRequestRaw(serializedRequest, 0x10u)

    fun writeSettings(serializedRequest: ByteArray): ByteArray =
        SlMessageBuilder.buildSettingsWriteRequestRaw(serializedRequest, 0x10u)

    fun requestCameraFrame(serializedRequest: ByteArray): ByteArray =
        SlMessageBuilder.buildCameraFrameRequestRaw(serializedRequest, 0x10u)

    fun requestMap(serializedRequest: ByteArray): ByteArray =
        SlMessageBuilder.buildMapRequestRaw(serializedRequest, 0x10u)

    fun sendControl(serializedCommand: ByteArray): ByteArray =
        SlMessageBuilder.buildControlCommandRaw(serializedCommand, 0x10u)

    fun sendTaskConfig(serializedTaskConfig: ByteArray): ByteArray =
        SlMessageBuilder.buildTaskConfigRaw(serializedTaskConfig, 0x10u)

    fun sendTaskCommand(serializedTaskCommand: ByteArray): ByteArray =
        SlMessageBuilder.buildTaskCommandRaw(serializedTaskCommand, 0x10u)

    fun requestTaskPath(serializedTaskPathRequest: ByteArray): ByteArray =
        SlMessageBuilder.buildTaskPathRequestRaw(serializedTaskPathRequest, 0x10u)

    fun requestMapPreview(serializedMapPreviewRequest: ByteArray): ByteArray =
        SlMessageBuilder.buildMapPreviewRequestRaw(serializedMapPreviewRequest, 0x10u)

    fun sendMapEdit(serializedMapEditCommand: ByteArray): ByteArray =
        SlMessageBuilder.buildMapEditCommandRaw(serializedMapEditCommand, 0x10u)

    fun requestVideoStreamInfo(serializedRequest: ByteArray): ByteArray =
        SlMessageBuilder.buildVideoStreamInfoRequestRaw(serializedRequest, 0x10u)

    fun handleReceivedData(data: ByteArray) {
        val frames = parser.parse(data)
        for (frame in frames) {
            handleFrame(frame)
        }
    }

    private fun handleFrame(frame: SlFrame) {
        println("Received frame:")
        println("  MSG_ID: 0x${frame.msgId.toString(16).padStart(4, '0')}")
        println("  SRC_ID: 0x${frame.srcId.toString(16).padStart(2, '0')}")
        println("  DST_ID: 0x${frame.dstId.toString(16).padStart(2, '0')}")
        println("  SEQ: ${frame.seq}")

        val lost = monitor.update(frame.srcId, frame.seq)
        if (lost > 0) {
            println("  Lost $lost packet(s) from 0x${frame.srcId.toString(16)}")
        }

        when (frame.msgId.toInt()) {
            0x0202 -> handleWifiStatusReport(frame)
            0x0204 -> handleSettingsReadResponse(frame)
            0x0206 -> handleSettingsWriteResponse(frame)
            0x0301 -> handleDeviceStatusReport(frame)
            0x0303 -> handleCameraFrameChunk(frame)
            0x0305 -> handleMapChunk(frame)
            0x0402 -> handleControlCommandResponse(frame)
            0x0503 -> println("  TaskCommandResponse payload=${frame.payload.size} bytes")
            0x0504 -> println("  TaskStatusReport payload=${frame.payload.size} bytes")
            0x0506 -> println("  TaskPathChunk payload=${frame.payload.size} bytes")
            0x0508 -> println("  MapPreviewResponse payload=${frame.payload.size} bytes")
            0x050A -> println("  MapEditResponse payload=${frame.payload.size} bytes")
            0x050B -> println("  MapEditStatusReport payload=${frame.payload.size} bytes")
            0x050D -> println("  VideoStreamInfoResponse payload=${frame.payload.size} bytes")
            else -> println("  Unknown message type")
        }
    }

    private fun handleWifiStatusReport(frame: SlFrame) {
        val report = sl_link.SlLink.WifiStatusReport.parseFrom(frame.payload)
        println("  WiFi result=${report.result} message=${report.message}")
    }

    private fun handleSettingsReadResponse(frame: SlFrame) {
        val response = sl_link.SlLink.SettingsReadResponse.parseFrom(frame.payload)
        println(
            "  SettingsRead result=${response.result} " +
                "runSpeed=${response.chassis.runSpeed} " +
                "discRpm=${response.chassis.discSpeedRpm} " +
                "obstacles=${response.map.obstacleRegionsCount} " +
                "workRegions=${response.map.workRegionsCount}"
        )
    }

    private fun handleSettingsWriteResponse(frame: SlFrame) {
        val response = sl_link.SlLink.SettingsWriteResponse.parseFrom(frame.payload)
        println("  SettingsWrite result=${response.result} message=${response.message}")
    }

    private fun handleDeviceStatusReport(frame: SlFrame) {
        val report = sl_link.SlLink.DeviceStatusReport.parseFrom(frame.payload)
        println(
            "  Status mode=${report.workMode} left=${report.leftWheelSpeed} " +
                "right=${report.rightWheelSpeed} discRpm=${report.discSpeedRpm} " +
                "discOn=${report.discEnabled} lift=${report.discLiftState} " +
                "light=${report.lightEnabled} chassis=${report.chassisEnabled} " +
                "pos=(${report.position.x}, ${report.position.y}, ${report.position.headingDeg})"
        )
    }

    private fun handleCameraFrameChunk(frame: SlFrame) {
        val chunk = sl_link.SlLink.CameraFrameChunk.parseFrom(frame.payload)
        println(
            "  Camera chunk frameId=${chunk.frameId} ${chunk.chunkIndex + 1}/${chunk.totalChunks} " +
                "size=${chunk.data.size()} codec=${chunk.codec}"
        )
    }

    private fun handleMapChunk(frame: SlFrame) {
        val chunk = sl_link.SlLink.MapChunk.parseFrom(frame.payload)
        println(
            "  Map chunk mapId=${chunk.mapId} ${chunk.chunkIndex + 1}/${chunk.totalChunks} " +
                "size=${chunk.data.size()} encoding=${chunk.encoding}"
        )
    }

    private fun handleControlCommandResponse(frame: SlFrame) {
        val response = sl_link.SlLink.ControlCommandResponse.parseFrom(frame.payload)
        println("  Control result=${response.result} message=${response.message}")
    }

    fun printStats() {
        monitor.logStats()
    }
}
