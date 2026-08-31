#include "sl_frame.h"
#include "sl_packet.h"
#include "sl_nanopb.h"
#include "sl_crc16.h"
#include "sl_link_monitor.h"
#include "sl_link.pb.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "pb_decode.h"

static sl_link_monitor_t g_monitor;

bool sl_uart_send(const uint8_t* data, uint16_t len) {
    printf("[UART] len=%u data=", len);
    for (uint16_t i = 0; i < len; ++i) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    return true;
}

static void print_control_command(const sl_link_ControlCommand* command) {
    switch (command->which_command) {
        case sl_link_ControlCommand_disc_lift_tag:
            printf("disc_lift=%d", command->command.disc_lift.command);
            break;
        case sl_link_ControlCommand_lighting_tag:
            printf("lighting=%d", command->command.lighting.enabled);
            break;
        case sl_link_ControlCommand_manual_drive_tag:
            printf(
                "manual motion=%d speed_ratio=%.2f",
                command->command.manual_drive.motion,
                command->command.manual_drive.speed_ratio
            );
            break;
        case sl_link_ControlCommand_chassis_power_tag:
            printf("chassis_power=%d", command->command.chassis_power.enabled);
            break;
        default:
            printf("unknown");
            break;
    }
}

void sl_frame_received(const sl_frame_header_t* header, const uint8_t* payload, uint16_t len) {
    uint16_t lost = sl_link_monitor_update(&g_monitor, header->src_id, header->seq);
    if (lost > 0) {
        printf("[MONITOR] lost=%u from 0x%02X\n", lost, header->src_id);
    }

    switch (header->msg_id) {
        case sl_link_MessageId_MSG_ID_WIFI_CONFIG: {
            sl_link_WifiConfig cfg = sl_link_WifiConfig_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_WifiConfig_fields, &cfg)) {
                printf("[RX] WifiConfig ssid=%s\n", cfg.ssid);
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_SETTINGS_READ_REQUEST: {
            sl_link_SettingsReadRequest req = sl_link_SettingsReadRequest_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_SettingsReadRequest_fields, &req)) {
                printf("[RX] SettingsReadRequest chassis=%d map=%d\n", req.read_chassis, req.read_map);
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_SETTINGS_WRITE_REQUEST: {
            sl_link_SettingsWriteRequest req = sl_link_SettingsWriteRequest_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_SettingsWriteRequest_fields, &req)) {
                printf(
                    "[RX] SettingsWriteRequest run_speed=%.2f disc_rpm=%u obstacle_count=%u work_count=%u\n",
                    req.chassis.run_speed,
                    req.chassis.disc_speed_rpm,
                    req.map.obstacle_regions_count,
                    req.map.work_regions_count
                );
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_DEVICE_STATUS_REPORT: {
            sl_link_DeviceStatusReport report = sl_link_DeviceStatusReport_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_DeviceStatusReport_fields, &report)) {
                printf(
                    "[RX] DeviceStatusReport left=%.2f right=%.2f disc_rpm=%u disc_on=%d lift=%d light=%d chassis=%d\n",
                    report.left_wheel_speed,
                    report.right_wheel_speed,
                    report.disc_speed_rpm,
                    report.disc_enabled,
                    report.disc_lift_state,
                    report.light_enabled,
                    report.chassis_enabled
                );
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_CAMERA_FRAME_CHUNK: {
            sl_link_CameraFrameChunk chunk = sl_link_CameraFrameChunk_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_CameraFrameChunk_fields, &chunk)) {
                printf("[RX] CameraFrameChunk frame=%u chunk=%u/%u bytes=%u\n",
                    chunk.frame_id,
                    chunk.chunk_index + 1,
                    chunk.total_chunks,
                    chunk.data.size);
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_MAP_CHUNK: {
            sl_link_MapChunk chunk = sl_link_MapChunk_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_MapChunk_fields, &chunk)) {
                printf("[RX] MapChunk map=%u chunk=%u/%u bytes=%u encoding=%d\n",
                    chunk.map_id,
                    chunk.chunk_index + 1,
                    chunk.total_chunks,
                    chunk.data.size,
                    chunk.encoding);
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_CONTROL_COMMAND: {
            sl_link_ControlCommand command = sl_link_ControlCommand_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_ControlCommand_fields, &command)) {
                printf("[RX] ControlCommand ");
                print_control_command(&command);
                printf("\n");
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_CONTROL_COMMAND_RESPONSE: {
            sl_link_ControlCommandResponse response = sl_link_ControlCommandResponse_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_ControlCommandResponse_fields, &response)) {
                printf("[RX] ControlCommandResponse result=%d message=%s\n",
                    response.result,
                    response.message);
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_TASK_CONFIG: {
            sl_link_TaskConfig task = sl_link_TaskConfig_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_TaskConfig_fields, &task)) {
                printf("[RX] TaskConfig task_id=%s work_regions=%u obstacle_regions=%u spacing=%.2f\n",
                    task.task_id,
                    task.work_regions_count,
                    task.obstacle_regions_count,
                    task.default_path_spacing);
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_TASK_COMMAND: {
            sl_link_TaskCommand command = sl_link_TaskCommand_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_TaskCommand_fields, &command)) {
                printf("[RX] TaskCommand task_id=%s command=%d\n", command.task_id, command.command);
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_TASK_STATUS_REPORT: {
            sl_link_TaskStatusReport report = sl_link_TaskStatusReport_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_TaskStatusReport_fields, &report)) {
                printf("[RX] TaskStatus task_id=%s state=%d progress=%.2f map_version=%u path_version=%u\n",
                    report.task_id,
                    report.state,
                    report.progress,
                    report.map_version,
                    report.path_version);
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_MAP_PREVIEW_RESPONSE: {
            sl_link_MapPreviewResponse response = sl_link_MapPreviewResponse_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_MapPreviewResponse_fields, &response)) {
                printf("[RX] MapPreviewResponse result=%d map_version=%u image_bytes=%u overlay_len=%zu\n",
                    response.result,
                    response.map_version,
                    response.image_data.size,
                    strlen(response.overlay_json));
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_MAP_EDIT_RESPONSE: {
            sl_link_MapEditResponse response = sl_link_MapEditResponse_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_MapEditResponse_fields, &response)) {
                printf("[RX] MapEditResponse result=%d map_version=%u message=%s\n",
                    response.result,
                    response.map_version,
                    response.message);
            }
            break;
        }
        case sl_link_MessageId_MSG_ID_VIDEO_STREAM_INFO_RESPONSE: {
            sl_link_VideoStreamInfoResponse response = sl_link_VideoStreamInfoResponse_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(payload, len);
            if (pb_decode(&stream, sl_link_VideoStreamInfoResponse_fields, &response)) {
                printf("[RX] VideoStreamInfoResponse online=%d url=%s codec=%s %ux%u\n",
                    response.online,
                    response.stream_url,
                    response.codec,
                    response.width,
                    response.height);
            }
            break;
        }
        default:
            printf("[RX] Unknown msg_id=0x%04X\n", header->msg_id);
            break;
    }
}

static void example_wifi_config(void) {
    sl_link_WifiConfig cfg = sl_link_WifiConfig_init_zero;
    strncpy(cfg.ssid, "MyHotspot", sizeof(cfg.ssid) - 1);
    strncpy(cfg.password, "secret123", sizeof(cfg.password) - 1);

    sl_set_source_id(sl_link_DeviceId_DEVICE_APP);
    sl_send_message(
        sl_link_MessageId_MSG_ID_WIFI_CONFIG,
        &cfg,
        sl_link_WifiConfig_fields,
        sl_link_DeviceId_DEVICE_LOWER,
        sl_link_ComponentId_COMP_WIFI,
        0,
        0
    );
}

static void example_settings_read_request(void) {
    sl_link_SettingsReadRequest req = sl_link_SettingsReadRequest_init_zero;
    req.read_chassis = true;
    req.read_map = true;

    sl_set_source_id(sl_link_DeviceId_DEVICE_APP);
    sl_send_message(
        sl_link_MessageId_MSG_ID_SETTINGS_READ_REQUEST,
        &req,
        sl_link_SettingsReadRequest_fields,
        sl_link_DeviceId_DEVICE_LOWER,
        sl_link_ComponentId_COMP_SETTINGS,
        0,
        0
    );
}

static void example_settings_write_request(void) {
    sl_link_SettingsWriteRequest req = sl_link_SettingsWriteRequest_init_zero;
    req.has_chassis = true;
    req.chassis.run_speed = 0.8f;
    req.chassis.disc_speed_rpm = 1200;
    req.chassis.disc_enabled = true;
    req.chassis.work_mode = sl_link_WorkMode_WORK_MODE_AUTO;

    req.has_map = true;
    req.map.vehicle_width = 0.72f;
    req.map.vehicle_length = 1.10f;
    req.map.default_path_spacing = 0.30f;
    req.map.turn_radius = 0.45f;
    req.map.overlap_ratio = 0.15f;
    req.map.inflation_radius = 0.12f;

    req.map.obstacle_regions_count = 1;
    strncpy(req.map.obstacle_regions[0].name, "obstacle_region_1", sizeof(req.map.obstacle_regions[0].name) - 1);
    req.map.obstacle_regions[0].points_count = 4;
    req.map.obstacle_regions[0].points[0].x = 1.0f; req.map.obstacle_regions[0].points[0].y = 1.0f;
    req.map.obstacle_regions[0].points[1].x = 1.4f; req.map.obstacle_regions[0].points[1].y = 1.0f;
    req.map.obstacle_regions[0].points[2].x = 1.4f; req.map.obstacle_regions[0].points[2].y = 1.4f;
    req.map.obstacle_regions[0].points[3].x = 1.0f; req.map.obstacle_regions[0].points[3].y = 1.4f;

    req.map.work_regions_count = 1;
    strncpy(req.map.work_regions[0].name, "work_region_1", sizeof(req.map.work_regions[0].name) - 1);
    req.map.work_regions[0].points_count = 4;
    req.map.work_regions[0].points[0].x = 0.0f; req.map.work_regions[0].points[0].y = 0.0f;
    req.map.work_regions[0].points[1].x = 4.0f; req.map.work_regions[0].points[1].y = 0.0f;
    req.map.work_regions[0].points[2].x = 4.0f; req.map.work_regions[0].points[2].y = 3.0f;
    req.map.work_regions[0].points[3].x = 0.0f; req.map.work_regions[0].points[3].y = 3.0f;

    sl_set_source_id(sl_link_DeviceId_DEVICE_APP);
    sl_send_message(
        sl_link_MessageId_MSG_ID_SETTINGS_WRITE_REQUEST,
        &req,
        sl_link_SettingsWriteRequest_fields,
        sl_link_DeviceId_DEVICE_LOWER,
        sl_link_ComponentId_COMP_SETTINGS,
        0,
        0
    );
}

static void example_device_status_report(void) {
    sl_link_DeviceStatusReport report = sl_link_DeviceStatusReport_init_zero;
    report.utc_time = 1710000000;
    report.system_status = sl_link_SystemStatus_SYS_STATUS_NORMAL;
    report.wifi_status = sl_link_WifiResult_WIFI_SUCCESS;
    report.work_mode = sl_link_WorkMode_WORK_MODE_AUTO;
    report.left_wheel_speed = 0.42f;
    report.right_wheel_speed = 0.40f;
    report.disc_speed_rpm = 1200;
    report.disc_enabled = true;
    report.disc_lift_state = sl_link_DiscLiftState_DISC_LIFT_STATE_DOWN;
    report.light_enabled = true;
    report.has_position = true;
    report.position.x = 1.25f;
    report.position.y = 2.50f;
    report.position.heading_deg = 90.0f;
    report.chassis_enabled = true;

    sl_set_source_id(sl_link_DeviceId_DEVICE_LOWER);
    sl_send_message(
        sl_link_MessageId_MSG_ID_DEVICE_STATUS_REPORT,
        &report,
        sl_link_DeviceStatusReport_fields,
        sl_link_DeviceId_DEVICE_APP,
        sl_link_ComponentId_COMP_SYSTEM,
        0,
        0
    );
}

static void example_camera_frame_chunk(void) {
    sl_link_CameraFrameChunk chunk = sl_link_CameraFrameChunk_init_zero;
    const char* data = "demo-camera-frame-jpeg";
    size_t n = strlen(data);
    chunk.frame_id = 101;
    chunk.utc_time = 1710000100;
    chunk.width = 640;
    chunk.height = 480;
    chunk.codec = sl_link_CameraCodec_CAMERA_CODEC_JPEG;
    chunk.chunk_index = 0;
    chunk.total_chunks = 1;
    if (n > sizeof(chunk.data.bytes)) n = sizeof(chunk.data.bytes);
    chunk.data.size = (pb_size_t)n;
    memcpy(chunk.data.bytes, data, n);

    sl_set_source_id(sl_link_DeviceId_DEVICE_LOWER);
    sl_send_message(
        sl_link_MessageId_MSG_ID_CAMERA_FRAME_CHUNK,
        &chunk,
        sl_link_CameraFrameChunk_fields,
        sl_link_DeviceId_DEVICE_APP,
        sl_link_ComponentId_COMP_MEDIA,
        0,
        0
    );
}

static void example_map_chunk(void) {
    sl_link_MapChunk chunk = sl_link_MapChunk_init_zero;
    const char* data = "{\"map\":\"demo\",\"regions\":2}";
    size_t n = strlen(data);
    chunk.map_id = 7;
    chunk.utc_time = 1710000200;
    chunk.encoding = sl_link_MapEncoding_MAP_ENCODING_JSON;
    chunk.width = 200;
    chunk.height = 120;
    chunk.resolution = 0.05f;
    chunk.chunk_index = 0;
    chunk.total_chunks = 1;
    if (n > sizeof(chunk.data.bytes)) n = sizeof(chunk.data.bytes);
    chunk.data.size = (pb_size_t)n;
    memcpy(chunk.data.bytes, data, n);

    sl_set_source_id(sl_link_DeviceId_DEVICE_LOWER);
    sl_send_message(
        sl_link_MessageId_MSG_ID_MAP_CHUNK,
        &chunk,
        sl_link_MapChunk_fields,
        sl_link_DeviceId_DEVICE_APP,
        sl_link_ComponentId_COMP_MEDIA,
        0,
        0
    );
}

static void example_control_commands(void) {
    sl_link_ControlCommand command = sl_link_ControlCommand_init_zero;

    command.which_command = sl_link_ControlCommand_chassis_power_tag;
    command.command.chassis_power.enabled = true;
    sl_set_source_id(sl_link_DeviceId_DEVICE_APP);
    sl_send_message(
        sl_link_MessageId_MSG_ID_CONTROL_COMMAND,
        &command,
        sl_link_ControlCommand_fields,
        sl_link_DeviceId_DEVICE_LOWER,
        sl_link_ComponentId_COMP_CONTROL,
        0,
        0
    );

    command = (sl_link_ControlCommand)sl_link_ControlCommand_init_zero;
    command.which_command = sl_link_ControlCommand_manual_drive_tag;
    command.command.manual_drive.motion = sl_link_ManualMotionCommand_MANUAL_MOTION_FORWARD_LEFT;
    command.command.manual_drive.speed_ratio = 0.7f;
    sl_send_message(
        sl_link_MessageId_MSG_ID_CONTROL_COMMAND,
        &command,
        sl_link_ControlCommand_fields,
        sl_link_DeviceId_DEVICE_LOWER,
        sl_link_ComponentId_COMP_CONTROL,
        0,
        0
    );
}

static void example_control_response(void) {
    sl_link_ControlCommandResponse response = sl_link_ControlCommandResponse_init_zero;
    response.result = sl_link_ResultCode_RESULT_SUCCESS;
    strncpy(response.message, "Control applied", sizeof(response.message) - 1);
    response.has_applied_command = true;
    response.applied_command.which_command = sl_link_ControlCommand_chassis_power_tag;
    response.applied_command.command.chassis_power.enabled = true;

    sl_set_source_id(sl_link_DeviceId_DEVICE_LOWER);
    sl_send_message(
        sl_link_MessageId_MSG_ID_CONTROL_COMMAND_RESPONSE,
        &response,
        sl_link_ControlCommandResponse_fields,
        sl_link_DeviceId_DEVICE_APP,
        sl_link_ComponentId_COMP_CONTROL,
        0,
        0
    );
}

static void example_task_config(void) {
    sl_link_TaskConfig task = sl_link_TaskConfig_init_zero;
    strncpy(task.task_id, "demo_task_001", sizeof(task.task_id) - 1);
    task.default_path_spacing = 1.2f;
    task.turn_radius = 0.8f;
    task.overlap_ratio = 0.1f;
    task.inflation_radius = 0.65f;

    task.work_regions_count = 1;
    strncpy(task.work_regions[0].name, "work_region_1", sizeof(task.work_regions[0].name) - 1);
    task.work_regions[0].points_count = 4;
    task.work_regions[0].points[0].x = 0.0f; task.work_regions[0].points[0].y = 0.0f;
    task.work_regions[0].points[1].x = 6.0f; task.work_regions[0].points[1].y = 0.0f;
    task.work_regions[0].points[2].x = 6.0f; task.work_regions[0].points[2].y = 4.0f;
    task.work_regions[0].points[3].x = 0.0f; task.work_regions[0].points[3].y = 4.0f;

    sl_set_source_id(sl_link_DeviceId_DEVICE_APP);
    sl_send_message(
        sl_link_MessageId_MSG_ID_TASK_CONFIG,
        &task,
        sl_link_TaskConfig_fields,
        sl_link_DeviceId_DEVICE_LOWER,
        sl_link_ComponentId_COMP_SCHEDULER,
        0,
        0
    );
}

static void example_task_command(void) {
    sl_link_TaskCommand command = sl_link_TaskCommand_init_zero;
    strncpy(command.task_id, "demo_task_001", sizeof(command.task_id) - 1);
    command.command = sl_link_TaskCommandType_TASK_CMD_START;

    sl_set_source_id(sl_link_DeviceId_DEVICE_APP);
    sl_send_message(
        sl_link_MessageId_MSG_ID_TASK_COMMAND,
        &command,
        sl_link_TaskCommand_fields,
        sl_link_DeviceId_DEVICE_LOWER,
        sl_link_ComponentId_COMP_SCHEDULER,
        0,
        0
    );
}

static void example_map_preview_request(void) {
    sl_link_MapPreviewRequest request = sl_link_MapPreviewRequest_init_zero;
    request.max_edge = 512;
    strncpy(request.image_format, "jpg", sizeof(request.image_format) - 1);
    request.include_overlay = true;

    sl_set_source_id(sl_link_DeviceId_DEVICE_APP);
    sl_send_message(
        sl_link_MessageId_MSG_ID_MAP_PREVIEW_REQUEST,
        &request,
        sl_link_MapPreviewRequest_fields,
        sl_link_DeviceId_DEVICE_LOWER,
        sl_link_ComponentId_COMP_SCHEDULER,
        0,
        0
    );
}

static void example_video_stream_info_request(void) {
    sl_link_VideoStreamInfoRequest request = sl_link_VideoStreamInfoRequest_init_zero;
    request.include_debug = false;

    sl_set_source_id(sl_link_DeviceId_DEVICE_APP);
    sl_send_message(
        sl_link_MessageId_MSG_ID_VIDEO_STREAM_INFO_REQUEST,
        &request,
        sl_link_VideoStreamInfoRequest_fields,
        sl_link_DeviceId_DEVICE_LOWER,
        sl_link_ComponentId_COMP_SCHEDULER,
        0,
        0
    );
}

static void example_send_ack(void) {
    sl_set_source_id(sl_link_DeviceId_DEVICE_LOWER);
    sl_send_ack(
        sl_link_MessageId_MSG_ID_SETTINGS_WRITE_RESPONSE,
        sl_link_DeviceId_DEVICE_APP,
        sl_link_ComponentId_COMP_SETTINGS,
        10
    );
}

static void example_crc16_usage(void) {
    uint8_t data[] = { 0x01, 0x02, 0x03, 0x04 };
    uint16_t crc = sl_crc16_calculate(data, sizeof(data));
    printf("[CRC16] 0x%04X\n", crc);
}

int main(void) {
    printf("SL-LinkA expanded C example\n");
    sl_link_monitor_init(&g_monitor);

    example_wifi_config();
    example_settings_read_request();
    example_settings_write_request();
    example_device_status_report();
    example_camera_frame_chunk();
    example_map_chunk();
    example_control_commands();
    example_control_response();
    example_task_config();
    example_task_command();
    example_map_preview_request();
    example_video_stream_info_request();
    example_send_ack();
    example_crc16_usage();

    return 0;
}
