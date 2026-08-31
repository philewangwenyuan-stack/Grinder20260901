#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "sl_nanopb.h"
#include "sl_link.pb.h"
#include "pb_decode.h"
#include "sl_frame.h"

static uint8_t sent_data[4096];
static uint16_t sent_len = 0;

bool sl_uart_send(const uint8_t* data, uint16_t len) {
    memcpy(sent_data, data, len);
    sent_len = len;
    return true;
}

void test_wifi_config_roundtrip(void) {
    sl_link_WifiConfig config = sl_link_WifiConfig_init_zero;
    strncpy(config.ssid, "demo-ssid", sizeof(config.ssid) - 1);
    strncpy(config.password, "demo-password", sizeof(config.password) - 1);

    sl_set_source_id(sl_link_DeviceId_DEVICE_APP);
    bool ok = sl_send_message(
        sl_link_MessageId_MSG_ID_WIFI_CONFIG,
        &config,
        sl_link_WifiConfig_fields,
        sl_link_DeviceId_DEVICE_LOWER,
        sl_link_ComponentId_COMP_WIFI,
        0,
        0
    );
    assert(ok);

    sl_frame_header_t* header = (sl_frame_header_t*)sent_data;
    assert(header->msg_id == sl_link_MessageId_MSG_ID_WIFI_CONFIG);

    sl_link_WifiConfig decoded = sl_link_WifiConfig_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(sent_data + 15, header->len);
    ok = pb_decode(&stream, sl_link_WifiConfig_fields, &decoded);
    assert(ok);
    assert(strcmp(decoded.ssid, "demo-ssid") == 0);
}

void test_settings_write_roundtrip(void) {
    sl_link_SettingsWriteRequest req = sl_link_SettingsWriteRequest_init_zero;
    req.has_chassis = true;
    req.chassis.run_speed = 0.8f;
    req.chassis.disc_speed_rpm = 1200;
    req.chassis.disc_enabled = true;
    req.chassis.work_mode = sl_link_WorkMode_WORK_MODE_AUTO;

    req.has_map = true;
    req.map.vehicle_width = 0.72f;
    req.map.vehicle_length = 1.10f;
    req.map.obstacle_regions_count = 1;
    strncpy(req.map.obstacle_regions[0].name, "obstacle_region_1", sizeof(req.map.obstacle_regions[0].name) - 1);
    req.map.obstacle_regions[0].points_count = 4;

    sl_set_source_id(sl_link_DeviceId_DEVICE_APP);
    bool ok = sl_send_message(
        sl_link_MessageId_MSG_ID_SETTINGS_WRITE_REQUEST,
        &req,
        sl_link_SettingsWriteRequest_fields,
        sl_link_DeviceId_DEVICE_LOWER,
        sl_link_ComponentId_COMP_SETTINGS,
        0,
        0
    );
    assert(ok);

    sl_frame_header_t* header = (sl_frame_header_t*)sent_data;
    assert(header->msg_id == sl_link_MessageId_MSG_ID_SETTINGS_WRITE_REQUEST);

    sl_link_SettingsWriteRequest decoded = sl_link_SettingsWriteRequest_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(sent_data + 15, header->len);
    ok = pb_decode(&stream, sl_link_SettingsWriteRequest_fields, &decoded);
    assert(ok);
    assert(decoded.has_chassis);
    assert(decoded.chassis.disc_speed_rpm == 1200);
    assert(decoded.has_map);
    assert(decoded.map.obstacle_regions_count == 1);
}

void test_control_command_chassis_power_roundtrip(void) {
    sl_link_ControlCommand command = sl_link_ControlCommand_init_zero;
    command.which_command = sl_link_ControlCommand_chassis_power_tag;
    command.command.chassis_power.enabled = true;

    sl_set_source_id(sl_link_DeviceId_DEVICE_APP);
    bool ok = sl_send_message(
        sl_link_MessageId_MSG_ID_CONTROL_COMMAND,
        &command,
        sl_link_ControlCommand_fields,
        sl_link_DeviceId_DEVICE_LOWER,
        sl_link_ComponentId_COMP_CONTROL,
        0,
        0
    );
    assert(ok);

    sl_frame_header_t* header = (sl_frame_header_t*)sent_data;
    assert(header->msg_id == sl_link_MessageId_MSG_ID_CONTROL_COMMAND);

    sl_link_ControlCommand decoded = sl_link_ControlCommand_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(sent_data + 15, header->len);
    ok = pb_decode(&stream, sl_link_ControlCommand_fields, &decoded);
    assert(ok);
    assert(decoded.which_command == sl_link_ControlCommand_chassis_power_tag);
    assert(decoded.command.chassis_power.enabled);
}

void test_device_status_report_roundtrip(void) {
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
    bool ok = sl_send_message(
        sl_link_MessageId_MSG_ID_DEVICE_STATUS_REPORT,
        &report,
        sl_link_DeviceStatusReport_fields,
        sl_link_DeviceId_DEVICE_APP,
        sl_link_ComponentId_COMP_SYSTEM,
        0,
        0
    );
    assert(ok);

    sl_frame_header_t* header = (sl_frame_header_t*)sent_data;
    assert(header->msg_id == sl_link_MessageId_MSG_ID_DEVICE_STATUS_REPORT);

    sl_link_DeviceStatusReport decoded = sl_link_DeviceStatusReport_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(sent_data + 15, header->len);
    ok = pb_decode(&stream, sl_link_DeviceStatusReport_fields, &decoded);
    assert(ok);
    assert(decoded.left_wheel_speed > 0.4f);
    assert(decoded.chassis_enabled);
    assert(decoded.has_position);
}

void test_ack(void) {
    uint16_t ack_seq = 0x1234;
    sl_set_source_id(sl_link_DeviceId_DEVICE_LOWER);
    bool ok = sl_send_ack(
        sl_link_MessageId_MSG_ID_SETTINGS_WRITE_RESPONSE,
        sl_link_DeviceId_DEVICE_APP,
        sl_link_ComponentId_COMP_SETTINGS,
        ack_seq
    );
    assert(ok);

    sl_frame_header_t* header = (sl_frame_header_t*)sent_data;
    assert(header->msg_id == sl_link_MessageId_MSG_ID_SETTINGS_WRITE_RESPONSE);
    assert(header->ack_seq == ack_seq);
    assert(header->src_id == sl_link_DeviceId_DEVICE_LOWER);
    assert(header->len == 0);
}

int main(void) {
    printf("Running NanoPb expanded protocol tests...\n");
    test_wifi_config_roundtrip();
    test_settings_write_roundtrip();
    test_control_command_chassis_power_roundtrip();
    test_device_status_report_roundtrip();
    test_ack();
    printf("NanoPb tests passed!\n");
    return 0;
}
