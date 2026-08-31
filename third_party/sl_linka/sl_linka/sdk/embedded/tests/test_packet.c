#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "sl_packet.h"
#include "sl_frame.h"
#include "sl_crc16.h"

static bool frame_received_called = false;
static sl_frame_header_t last_header;
static uint8_t last_payload[1024];
static uint16_t last_payload_len = 0;

void sl_frame_received(const sl_frame_header_t* header, const uint8_t* payload, uint16_t len) {
    frame_received_called = true;
    last_header = *header;
    last_payload_len = len;
    memcpy(last_payload, payload, len);
}

// Dummy for linking if needed
bool sl_uart_send(const uint8_t* data, uint16_t len) { return true; }

void test_packet_parsing(void) {
    uint8_t buffer[1024];
    sl_parser_context_t parser;
    sl_parser_init(&parser, buffer, sizeof(buffer));

    // Construct a valid frame
    uint8_t frame[64];
    size_t i = 0;
    frame[i++] = 0xFD; // STX1
    frame[i++] = 0x55; // STX2
    frame[i++] = 0x01; // VER
    frame[i++] = 0x00; // FLAGS
    frame[i++] = 0x01; frame[i++] = 0x00; // SEQ
    frame[i++] = 0x00; frame[i++] = 0x00; // ACK_SEQ
    frame[i++] = 0x01; // SRC
    frame[i++] = 0x02; // DST
    frame[i++] = 0x03; // COMP
    frame[i++] = 0x04; frame[i++] = 0x00; // MSG_ID = 0x0004
    frame[i++] = 0x02; frame[i++] = 0x00; // LEN = 2
    frame[i++] = 0xAA; frame[i++] = 0xBB; // PAYLOAD
    
    uint16_t crc = sl_crc16_calculate(frame, i);
    frame[i++] = (uint8_t)(crc & 0xFF);
    frame[i++] = (uint8_t)((crc >> 8) & 0xFF);
    frame[i++] = 0xFE; // TAIL

    frame_received_called = false;
    for (size_t j = 0; j < i; j++) {
        sl_parse_byte(&parser, frame[j]);
    }

    assert(frame_received_called);
    assert(last_header.msg_id == 0x0004);
    assert(last_payload_len == 2);
    assert(last_payload[0] == 0xAA);
    assert(last_payload[1] == 0xBB);
}

int main(void) {
    printf("Running Packet parser tests...\n");
    test_packet_parsing();
    printf("Packet parser tests passed!\n");
    return 0;
}
