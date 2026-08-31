#include "sl_nanopb.h"
#include <stdlib.h>
#include "sl_crc16.h"
#include "pb_encode.h"

// Internal state
static uint16_t g_seq = 0;
static sl_frame_header_t g_header_template = {
    .stx1 = SL_STX1,
    .stx2 = SL_STX2,
    .version = SL_PROTOCOL_VERSION,
    .flags = 0,
    .seq = 0,
    .ack_seq = 0,
    .src_id = 0x01, // Default source ID
    .dst_id = 0,
    .comp_id = 0,
    .msg_id = 0,
    .len = 0
};

// Static buffer to avoid malloc (max size: header + max payload + crc + tail)
// Note: Adjusted to a reasonable default or can be defined by user
#ifndef SL_TX_BUFFER_SIZE
#define SL_TX_BUFFER_SIZE 1024
#endif
static uint8_t g_tx_buffer[SL_TX_BUFFER_SIZE];

uint16_t sl_get_next_seq(void) {
    return g_seq++;
}

void sl_set_source_id(uint8_t src_id) {
    g_header_template.src_id = src_id;
}

uint8_t sl_get_source_id(void) {
    return g_header_template.src_id;
}

bool sl_send_message(uint16_t msg_id,
                    const void* pb_struct,
                    const pb_msgdesc_t* fields,
                    uint8_t dst_id,
                    uint8_t comp_id,
                    uint8_t flags,
                    uint16_t ack_seq) {
    
    // 1. Encode protobuf payload directly into the tx buffer (after header space)
    uint8_t *payload_ptr = &g_tx_buffer[SL_FRAME_HEADER_SIZE];
    uint16_t max_payload = SL_TX_BUFFER_SIZE - SL_FRAME_HEADER_SIZE - 3;
    uint16_t payload_len = 0;

    if (pb_struct && fields) {
        pb_ostream_t stream = pb_ostream_from_buffer(payload_ptr, max_payload);
        if (!pb_encode(&stream, fields, pb_struct)) {
            return false;
        }
        payload_len = (uint16_t)stream.bytes_written;
    }

    // 2. Update and serialized header directly into the tx buffer
    uint16_t seq = sl_get_next_seq();
    
    uint8_t *h = g_tx_buffer;
    *h++ = g_header_template.stx1;
    *h++ = g_header_template.stx2;
    *h++ = g_header_template.version;
    *h++ = flags;
    *h++ = (uint8_t)(seq & 0xFF);
    *h++ = (uint8_t)((seq >> 8) & 0xFF);
    *h++ = (uint8_t)(ack_seq & 0xFF);
    *h++ = (uint8_t)((ack_seq >> 8) & 0xFF);
    *h++ = g_header_template.src_id;
    *h++ = dst_id;
    *h++ = comp_id;
    *h++ = (uint8_t)(msg_id & 0xFF);
    *h++ = (uint8_t)((msg_id >> 8) & 0xFF);
    *h++ = (uint8_t)(payload_len & 0xFF);
    *h++ = (uint8_t)((payload_len >> 8) & 0xFF);

    // 3. Calculate CRC over header and payload
    uint16_t crc_input_len = SL_FRAME_HEADER_SIZE + payload_len;
    uint16_t crc = sl_crc16_calculate(g_tx_buffer, crc_input_len);

    // 4. Append CRC and TAIL
    uint8_t *p = &g_tx_buffer[crc_input_len];
    *p++ = (uint8_t)(crc & 0xFF);
    *p++ = (uint8_t)((crc >> 8) & 0xFF);
    *p++ = SL_TAIL;

    // 5. Send via UART
    return sl_uart_send(g_tx_buffer, (uint16_t)(crc_input_len + 3));
}

bool sl_send_ack(uint16_t msg_id, uint8_t dst_id, uint8_t comp_id, uint16_t ack_seq) {
    return sl_send_message(msg_id, NULL, NULL, dst_id, comp_id, 0, ack_seq);
}
