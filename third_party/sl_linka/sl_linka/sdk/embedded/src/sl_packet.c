#include "sl_packet.h"
#include "sl_crc16.h"
#include <string.h>
#include <stdlib.h>

void sl_parser_init(sl_parser_context_t* ctx, uint8_t* payload_buffer, uint16_t buffer_size) {
    memset(ctx, 0, sizeof(sl_parser_context_t));
    ctx->state = SL_PARSE_WAIT_STX1;
    ctx->payload_buf = payload_buffer;
    ctx->payload_capacity = buffer_size;
}

void sl_parser_reset(sl_parser_context_t* ctx) {
    ctx->state = SL_PARSE_WAIT_STX1;
    ctx->header_index = 0;
    ctx->payload_index = 0;
    ctx->crc_index = 0;
}

static void copy_header_to_struct(sl_parser_context_t* ctx) {
    // Parse header from buffer (Little Endian)
    uint8_t* buf = ctx->header_buf;
    ctx->current_header.stx1 = buf[0];
    ctx->current_header.stx2 = buf[1];
    ctx->current_header.version = buf[2];
    ctx->current_header.flags = buf[3];
    ctx->current_header.seq = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    ctx->current_header.ack_seq = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
    ctx->current_header.src_id = buf[8];
    ctx->current_header.dst_id = buf[9];
    ctx->current_header.comp_id = buf[10];
    ctx->current_header.msg_id = (uint16_t)buf[11] | ((uint16_t)buf[12] << 8);
    ctx->current_header.len = (uint16_t)buf[13] | ((uint16_t)buf[14] << 8);
}

bool sl_parse_byte(sl_parser_context_t* ctx, uint8_t byte) {
    switch (ctx->state) {
        case SL_PARSE_WAIT_STX1:
            if (byte == SL_STX1) {
                ctx->header_buf[0] = byte;
                ctx->header_index = 1;
                ctx->state = SL_PARSE_WAIT_STX2;
            }
            break;
            
        case SL_PARSE_WAIT_STX2:
            if (byte == SL_STX2) {
                ctx->header_buf[1] = byte;
                ctx->header_index = 2;
                ctx->state = SL_PARSE_HEADER;
            } else {
                sl_parser_reset(ctx);
            }
            break;
            
        case SL_PARSE_HEADER:
            ctx->header_buf[ctx->header_index++] = byte;
            
            if (ctx->header_index >= SL_FRAME_HEADER_SIZE) {
                // Header complete, parse it
                copy_header_to_struct(ctx);
                
                // Validate payload length
                if (ctx->current_header.len > SL_MAX_PAYLOAD_SIZE) {
                    sl_parser_reset(ctx);
                    break;
                }
                
                // Check if we have enough buffer space
                if (ctx->payload_buf == NULL || ctx->current_header.len > ctx->payload_capacity) {
                    // Need dynamic allocation or buffer too small - skip this frame
                    sl_parser_reset(ctx);
                    break;
                }
                
                ctx->payload_size = ctx->current_header.len;
                ctx->payload_index = 0;
                
                if (ctx->payload_size == 0) {
                    // No payload, go directly to CRC
                    ctx->state = SL_PARSE_CRC;
                    ctx->crc_index = 0;
                } else {
                    ctx->state = SL_PARSE_PAYLOAD;
                }
            }
            break;
            
        case SL_PARSE_PAYLOAD:
            ctx->payload_buf[ctx->payload_index++] = byte;
            
            if (ctx->payload_index >= ctx->payload_size) {
                // Payload complete, move to CRC
                ctx->state = SL_PARSE_CRC;
                ctx->crc_index = 0;
            }
            break;
            
        case SL_PARSE_CRC:
            // Receive CRC16 (Little Endian, 2 bytes)
            if (ctx->crc_index == 0) {
                ctx->crc_received = byte;
                ctx->crc_index = 1;
            } else {
                ctx->crc_received |= ((uint16_t)byte << 8);
                
                // Calculate CRC over header + payload
                uint16_t crc_calc = sl_crc16_calculate(ctx->header_buf, SL_FRAME_HEADER_SIZE);
                if (ctx->payload_size > 0) {
                    // Continue CRC calculation with payload
                    uint16_t temp_crc = crc_calc;
                    for (uint16_t i = 0; i < ctx->payload_size; i++) {
                        uint8_t data = ctx->payload_buf[i];
                        temp_crc ^= (uint16_t)data << 8;
                        for (uint8_t j = 0; j < 8; j++) {
                            if (temp_crc & 0x8000) {
                                temp_crc = (temp_crc << 1) ^ 0x1021;
                            } else {
                                temp_crc = temp_crc << 1;
                            }
                        }
                    }
                    crc_calc = temp_crc;
                }
                
                if (crc_calc == ctx->crc_received) {
                    ctx->state = SL_PARSE_TAIL;
                } else {
                    // CRC mismatch, reset
                    sl_parser_reset(ctx);
                }
            }
            break;
            
        case SL_PARSE_TAIL:
            if (byte == SL_TAIL) {
                // Frame complete and valid!
                sl_frame_received(&ctx->current_header, ctx->payload_buf, ctx->payload_size);
                sl_parser_reset(ctx);
                return true;
            } else {
                // Invalid tail, reset
                sl_parser_reset(ctx);
            }
            break;
    }
    
    return false;
}
